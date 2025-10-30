<template>
  <div id="app">
    <div class="ui-overlay">
      <h1>Boids Simulation</h1>
      <details>
        <summary>Settings</summary>
        <div v-for="(s, i) in settings" :key="i">
          <Settings :settings="s" :can-remove="settings.length > 1" @remove="removeSpecies(i)" />
        </div>
        <button class="add-species-button" @click="addSpecies">種族を追加</button>
        <button @click="resetSettings" style="margin-bottom:1em;">リセット</button>
        <div>
          <label>
            <input type="checkbox" v-model="showUnits" />
            Unit可視化
          </label>
          <label style="margin-left:1em;">
            <input type="checkbox" v-model="showUnitSpheres" />
            スフィアのみ表示
          </label>
          <label style="margin-left:1em;">
            <input type="checkbox" v-model="showUnitLines" />
            線のみ表示
          </label>
          <label style="margin-left:1em;">
            <input type="checkbox" v-model="showUnitColors" />
            Unit色分け
          </label>
          <label style="margin-left:1em;">
            表示レイヤ下限: <input type="range" min="1" max="20" v-model="unitLayer" />
            {{ unitLayer }}
          </label>
        </div>
      </details>
      <div class="info">
        <p>Boids Count: {{ totalBoids }}</p>
      </div>
    </div>
    <div ref="threeContainer" class="three-container" />
    <audio ref="backgroundAudio" src="/UnderWater.mp3" loop style="display:none" />
  </div>
</template>

<script setup>
import { inject, onMounted, ref, watch, toRaw } from 'vue';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import Settings from './components/Settings.vue';
import StatsGl from 'stats-gl';
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";
import { BoidInstancing } from './rendering/BoidInstancing.js';
import { FogPipeline } from './rendering/FogPipeline.js';
import { ParticleField } from './rendering/ParticleField.js';
import { WasmtimeBridge } from './simulation/WasmtimeBridge.js';
import { createFlockSettingsStore } from './state/FlockSettingsStore.js';

const wasmModule = inject('wasmModule');
if (!wasmModule) {
  console.error('wasmModule not provided');
}

const wasmBridge = wasmModule ? new WasmtimeBridge(wasmModule) : null;

// const getUnitCount = wasmModule.cwrap('getUnitCount', 'number', []);
// const getUnitParentIndicesPtr = wasmModule.cwrap('getUnitParentIndicesPtr', 'number', []);

function isMobileDevice() {
  if (typeof navigator === 'undefined') {
    return false;
  }
  return /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
}

function fetchTreeStructure() {
  return wasmBridge?.exportTreeStructure() ?? null;
}
const mobileBoidCount = isMobileDevice() ? 6000 : 10000;

const DEFAULT_SETTINGS = [{
  species: 'Boids',         // 種族名
  count: mobileBoidCount,   // 群れの数（スマホなら6000、PCなら10000）
  cohesion: 38,             // 凝集
  cohesionRange: 30,        // 凝集範囲
  separation: 8,            // 分離
  separationRange: 0.6,     // 分離範囲
  alignment: 19,            // 整列
  alignmentRange: 6,        // 整列範囲
  maxSpeed: 0.26,           // 最大速度
  maxTurnAngle: 0.25,       // 最大旋回角
  maxNeighbors: 6,          // 最大近傍数
  horizontalTorque: 0.019,  // 水平化トルク
  torqueStrength: 10,       // 回転トルク強度
  lambda: 0.62,             // 速度調整係数
  tau: 1.0                  // 記憶時間スケール
}, {
  species: 'Predator',
  count: 1,
  cohesion: 5.58,                     // 捕食者には使わない
  separation: 0.0,
  alignment: 0.0,
  maxSpeed: 1.37,                     // 速く逃げられるよう速度は大きめ
  minSpeed: 0.4,
  maxTurnAngle: 0.2,
  separationRange: 14.0,
  alignmentRange: 11.0,
  cohesionRange: 77.0,
  maxNeighbors: 0,
  lambda: 0.05,
  tau: 1.0, // 捕食者は常に追いかける
  horizontalTorque: 0.022,
  torqueStrength: 0.0,
  isPredator: true                // ← 捕食者フラグ
}];

const flockStore = createFlockSettingsStore(DEFAULT_SETTINGS);
const {
  settings,
  totalBoids,
  replaceSettings,
  resetToDefaults,
  addSpecies: addSpeciesFromStore,
  removeSpecies: removeSpeciesFromStore,
  saveToStorage,
} = flockStore;

function snapshotSettingsList(list) {
  return list.map((item) => JSON.parse(JSON.stringify(toRaw(item))));
}

let cachedTotalBoidCount = totalBoids.value;
let lastSpeciesSignature = getSpeciesSignature(settings);
let previousSettingsSnapshot = snapshotSettingsList(settings);
let pendingStateForReinitialize = null;
let pendingSettingsSnapshot = null;

function applySettingsSnapshot(snapshot) {
  if (!Array.isArray(snapshot) || snapshot.length === 0) {
    return null;
  }
  const sanitized = replaceSettings(snapshot);
  saveToStorage();
  previousSettingsSnapshot = snapshotSettingsList(settings);
  return sanitized;
}

function addSpecies(template) {
  const added = addSpeciesFromStore(template);
  saveToStorage();
  return added;
}

function removeSpecies(index) {
  if (settings.length <= 1) {
    return null;
  }
  const removed = removeSpeciesFromStore(index);
  if (removed) {
    saveToStorage();
  }
  return removed;
}

const threeContainer = ref(null);
const backgroundAudio = ref(null);
let scene, camera, renderer, controls;
let fogPipeline = null;
let particleField = null;

const paused = ref(false);

const showUnits = ref(true);
const showUnitSpheres = ref(false);
const showUnitLines = ref(false);
const showUnitColors = ref(false);
const unitLayer = ref(1);

let unitSpheres = [];
let unitLines = [];

let maxDepth = 1;
let stats = null;
let glContext = null;
let frameCounter = 0;
let flockReinitTimer = null;

let animationTimer = null;
const FRAME_INTERVAL = 1000 / 1000;//1000 / 60; // 60FPS
const COUNT_REINIT_DELAY_MS = 400;

function positionStatsOverlay(element) {
  if (!element) return;
  element.style.position = 'fixed';
  element.style.padding = '80px';
  element.style.pointerEvents = 'none';
  element.style.display = 'flex';
  element.style.flexDirection = 'column';
  element.style.alignItems = 'flex-end';
  element.style.gap = '6px';
  element.style.maxWidth = '200px';
  element.style.width = 'auto';
  element.style.boxSizing = 'border-box';
  element.style.zIndex = 1000;
}

// ツリーの最大深さを計算
function calcMaxDepth(unit, depth = 0) {
  if (!unit || !unit.children || typeof unit.children.size !== 'function' || unit.children.size() === 0) {
    return depth;
  }
  let max = depth;
  for (let i = 0; i < unit.children.size(); i++) {
    const child = unit.children.get(i);
    max = Math.max(max, calcMaxDepth(child, depth + 1));
  }
  return max;
}

function handleKeydown(e) {
  if (e.code === 'Space') {
    paused.value = !paused.value;
  }
}

function initThreeJS() {
  const width = window.innerWidth;
  const height = window.innerHeight;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(OCEAN_COLORS.DEEP_BLUE);
  createOceanSphere();

  camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
  camera.position.set(3, -5, 3);
  camera.lookAt(0, 0, 0);

  renderer = new THREE.WebGLRenderer({
    antialias: true,
    depth: true, // 深度バッファを有効化

  });
  renderer.setPixelRatio(window.devicePixelRatio); // 高DPI対応
  renderer.setSize(width, height);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap; // 影を柔らかく

  glContext = renderer.getContext();

  threeContainer.value.appendChild(renderer.domElement);

  camera.aspect = width / height;
  camera.updateProjectionMatrix();

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; // なめらかな操作
  controls.dampingFactor = 0.1;

  // 地面メッシュ追加
  const groundGeo = new THREE.PlaneGeometry(300, 300);
  const groundMat = createFadeOutGroundMaterial();
  groundMat.depthTest = true;
  const ground = new THREE.Mesh(groundGeo, groundMat);
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -10;
  ground.receiveShadow = true; // 影を受ける
  scene.add(ground);

  // ライト
  const ambientLight = new THREE.AmbientLight(toHex(OCEAN_COLORS.AMBIENT_LIGHT), 0.9);
  scene.add(ambientLight);

  // 太陽光（やや暖色のDirectionalLight）
  const dirLight = new THREE.DirectionalLight(toHex(OCEAN_COLORS.SUN_LIGHT), 20); // 暖色＆強め
  dirLight.position.set(300, 500, 200); // 高い位置から照らす
  dirLight.castShadow = true;

  // 影カメラの範囲を広げる
  dirLight.shadow.camera.left = -100;
  dirLight.shadow.camera.right = 100;
  dirLight.shadow.camera.top = 100;
  dirLight.shadow.camera.bottom = -100;
  dirLight.shadow.camera.near = 1;
  dirLight.shadow.camera.far = 1000;

  dirLight.shadow.mapSize.width = 1024;
  dirLight.shadow.mapSize.height = 1024;
  dirLight.shadow.bias = -0.01;
  dirLight.shadow.normalBias = 0.01;

  scene.add(dirLight);
  initParticleSystem();
  if (!isMobileDevice()) {
    fogPipeline?.dispose();
    fogPipeline = new FogPipeline(heightFogConfig);
    fogPipeline.init(renderer, scene, camera, width, height);
  } else if (fogPipeline) {
    fogPipeline.dispose();
    fogPipeline = null;
  }
  // ウィンドウリサイズ対応
  window.addEventListener('resize', onWindowResize);
}

function onWindowResize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
  fogPipeline?.resize(width, height);
}

function createSinCosLutTexture(size) {
  const data = new Uint8Array(size * 4);
  for (let i = 0; i < size; i++) {
    const angle = (i / size) * Math.PI * 2;
    const sinValue = Math.sin(angle);
    const cosValue = Math.cos(angle);
    data[i * 4] = Math.round((sinValue * 0.5 + 0.5) * 255);
    data[i * 4 + 1] = Math.round((cosValue * 0.5 + 0.5) * 255);
    data[i * 4 + 2] = 0;
    data[i * 4 + 3] = 255;
  }
  const texture = new THREE.DataTexture(data, size, 1, THREE.RGBAFormat);
  texture.needsUpdate = true;
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.ClampToEdgeWrapping;
  texture.magFilter = THREE.LinearFilter;
  texture.minFilter = THREE.LinearFilter;
  texture.generateMipmaps = false;
  texture.flipY = false;
  return texture;
}

const TRIPLE_BUFFER_SIZE = 3;               // BoidInstancing のトリプルバッファ数
const HIDDEN_POSITION = 1e6;                // 非表示インスタンスを退避させる座標値
const IDENTITY_QUATERNION = [0, 0, 0, 1];   // 非表示インスタンスに適用する無回転クォータニオン
const SIN_LUT_SIZE = 256;                   // 尾びれアニメ用サイン LUT サイズ
const sinCosLutTexture = createSinCosLutTexture(SIN_LUT_SIZE);
// LOD距離閾値（平方距離）: 近距離はハイポリ、中距離はLOD+アニメ、遠距離はLOD静止
const LOD_NEAR_DISTANCE_SQ = 4; // 2m以内はメインモデル
const LOD_MID_DISTANCE_SQ = 25; // 5m以内はLODモデル＋アニメ
const tailAnimation = {
  uniforms: {
    uTailTime: { value: 0 },            // 時間（波形生成用）
    uTailAmplitude: { value: 0.14 },    // 振幅（全身の揺れ幅）
    uTailFrequency: { value: 10.0 },    // 周波数（くねり速度）
    uTailPhaseStride: { value: 5.0 },   // 体の長さ方向の位相差（波長に相当）
    uTailTurnStrength: { value: 0.1 },  // 旋回時の強度
    uTailSpeedScale: { value: 1 },      // 速度による影響度
    uTailRight: { value: new THREE.Vector3(1, 0, 0) },     // 尾アニメの右方向ベクトル
    uTailForward: { value: new THREE.Vector3(0, 0, 1) },   // 尾アニメの進行方向ベクトル
    uTailUp: { value: new THREE.Vector3(0, 1, 0) },        // 尾アニメの上方向ベクトル
    uTailEnable: { value: 1 },          // アニメーション有効/無効
    uSinLut: { value: sinCosLutTexture },
    uLutSize: { value: SIN_LUT_SIZE },
  },
};

const boidInstancing = new BoidInstancing({
  tailAnimation,
  tripleBufferSize: TRIPLE_BUFFER_SIZE,
  hiddenPosition: HIDDEN_POSITION,
  identityQuaternion: IDENTITY_QUATERNION,
  lodNearDistanceSq: LOD_NEAR_DISTANCE_SQ,
  lodMidDistanceSq: LOD_MID_DISTANCE_SQ,
});

let instancedMeshHigh = null;
let instancedMeshLow = null;


// 海中シーンの色味をまとめて管理する定数群
const OCEAN_COLORS = {
  SKY_HIGHLIGHT: '#4fbaff',
  SKY_BLUE: '#15a1ff',
  DEEP_BLUE: '#002968',
  SEAFLOOR: '#777465',
  FOG: '#153a6c',
  AMBIENT_LIGHT: '#2c9aff',
  SUN_LIGHT: '#5389b7',
  SIDE_LIGHT1: '#6ba3d0',
  SIDE_LIGHT2: '#2d5f7a',
  BOTTOM_LIGHT: '#0f2635',
};

// '#rrggbb' 形式の色を three.js の整数表現に変換
const toHex = (colorStr) => parseInt(colorStr.replace('#', '0x'), 16);

// 距離と深度で濃さが変わる海中フォグ設定
const heightFogConfig = {
  color: new THREE.Color(OCEAN_COLORS.DEEP_BLUE),       // 遠景で溶け込む深海色
  distanceStart: 4.0,                      // カメラからこの距離まではフォグゼロ
  distanceEnd: 60.0,                       // この距離でフォグが最大になる
  distanceExponent: 0.4,                   // 距離カーブの滑らかさ
  distanceControlPoint1: new THREE.Vector2(0.2, 0.8),    // 距離ベジェ曲線の制御点（開始側）
  distanceControlPoint2: new THREE.Vector2(0.75, 0.95),  // 距離ベジェ曲線の制御点（終端側）
  surfaceLevel: 100.0,                       // 水面の高さ。ここから下がるほど暗くなる
  heightFalloff: 0.01,                       // 深度による減衰率
  heightExponent: 1,                          // 深度カーブの強さ
  maxOpacity: 1.2,                            // 最大フォグ率
};

function createOceanSphere() {
  if (!scene) return null;

  // 上層→深層のグラデーションで海中の空気感を演出
  const canvas = document.createElement('canvas');
  const context = canvas.getContext('2d');
  canvas.width = 512;
  canvas.height = 512;

  const gradient = context.createLinearGradient(0, 0, 0, canvas.height);
  gradient.addColorStop(0, OCEAN_COLORS.SKY_HIGHLIGHT);
  gradient.addColorStop(0.2, OCEAN_COLORS.SKY_BLUE);
  gradient.addColorStop(0.6, OCEAN_COLORS.DEEP_BLUE);
  gradient.addColorStop(1, OCEAN_COLORS.DEEP_BLUE);

  context.fillStyle = gradient;
  context.fillRect(0, 0, canvas.width, canvas.height);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.generateMipmaps = false;
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;

  const sphereGeo = new THREE.SphereGeometry(300, 32, 32);
  const sphereMat = new THREE.MeshBasicMaterial({
    map: texture,
    side: THREE.BackSide,
    fog: false,
  });

  const oceanSphere = new THREE.Mesh(sphereGeo, sphereMat);
  scene.add(oceanSphere);
  return oceanSphere;
}

function createFadeOutGroundMaterial() {
  const textureLoader = new THREE.TextureLoader();
  const alphaMap = textureLoader.load('./models/groundAlfa.png');

  alphaMap.minFilter = THREE.LinearFilter;
  alphaMap.magFilter = THREE.LinearFilter;
  alphaMap.wrapS = THREE.ClampToEdgeWrapping;
  alphaMap.wrapT = THREE.ClampToEdgeWrapping;

  const material = new THREE.MeshStandardMaterial({
    color: toHex(OCEAN_COLORS.SEAFLOOR),
    transparent: true,
    alphaMap,
    depthWrite: true,
  });

  material.roughness = 0.85;
  material.metalness = 0.0;
  return material;
}

function updateInstancingMaterialUniforms(time) {
  boidInstancing.setTailUniformTime(time);
  particleField?.setTime(time);
}

let shaderTime = 0;

// WebGL2専用の軽量パーティクルを初期化
function initParticleSystem() {
  if (!scene || !renderer) {
    return;
  }
  if (!particleField) {
    particleField = new ParticleField(isMobileDevice());
  }
  particleField.init(scene, renderer, camera, controls);
}

// カメラ操作に応じて粒子ボリュームの中心とスケールを更新
function updateParticleUniforms() {
  particleField?.update(camera, controls);
}

function initBackgroundAudioPlayback() {
  const audioEl = backgroundAudio.value;
  if (!audioEl) {
    return;
  }

  audioEl.volume = 0.1;
  audioEl.loop = true;

  const tryPlay = () => {
    const playResult = audioEl.play();
    if (playResult && typeof playResult.then === 'function') {
      playResult.catch(() => {
        const resume = () => {
          document.removeEventListener('pointerdown', resume);
          document.removeEventListener('keydown', resume);
          audioEl.play().catch(() => {
            /* ignored */
          });
        };
        document.addEventListener('pointerdown', resume, { once: true });
        document.addEventListener('keydown', resume, { once: true });
      });
    }
  };

  tryPlay();
}

function stepSimulationAndUpdateState(deltaTime) {
  if (!wasmBridge) {
    return 0;
  }
  return wasmBridge.stepSimulation(deltaTime);
}

function getWasmViews(count) {
  if (!wasmBridge) {
    return {
      positions: new Float32Array(0),
      orientations: new Float32Array(0),
      velocities: new Float32Array(0),
      speciesIds: new Int32Array(0),
    };
  }
  return wasmBridge.getBuffers(count);
}

function captureFlockState() {
  return wasmBridge?.captureState() ?? null;
}

function restoreFlockState(previousState, oldSettings, newSettings) {
  wasmBridge?.restoreState(previousState, oldSettings, newSettings);
}

let boidModel = null; // 読み込んだモデルを保持
let boidModelLod = null; // 読み込んだモデルを保持
let originalMaterial = null; // 元のマテリアルを保持
let originalMaterialLod = null; // 元のLODマテリアルを保持
let predatorModel = null;
let predatorMaterial = null;

// 前回のshowUnitColorsの状態を保持（OFF→ONの検知用）
let lastShowUnitColors = false;

function getPredatorCount() {
  return settings.reduce((sum, s) => sum + ((s.isPredator && s.count) ? s.count : 0), 0);
}

function getTotalBoidCount() {
  return totalBoids.value;
}

function getSpeciesSignature(specList = settings) {
  if (!Array.isArray(specList)) {
    return '';
  }
  return specList
    .map((s, index) => `${index}:${(s && s.count) || 0}:${s && s.isPredator ? 1 : 0}`)
    .join('|');
}

function createSpeciesParamsVector() {
  if (!wasmBridge) {
    console.warn('WasmtimeBridge が初期化されていません');
    return null;
  }

  const rawSettings = toRaw(settings);
  const source = Array.isArray(rawSettings) ? rawSettings : Array.from(settings ?? []);
  const vector = wasmBridge.buildSpeciesParams(source);
  if (!vector) {
    console.warn('WasmtimeBridge.buildSpeciesParams の呼び出しに失敗しました');
  }
  return vector;
}

function reinitializeFlockNow() {
  if (!wasmModule || !wasmBridge) return;

  const pendingState = pendingStateForReinitialize?.state || null;
  const oldSettingsRef = pendingStateForReinitialize?.oldSettings || previousSettingsSnapshot;
  const newSettingsRef = pendingSettingsSnapshot || snapshotSettingsList(settings);
  const targetSignature = getSpeciesSignature(newSettingsRef);

  const vector = createSpeciesParamsVector();
  if (!vector) {
    console.error('Failed to create species parameter vector');
    applySettingsSnapshot(previousSettingsSnapshot);
    pendingStateForReinitialize = null;
    pendingSettingsSnapshot = null;
    return;
  }

  try {
    wasmModule.callInitBoids(vector, 1, 6, 0.25);
  } finally {
    if (typeof vector.delete === 'function') {
      vector.delete();
    }
  }
  try {
    wasmBridge.buildSpatialIndex(16, 0);
  } catch (error) {
    console.error('WasmtimeBridge.buildSpatialIndex の呼び出しに失敗しました', error);
  }
  const total = getTotalBoidCount();
  cachedTotalBoidCount = total;
  initInstancedBoids(total);

  if (pendingState?.count > 0) {
    restoreFlockState(pendingState, oldSettingsRef, newSettingsRef);
  }

  lastSpeciesSignature = targetSignature;
  previousSettingsSnapshot = newSettingsRef;
  pendingStateForReinitialize = null;
  pendingSettingsSnapshot = null;
}

function scheduleFlockReinitialize() {
  if (flockReinitTimer) {
    clearTimeout(flockReinitTimer);
  }

  if (!pendingStateForReinitialize) {
    pendingStateForReinitialize = {
      state: captureFlockState(),
      oldSettings: previousSettingsSnapshot,
    };
  }

  pendingSettingsSnapshot = snapshotSettingsList(settings);

  flockReinitTimer = setTimeout(() => {
    flockReinitTimer = null;
    reinitializeFlockNow();
  }, COUNT_REINIT_DELAY_MS);
}

function initInstancedBoids(count) {
  if (!scene || !boidModel || !boidModelLod || !originalMaterial || !originalMaterialLod) {
    console.warn('initInstancedBoids: required assets are not ready');
    return;
  }

  const initialized = boidInstancing.init(scene, {
    count,
    boidModel,
    boidModelLod,
    highMaterial: originalMaterial,
    lowMaterial: originalMaterialLod,
    predatorModel,
  });

  if (!initialized) {
    console.error('Failed to initialize boid instancing');
    return;
  }

  const meshes = boidInstancing.getMeshes();
  instancedMeshHigh = meshes.high;
  instancedMeshLow = meshes.low;

  // 捕食者メッシュも最新設定に合わせて生成しておく
  const predatorCount = getPredatorCount();
  if (typeof boidInstancing.ensurePredatorMeshes === 'function') {
    boidInstancing.ensurePredatorMeshes(predatorCount);
  }

  if (instancedMeshHigh?.instanceColor) {
    instancedMeshHigh.instanceColor.needsUpdate = true;
  }
  if (instancedMeshLow?.instanceColor) {
    instancedMeshLow.instanceColor.needsUpdate = true;
  }
}



function loadBoidModel(callback) {
  const loader = new GLTFLoader();
  const basePath = process.env.BASE_URL || '/'; // publicPath を取得
  const textureLoader = new THREE.TextureLoader();
  let pendingAssets = 3;
  const notifyReady = () => {
    pendingAssets = Math.max(0, pendingAssets - 1);
    if (pendingAssets === 0) {
      callback();
    }
  };
  const texture = textureLoader.load(
    './models/fish.png', // テクスチャのパス
    () => {
      console.log('Texture loaded successfully.');
    },
    undefined,
    (error) => {
      console.error('An error occurred while loading the texture:', error);
    }
  );
  const textureLod = textureLoader.load(
    './models/fish_lod.png', // テクスチャのパス
    () => {
      console.log('Texture loaded successfully.');
    },
    undefined,
    (error) => {
      console.error('An error occurred while loading the texture:', error);
    }
  );
  texture.flipY = false;
  texture.colorSpace = THREE.SRGBColorSpace; // sRGBカラー空間を使用
  textureLod.flipY = false;
  textureLod.colorSpace = THREE.SRGBColorSpace;

  const predatorTexture = textureLoader.load(
    './models/fishPredetor.png',
    () => {
      console.log('Predator texture loaded successfully.');
    },
    undefined,
    (error) => {
      console.error('An error occurred while loading the predator texture:', error);
    }
  );
  predatorTexture.flipY = false;
  predatorTexture.colorSpace = THREE.SRGBColorSpace;

  let boidMaterial = new THREE.MeshStandardMaterial({
    color: 0xffffff, // 白色
    roughness: 0.5,
    metalness: 0.2,
    transparent: false, // 半透明を有効化
    alphaTest: 0.5,    // アルファテストを設定
    map: texture,      // テクスチャを設定
    vertexColors: false, // 通常時は無効
    vertexColor: 0xffffff
  });

  let boidLodMaterial = new THREE.MeshStandardMaterial({
    color: 0xffffff, // 白色
    roughness: 0.5,
    metalness: 0.2,
    transparent: false, // 半透明を有効化
    alphaTest: 0.5,    // アルファテストを設定
    map: textureLod,      // テクスチャを設定
    vertexColors: false, // 通常時は無効
    vertexColor: 0xffffff

  });

  originalMaterial = boidMaterial;
  originalMaterialLod = boidLodMaterial;
  predatorMaterial = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    roughness: 0.5,
    metalness: 0,
    transparent: false,
    alphaTest: 0.5,
    map: predatorTexture,
    vertexColors: false,
    vertexColor: 0xffffff,
  });
  loader.load(
    `./models/boidModel.glb`, // モデルのパス
    (gltf) => {
      boidModel = gltf.scene;

      // マテリアルの transparent と alphaTest を変更
      boidModel.traverse((child) => {
        if (child.isMesh) {
          child.material = boidMaterial; // 半透明を有効化
        }
      });

      notifyReady();
    },
    undefined,
    (error) => {
      console.error('An error occurred while loading the model:', error);
      notifyReady();
    }
  );

  loader.load(
    `./models/boidModel_lod.glb`, // LODモデルのパス
    (gltf) => {
      boidModelLod = gltf.scene;

      // マテリアルの transparent と alphaTest を変更
      boidModelLod.traverse((child) => {
        if (child.isMesh) {
          child.material = boidLodMaterial; // 半透明を有効化
        }
      });

      notifyReady();
    },
    undefined,
    (error) => {
      console.error('An error occurred while loading the LOD model:', error);
      notifyReady();
    }
  );

  loader.load(
    `./models/boidPredetorModel.glb`,
    (gltf) => {
      predatorModel = gltf.scene;
      predatorModel.traverse((child) => {
        if (child.isMesh) {
          child.material = predatorMaterial;
          child.castShadow = true;
          child.receiveShadow = true;
        }
      });
      notifyReady();
    },
    undefined,
    (error) => {
      console.error('An error occurred while loading the predator model:', error);
      notifyReady();
    }
  );
}

function clearUnitVisuals() {
  for (const mesh of unitSpheres) scene.remove(mesh);
  for (const line of unitLines) scene.remove(line);
  unitSpheres = [];
  unitLines = [];
}

// レイヤ制限付き再帰描画
function drawUnitTree(unit, layer = 0) {
  // スフィア: スライダで制御
  if (
    layer >= (maxDepth - unitLayer.value + 1) &&
    (unit.children == null || unit.children.size() === 0 || layer === maxDepth)
  ) {
    let sphere;
    if (unitSpheres.length > 0) {
      sphere = unitSpheres.pop(); // 再利用
      sphere.visible = true;
    } else {
      const geometry = new THREE.SphereGeometry(unit.radius, 16, 16);
      const material = new THREE.MeshBasicMaterial({
        color: new THREE.Color().setHSL(0.1, 1, 0.7 - 0.4 * (layer / maxDepth)),
        wireframe: true,
        opacity: 0.3,
        transparent: true,
      });
      sphere = new THREE.Mesh(geometry, material);
      scene.add(sphere);
    }
    sphere.position.set(unit.center.x, unit.center.y, unit.center.z);
    unitSpheres.push(sphere);
  }

  // 線: チェックボックスで全て表示、色は深さでグラデ
  if (showUnitLines.value && unit.children && typeof unit.children.size === 'function' && unit.children.size() > 0) {
    for (let i = 0; i < unit.children.size(); i++) {
      const child = unit.children.get(i);
      let line;
      if (unitLines.length > 0) {
        line = unitLines.pop(); // 再利用
        line.visible = true;
      } else {
        const lineGeometry = new THREE.BufferGeometry();
        const lineMaterial = new THREE.LineBasicMaterial({
          color: new THREE.Color().setHSL(0.35, 1, 0.7 - 0.4 * (layer / maxDepth)),
        });
        line = new THREE.Line(lineGeometry, lineMaterial);
        scene.add(line);
      }
      const points = [
        new THREE.Vector3(unit.center.x, unit.center.y, unit.center.z),
        new THREE.Vector3(child.center.x, child.center.y, child.center.z),
      ];
      line.geometry.setFromPoints(points);
      unitLines.push(line);
    }
  }

  // 再帰
  if (unit.children && typeof unit.children.size === 'function' && unit.children.size() > 0) {
    for (let i = 0; i < unit.children.size(); i++) {
      const child = unit.children.get(i);
      drawUnitTree(child, layer + 1);
    }
  }
}
let lastTime = performance.now(); // 前回のフレームのタイムスタンプ

function animate() {
  stats?.begin();

  const currentTime = performance.now();
  const deltaTime = (currentTime - lastTime) / 1000;
  lastTime = currentTime;

  if (!paused.value) {
    shaderTime += deltaTime;
  }
  updateInstancingMaterialUniforms(shaderTime);

  const count = stepSimulationAndUpdateState(paused.value ? 0 : deltaTime);

  const meshes = boidInstancing.getMeshes();
  instancedMeshHigh = meshes.high;
  instancedMeshLow = meshes.low;
  const pipelineReady = fogPipeline?.isReady?.() ?? false;

  if (!instancedMeshHigh || !instancedMeshLow) {
    controls.update();
    updateParticleUniforms();
    if (pipelineReady) {
      fogPipeline.updateCameraUniforms(camera);
      fogPipeline.render(deltaTime);
    } else {
      renderer.render(scene, camera);
    }
    stats?.end();
    stats?.update();
    animationTimer = setTimeout(animate, FRAME_INTERVAL);
    return;
  }

  const { positions, orientations, velocities } = getWasmViews(count);
  if ((frameCounter++ & 63) === 0) {
    wasmBridge?.currentFirstBoidX();
  }

  const predatorCount = getPredatorCount();
  const updateInfo = boidInstancing.update({
    count,
    positions,
    orientations,
    velocities,
    cameraPosition: camera.position,
    predatorCount,
  });

  const visibleCount = updateInfo.visibleCount ?? Math.max(0, count - predatorCount);

  if (showUnitColors.value && instancedMeshHigh.instanceColor && instancedMeshLow.instanceColor) {
    const unitMappings = wasmBridge?.getUnitMappings(count);
    if (unitMappings) {
      for (let i = 0; i < visibleCount; i++) {
        let unitId = -1;
        for (let j = 0; j < unitMappings.length; j += 2) {
          if (unitMappings[j] === i) {
            unitId = unitMappings[j + 1];
            break;
          }
        }

        const color = unitId >= 0
          ? new THREE.Color().setHSL((unitId % 100) / 100, 0.8, 0.6)
          : new THREE.Color(1, 0, 0);

        instancedMeshHigh.setColorAt(i, color);
        instancedMeshLow.setColorAt(i, color);
      }
      instancedMeshHigh.instanceColor.needsUpdate = true;
      instancedMeshLow.instanceColor.needsUpdate = true;
    }
  } else if (lastShowUnitColors && instancedMeshHigh.instanceColor && instancedMeshLow.instanceColor) {
    const whiteColor = new THREE.Color(1, 1, 1);
    for (let i = 0; i < visibleCount; i++) {
      instancedMeshHigh.setColorAt(i, whiteColor);
      instancedMeshLow.setColorAt(i, whiteColor);
    }
    instancedMeshHigh.instanceColor.needsUpdate = true;
    instancedMeshLow.instanceColor.needsUpdate = true;
    console.log('✓ Reset vertex colors to white (OFF mode)');
  }

  lastShowUnitColors = showUnitColors.value;

  controls.update();
  updateParticleUniforms();
  if (pipelineReady) {
    fogPipeline.updateCameraUniforms(camera);
    fogPipeline.render(deltaTime);
  } else {
    renderer.render(scene, camera);
  }

  stats?.end();
  stats?.update();

  animationTimer = setTimeout(animate, FRAME_INTERVAL);
}

function drawTreeStructure(treeData) {
  const drawNode = (node, parentPosition = null) => {
    const position = new THREE.Vector3(
      node.center[0],
      node.center[1],
      node.center[2]
    );
    if (parentPosition) {
      controls.update();
      updateParticleUniforms();
      const geometry = new THREE.BufferGeometry().setFromPoints([
        parentPosition,
        position,
      ]);
      const material = new THREE.LineBasicMaterial({ color: 0xffffff });
      const line = new THREE.Line(geometry, material);
      scene.add(line);
    }

    if (node.children) {
      node.children.forEach((child) => drawNode(child, position));
    }
  };

  treeData.forEach((rootNode) => drawNode(rootNode));
}
function startSimulation() {
  reinitializeFlockNow();
  animate();
}

onMounted(() => {
  initThreeJS();
  loadBoidModel(() => {
    console.log('Boid model loaded successfully.');

    stats = new StatsGl({
      trackGPU: true,
      trackHz: true,
      trackCPT: true,
      logsPerSecond: 4,
      graphsPerSecond: 30,
      samplesLog: 40,
      samplesGraph: 10,
      precision: 2,
      horizontal: true,
      minimal: false,
      mode: 0,
    });

    const statsInitTarget = renderer?.domElement ?? document.body;

    const ensureStatsOverlay = () => {
      const statsElement =
        (typeof stats?.domElement !== 'undefined' ? stats.domElement : null) ||
        (typeof stats?.getDom === 'function' ? stats.getDom() : null) ||
        stats?.dom ||
        stats?.container ||
        stats?.wrapper ||
        null;

      if (statsElement) {
        if (!statsElement.parentElement) {
          document.body.appendChild(statsElement);
        }
        positionStatsOverlay(statsElement);
      }
    };

    const initPromise =
      stats && typeof stats.init === 'function'
        ? Promise.resolve(stats.init(statsInitTarget))
        : Promise.resolve();

    initPromise
      .then(() => {
        if (typeof stats?.patchThreeRenderer === 'function') {
          stats.patchThreeRenderer(renderer);
        }
        ensureStatsOverlay();
      })
      .catch((error) => {
        console.error('Failed to initialize stats-gl:', error);
        ensureStatsOverlay();
      });

    startSimulation();
    initBackgroundAudioPlayback();
  });

  window.addEventListener('keydown', handleKeydown);
});

watch(
  settings,
  () => {
    if (wasmModule && wasmModule.setGlobalSpeciesParamsFromJS) {
      const vector = createSpeciesParamsVector();
      if (vector) {
        try {
          wasmModule.setGlobalSpeciesParamsFromJS(vector, 1);
        } finally {
          if (typeof vector.delete === 'function') {
            vector.delete();
          }
        }
      }
    }

    saveToStorage();

    const signature = getSpeciesSignature(settings);
    if (signature !== lastSpeciesSignature) {
      scheduleFlockReinitialize();
    } else {
      if (flockReinitTimer) {
        clearTimeout(flockReinitTimer);
        flockReinitTimer = null;
      }
      pendingStateForReinitialize = null;
      pendingSettingsSnapshot = null;
      previousSettingsSnapshot = snapshotSettingsList(settings);
    }

    const predators = getPredatorCount();
    if (typeof boidInstancing.ensurePredatorMeshes === 'function') {
      boidInstancing.ensurePredatorMeshes(predators);
    }
    const predatorMeshes =
      typeof boidInstancing.getPredatorMeshes === 'function'
        ? boidInstancing.getPredatorMeshes()
        : [];
    for (const mesh of predatorMeshes) {
      mesh.visible = false;
    }
  },
  { deep: true }
);

function resetSettings(presetList) {
  if (Array.isArray(presetList) && presetList.length > 0) {
    applySettingsSnapshot(presetList);
  } else {
    resetToDefaults();
  }
}

// Unit可視化の変更を監視
watch(showUnits, (newValue) => {
  console.log('showUnits changed to:', newValue);
  if (!newValue) {
    // Unit可視化をオフにした場合、既存の可視化要素をクリア
    clearUnitVisuals();
  }
});

// Unit表示モードの変更を監視
watch([showUnitSpheres, showUnitLines], ([newSpheres, newLines]) => {
  console.log('Unit display mode changed - Spheres:', newSpheres, 'Lines:', newLines);
  // 表示モードが変更されたら既存の表示をクリアして再描画
  if (showUnits.value) {
    clearUnitVisuals();
  }
});
</script>

<style>
#app {
  font-family: Arial, sans-serif;
  padding: 0;
  margin: 0;
  position: relative;
  z-index: 1;
}

.settings {
  margin-bottom: 20px;
}

.info {
  margin-top: 20px;
}

.three-container {
  position: fixed;
  left: 0;
  top: 0;
  width: 100vw;
  height: 100vh;
  z-index: 0;
  display: block;
  border: none;
  overflow: hidden;
  background: #0a1e3a;
}

.ui-overlay {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  padding: 20px;
  box-sizing: border-box;
  color: #fff;
  z-index: 2;
  pointer-events: none;
}

.ui-overlay * {
  pointer-events: auto;
}

.add-species-button {
  margin-top: 10px;
  margin-bottom: 10px;
}
</style>