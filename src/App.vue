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
import { computed, inject, onMounted, reactive, ref, watch, toRaw } from 'vue';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import Settings from './components/Settings.vue';
import StatsGl from 'stats-gl';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { SSAOPass } from 'three/examples/jsm/postprocessing/SSAOPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { ShaderPass } from 'three/examples/jsm/postprocessing/ShaderPass.js';
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";

const wasmModule = inject('wasmModule');
if (!wasmModule) {
  console.error('wasmModule not provided');
}

const stepSimulation = wasmModule.cwrap('stepSimulation', 'number', ['number'])
const build = wasmModule.cwrap('build', 'void', ['number', 'number'])
const exportTreeStructure = wasmModule.cwrap('exportTreeStructure', 'object', [])
const boidUnitMappingPtr = wasmModule.cwrap('boidUnitMappingPtr', 'number', []);
const currentFirstBoidX = wasmModule.cwrap('currentFirstBoidX', 'number', []);
const speciesIdsPtr = wasmModule.cwrap('speciesIdsPtr', 'number', []);
const syncReadToWriteBuffers = wasmModule.cwrap('syncReadToWriteBuffers', 'void', []);
// const getUnitCount = wasmModule.cwrap('getUnitCount', 'number', []);
// const getUnitParentIndicesPtr = wasmModule.cwrap('getUnitParentIndicesPtr', 'number', []);

function isMobileDevice() {
  if (typeof navigator === 'undefined') {
    return false;
  }
  return /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
}

function fetchTreeStructure() {
  const treeData = exportTreeStructure();
  return treeData;
}
const mobileBoidCount = isMobileDevice() ? 6000 : 10000;

const DEFAULT_SETTINGS = [{
  species: 'Boids',         // 種族名
  count: mobileBoidCount,   // 群れの数（スマホなら6000、PCなら10000）
  cohesion: 32,             // 凝集
  cohesionRange: 30,        // 凝集範囲
  separation: 8,            // 分離
  separationRange: 0.6,     // 分離範囲
  alignment: 17,            // 整列
  alignmentRange: 6,        // 整列範囲
  maxSpeed: 0.26,           // 最大速度
  maxTurnAngle: 0.25,       // 最大旋回角
  maxNeighbors: 6,          // 最大近傍数
  horizontalTorque: 0.019,  // 水平化トルク
  torqueStrength: 10,       // 回転トルク強度
  lambda: 0.62,             // 速度調整係数
  tau: 1.5                  // 記憶時間スケール
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

function loadSettings() {
  try {
    const saved = localStorage.getItem('boids_settings');
    if (saved) {
      const parsed = JSON.parse(saved);
      if (Array.isArray(parsed)) {
        return parsed; // 配列として保存されている場合のみ返す
      }
    }
  } catch (error) {
    console.error('Failed to load settings from localStorage:', error);
  }
  return DEFAULT_SETTINGS; // デフォルト値を返す
}

function snapshotSettingsList(list) {
  return list.map((item) => JSON.parse(JSON.stringify(toRaw(item))));
}

let cachedTotalBoidCount = 0;
let lastSpeciesSignature = '';
const totalBoids = computed(() => getTotalBoidCount());

const settings = reactive(loadSettings());
cachedTotalBoidCount = getTotalBoidCount();
lastSpeciesSignature = getSpeciesSignature(settings);
let previousSettingsSnapshot = snapshotSettingsList(settings);
let pendingStateForReinitialize = null;
let pendingSettingsSnapshot = null;

function generateUniqueSpeciesName(baseName) {
  const taken = new Set(settings.map((s) => s.species));
  if (!baseName) {
    baseName = 'Species';
  }
  if (!taken.has(baseName)) {
    return baseName;
  }
  let index = 2;
  let candidate = `${baseName} ${index}`;
  while (taken.has(candidate)) {
    index += 1;
    candidate = `${baseName} ${index}`;
  }
  return candidate;
}

function cloneSpeciesTemplate(template) {
  return JSON.parse(JSON.stringify(template));
}

function addSpecies() {
  const template = DEFAULT_SETTINGS[0] ? cloneSpeciesTemplate(DEFAULT_SETTINGS[0]) : {
    species: 'Species',
    count: 0,
    cohesion: 20,
    cohesionRange: 30,
    separation: 5,
    separationRange: 1,
    alignment: 10,
    alignmentRange: 6,
    maxSpeed: 0.3,
    maxTurnAngle: 0.2,
    maxNeighbors: 6,
    horizontalTorque: 0.02,
    torqueStrength: 5,
    lambda: 0.5,
    tau: 1.5,
  };
  const next = cloneSpeciesTemplate(template);
  next.isPredator = false;
  next.species = generateUniqueSpeciesName(next.species);
  next.count = 0;
  settings.push(next);
}

function removeSpecies(index) {
  if (index < 0 || index >= settings.length) {
    return;
  }
  settings.splice(index, 1);
}

const threeContainer = ref(null);
const backgroundAudio = ref(null);
let scene, camera, renderer, controls, composer;
let heightFogPass = null;
let heightFogRenderTarget = null;
let particlePoints = null;
let particleMaterial = null;

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
let latestStatePtr = 0;
let latestStateHeaderPtr = 0;
let latestStateHeaderView = null;
let latestPositionsPtr = 0;
let latestVelocitiesPtr = 0;
let latestOrientationsPtr = 0;
let latestBoidCountFromWasm = 0;
let frameCounter = 0;
let debugTimer = 0;
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
  // EffectComposer の初期化（スマホ以外の場合のみ）
  if (!isMobileDevice()) {
    const renderTargetOptions = {
      depthBuffer: true,
      stencilBuffer: false,
    };
    heightFogRenderTarget = new THREE.WebGLRenderTarget(width, height, renderTargetOptions);
    heightFogRenderTarget.depthTexture = new THREE.DepthTexture(width, height, THREE.FloatType);
    heightFogRenderTarget.depthTexture.format = THREE.DepthFormat;
    heightFogRenderTarget.depthTexture.type = THREE.FloatType;
    heightFogRenderTarget.depthTexture.needsUpdate = true;

    // EffectComposer の初期化
    composer = new EffectComposer(renderer, heightFogRenderTarget);

    // RenderPass を追加
    const renderPass = new RenderPass(scene, camera);
    composer.addPass(renderPass);

    const ssaoPass = new SSAOPass(scene, camera, width, height);
    ssaoPass.kernelRadius = 5; // サンプル半径（大きくすると効果が広がる）
    ssaoPass.minDistance = 0.01; // 最小距離（小さくすると近距離の効果が強調される）
    ssaoPass.maxDistance = 0.3; // 最大距離（大きくすると遠距離の効果が強調される）
    composer.addPass(ssaoPass);

    // UnrealBloomPass を追加（任意）
    const bloomPass = new UnrealBloomPass(new THREE.Vector2(width, height), 1.5, 0.4, 0.85);
    composer.addPass(bloomPass);

    heightFogPass = new ShaderPass(HeightFogShader);
    const heightFogOriginalRender = heightFogPass.render.bind(heightFogPass);
    heightFogPass.render = (renderer, writeBuffer, readBuffer, deltaTime, maskActive) => {
      if (heightFogPass.uniforms.tDepth) {
        heightFogPass.uniforms.tDepth.value = readBuffer?.depthTexture ?? heightFogPass.uniforms.tDepth.value;
      }
      heightFogOriginalRender(renderer, writeBuffer, readBuffer, deltaTime, maskActive);
    };
    heightFogPass.needsSwap = false;
    heightFogPass.uniforms.tDepth.value = heightFogRenderTarget.depthTexture;
    heightFogPass.uniforms.fogColor.value.copy(heightFogConfig.color);
    heightFogPass.uniforms.distanceStart.value = heightFogConfig.distanceStart;
    heightFogPass.uniforms.distanceEnd.value = heightFogConfig.distanceEnd;
    heightFogPass.uniforms.distanceExponent.value = heightFogConfig.distanceExponent;
    heightFogPass.uniforms.distanceControlPoint1.value.copy(heightFogConfig.distanceControlPoint1);
    heightFogPass.uniforms.distanceControlPoint2.value.copy(heightFogConfig.distanceControlPoint2);
    heightFogPass.uniforms.surfaceLevel.value = heightFogConfig.surfaceLevel;
    heightFogPass.uniforms.heightFalloff.value = heightFogConfig.heightFalloff;
    heightFogPass.uniforms.heightExponent.value = heightFogConfig.heightExponent;
    heightFogPass.uniforms.maxOpacity.value = heightFogConfig.maxOpacity;

    heightFogPass.material.depthTest = false;
    heightFogPass.material.depthWrite = false;
    heightFogPass.material.transparent = false;
    heightFogPass.material.blending = THREE.NoBlending;
    heightFogPass.material.toneMapped = true;
    heightFogPass.material.needsUpdate = true;
    heightFogPass.renderToScreen = true;
    composer.addPass(heightFogPass);


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
  if (composer) {
    composer.setSize(width, height);
  }
  if (heightFogRenderTarget) {
    heightFogRenderTarget.setSize(width, height);
  }
}

// 一時的に従来方式に戻す - instancedMeshを単一で使用
let instancedMeshHigh = null; // 高ポリゴン用
let instancedMeshLow = null;  // 低ポリゴン用
const instancingMaterials = new Set();

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

const TRIPLE_BUFFER_SIZE = 3;
const HIDDEN_POSITION = 1e6;
const IDENTITY_QUATERNION = [0, 0, 0, 1];
const SIN_LUT_SIZE = 256;
const sinCosLutTexture = createSinCosLutTexture(SIN_LUT_SIZE);
// LOD距離閾値（平方距離）: 近距離はハイポリ、中距離はLOD+アニメ、遠距離はLOD静止
const LOD_NEAR_DISTANCE_SQ = 4; // 2m以内はメインモデル
const LOD_MID_DISTANCE_SQ = 25; // 5m以内はLODモデル＋アニメ
const STREAM_USAGE = THREE.StreamDrawUsage ?? THREE.DynamicDrawUsage;
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

const PARTICLE_BASIS_TEMP_DIR = new THREE.Vector3();
const PARTICLE_BASIS_TEMP_AXIS = new THREE.Vector3();
const PARTICLE_BASIS_LAT1 = new THREE.Vector3();
const PARTICLE_BASIS_LAT2 = new THREE.Vector3();
const PARTICLE_BASE_SPREAD_DESKTOP = new THREE.Vector3(24, 12, 26);
const PARTICLE_BASE_SPREAD_MOBILE = new THREE.Vector3(16, 9, 18);
const PARTICLE_BASE_MAX_DISTANCE_DESKTOP = PARTICLE_BASE_SPREAD_DESKTOP.z * 1.2;
const PARTICLE_BASE_MAX_DISTANCE_MOBILE = PARTICLE_BASE_SPREAD_MOBILE.z * 1.2;

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
  distanceControlPoint1: new THREE.Vector2(0.2, 0.8),
  distanceControlPoint2: new THREE.Vector2(0.75, 0.95),
  surfaceLevel: 100.0,                       // 水面の高さ。ここから下がるほど暗くなる
  heightFalloff: 0.01,                       // 深度による減衰率
  heightExponent: 1,                          // 深度カーブの強さ
  maxOpacity: 1.2,                            // 最大フォグ率
};

// カメラ位置からの距離と高さでブレンドするフォグシェーダー
const HeightFogShader = {
  uniforms: {
    tDiffuse: { value: null },
    tDepth: { value: null },
    cameraNear: { value: 0.1 },
    cameraFar: { value: 1000 },
    projectionMatrixInverse: { value: new THREE.Matrix4() },
    cameraMatrixWorld: { value: new THREE.Matrix4() },
    fogColor: { value: heightFogConfig.color.clone() },
    distanceStart: { value: heightFogConfig.distanceStart },
    distanceEnd: { value: heightFogConfig.distanceEnd },
    distanceExponent: { value: heightFogConfig.distanceExponent },
    distanceControlPoint1: { value: heightFogConfig.distanceControlPoint1.clone() },
    distanceControlPoint2: { value: heightFogConfig.distanceControlPoint2.clone() },
    surfaceLevel: { value: heightFogConfig.surfaceLevel },
    heightFalloff: { value: heightFogConfig.heightFalloff },
    heightExponent: { value: heightFogConfig.heightExponent },
    maxOpacity: { value: heightFogConfig.maxOpacity },
  },
  vertexShader: /* glsl */`
    varying vec2 vUv;
    void main() {
      vUv = uv;
      gl_Position = vec4(position.xy, 0.0, 1.0);
    }
  `,
  fragmentShader: /* glsl */`
    precision highp float;
    varying vec2 vUv;

    uniform sampler2D tDiffuse;
    uniform sampler2D tDepth;
    uniform float cameraNear;
    uniform float cameraFar;
    uniform vec3 fogColor;
    uniform float distanceStart;
    uniform float distanceEnd;
    uniform float distanceExponent;
    uniform vec2 distanceControlPoint1;
    uniform vec2 distanceControlPoint2;
    uniform float surfaceLevel;
    uniform float heightFalloff;
    uniform float heightExponent;
    uniform float maxOpacity;
    uniform mat4 projectionMatrixInverse;
    uniform mat4 cameraMatrixWorld;

    float cubicBezier1D(float t, float p0, float p1, float p2, float p3) {
      float u = 1.0 - t;
      float uu = u * u;
      float tt = t * t;
      return u * uu * p0 + 3.0 * uu * t * p1 + 3.0 * u * tt * p2 + tt * t * p3;
    }

    float cubicBezierDerivative1D(float t, float p0, float p1, float p2, float p3) {
      float u = 1.0 - t;
      return 3.0 * u * u * (p1 - p0)
           + 6.0 * u * t * (p2 - p1)
           + 3.0 * t * t * (p3 - p2);
    }

    float cubicBezierInverse(float x, vec2 c1, vec2 c2) {
      float t = clamp(x, 0.0, 1.0);
      for (int i = 0; i < 5; i++) {
        float current = cubicBezier1D(t, 0.0, c1.x, c2.x, 1.0) - x;
        float slope = cubicBezierDerivative1D(t, 0.0, c1.x, c2.x, 1.0);
        if (abs(slope) < 1e-5) {
          break;
        }
        t -= current / slope;
        t = clamp(t, 0.0, 1.0);
      }
      return t;
    }

    float sampleDistanceCurve(float x, vec2 c1, vec2 c2) {
      float t = cubicBezierInverse(x, c1, c2);
      return cubicBezier1D(t, 0.0, c1.y, c2.y, 1.0);
    }

    void main() {
      vec4 baseColor = texture2D(tDiffuse, vUv);
      float depth = texture2D(tDepth, vUv).x;

      if (depth >= 1.0) {
        gl_FragColor = baseColor;
        return;
      }

      vec2 ndc = vUv * 2.0 - 1.0;
      float ndcZ = depth * 2.0 - 1.0;
      vec4 clipPos = vec4(ndc, ndcZ, 1.0);
      vec4 viewPos = projectionMatrixInverse * clipPos;
      viewPos /= max(viewPos.w, 1e-5);
      vec4 worldPos = cameraMatrixWorld * viewPos;

      float viewDistance = length(viewPos.xyz);
      float distanceFogNorm = clamp(
        (viewDistance - distanceStart) / max(distanceEnd - distanceStart, 1e-5),
        0.0,
        1.0
      );
      float distanceFog = sampleDistanceCurve(distanceFogNorm, distanceControlPoint1, distanceControlPoint2);
      distanceFog = pow(distanceFog, distanceExponent);

      float depthBelowSurface = max(surfaceLevel - worldPos.y, 0.0);
      float heightFactor = 1.0 - exp(-depthBelowSurface * heightFalloff);
      heightFactor = clamp(pow(heightFactor, heightExponent), 0.0, 1.0);

      float fogFactor = clamp(distanceFog * heightFactor, 0.0, 1.0);
      fogFactor = mix(0.0, maxOpacity, fogFactor);

      vec3 fogged = mix(baseColor.rgb, fogColor, fogFactor);
      gl_FragColor = vec4(fogged, baseColor.a);
      #include <tonemapping_fragment>
      #include <colorspace_fragment>
    }
  `,
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
  tailAnimation.uniforms.uTailTime.value = time;
  if (particleMaterial?.uniforms?.uTime) {
    particleMaterial.uniforms.uTime.value = time;
  }
}

let bufferCursor = 0;
let bufferSetHigh = null;
let bufferSetLow = null;
let cachedHeapBuffer = null;
let cachedPositionsPtr = 0;
let cachedOrientationsPtr = 0;
let cachedPositionsView = null;
let cachedOrientationsView = null;
let cachedPositionsCount = 0;
let cachedOrientationsCount = 0;
let shaderTime = 0;
let cachedVelocitiesPtr = 0;
let cachedVelocitiesView = null;
let cachedVelocitiesCount = 0;
let tailPhaseSeeds = null;
let previousVelocities = null;
let cachedSpeciesIdsView = null;
let cachedSpeciesIdsPtr = 0;
let cachedSpeciesIdsCount = 0;
let cachedSpeciesIdsBuffer = null;
let previousLodFlags = null;

function getTailParamsArrayType() {
  // TODO: Float16 を使う場合は Uint16Array へのエンコード処理が必要
  return Float32Array;
}

// WebGL2専用の軽量パーティクルを初期化
function initParticleSystem() {
  if (!scene || !renderer?.capabilities?.isWebGL2) {
    console.warn('Skipping particle system: WebGL2 required.');
    return;
  }

  if (particlePoints) {
    scene.remove(particlePoints);
    particlePoints.geometry?.dispose();
    particlePoints.material?.dispose();
    particlePoints = null;
    particleMaterial = null;
  }

  // 端末負荷を考慮した粒子数（モバイルは控えめ）
  const count = isMobileDevice() ? 800 : 2000;
  const positions = new Float32Array(count * 3);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setDrawRange(0, count);

  // カメラ周囲のランダム配置範囲（x,y:幅 / z:奥行）
  const baseSpread = isMobileDevice()
    ? PARTICLE_BASE_SPREAD_MOBILE
    : PARTICLE_BASE_SPREAD_DESKTOP;
  const initialSpread = baseSpread.clone();
  const baseMaxDistance = isMobileDevice()
    ? PARTICLE_BASE_MAX_DISTANCE_MOBILE
    : PARTICLE_BASE_MAX_DISTANCE_DESKTOP;

  particleMaterial = new THREE.ShaderMaterial({
    glslVersion: THREE.GLSL3,
    transparent: true,
    depthWrite: false,
    depthTest: true,
    blending: THREE.AdditiveBlending,
    uniforms: {
      // 時間と空間配置
      uTime: { value: 0 },
      uOrigin: { value: new THREE.Vector3() },
      uFlowDir: { value: new THREE.Vector3(1, 0, 0).normalize() },
      uLat1: { value: new THREE.Vector3(0, 0, 1) },
      uLat2: { value: new THREE.Vector3(0, 1, 0) },
      uSpread: { value: initialSpread },
      uMaxDistance: { value: baseMaxDistance },
      // 振る舞いと描画
      uBaseSpeed: { value: 0.6 },
      uJitterAmp: { value: 0.22 },
      uSizePx: { value: isMobileDevice() ? 18.0 : 28.0 },
      uFadeNear: { value: 1.5 },
      uFadeFar: { value: 14.0 },
      uColorNear: { value: new THREE.Color(0x9fd6ff) },
      uColorFar: { value: new THREE.Color(0x0a5270) },
    },
    vertexShader: `
precision highp float;
precision highp int;

uniform float uTime;
uniform vec3 uOrigin;
uniform vec3 uFlowDir;
uniform vec3 uLat1;
uniform vec3 uLat2;
uniform vec3 uSpread;
uniform float uMaxDistance;
uniform float uBaseSpeed;
uniform float uJitterAmp;
uniform float uSizePx;
uniform float uFadeNear;
uniform float uFadeFar;

out float vFade;
out float vColorMix;

uint hashUint(uint n) {
  n ^= (n << 13);
  n ^= (n >> 17);
  n ^= (n << 5);
  return n * 0x27d4eb2du;
}

float hashFloat(uint n) {
  return float(hashUint(n)) / 4294967296.0;
}

vec3 hashVec3(uint n) {
  return vec3(
    hashFloat(n),
    hashFloat(n * 1664525u + 1013904223u),
    hashFloat(n * 22695477u + 1u)
  );
}

void main() {
  uint id = uint(gl_VertexID);
  vec3 randSeed = hashVec3(id) - 0.5;

  // ワールド固定の粒子場を生成
  vec3 flowDir = normalize(uFlowDir);
  vec3 lateral = uLat1 * (randSeed.x * uSpread.x)
    + uLat2 * (randSeed.y * uSpread.y);

  float speedSeed = hashFloat(id * 747796405u);
  float speed = uBaseSpeed * (0.7 + 0.6 * speedSeed);
  float flowCycle = max(uSpread.z, 1e-3);
  float flowParam = randSeed.z + (uTime * speed) / flowCycle;
  float wrappedZ = fract(flowParam) - 0.5;
  vec3 pos = uOrigin + lateral + flowDir * (wrappedZ * flowCycle);

  float phase = hashFloat(id * 1664525u) * 6.28318530718;
  float driftSeed = hashFloat(id * 22695477u);
  float triangle = 1.0 - abs(fract((phase + uTime * (0.8 + 0.4 * driftSeed)) / 3.14159265) * 2.0 - 1.0);
  // slow drift + ランダム揺らぎで単純な連続パターンを回避
  vec3 jitter1 = normalize(uLat1);
  vec3 jitter2 = normalize(uLat2);
  pos += jitter1 * ((triangle - 0.5) * uJitterAmp);
  pos += jitter2 * ((hashFloat(id * 1103515245u) - 0.5) * uJitterAmp * 0.6);

  vec4 mvPosition = modelViewMatrix * vec4(pos, 1.0);
  float dist = -mvPosition.z;
  float forwardMask = step(0.0, dist);

  vec4 clipPosition = projectionMatrix * mvPosition;
  float invW = 1.0 / max(abs(clipPosition.w), 1e-3);
  vec2 ndc = clipPosition.xy * invW;

  float screenMask = 1.0 - smoothstep(0.96, 1.16, max(abs(ndc.x), abs(ndc.y)));
  float rangeMask = 1.0 - smoothstep(uMaxDistance * 0.7, uMaxDistance, dist);

  float fadeT = clamp((dist - uFadeNear) / max(uFadeFar - uFadeNear, 1e-3), 0.0, 1.0);
  float fade = (1.0 - fadeT) * screenMask * rangeMask * forwardMask;
  vFade = fade;
  vColorMix = fade;

  float sizeBase = uSizePx * projectionMatrix[1][1] / max(dist, 1e-3);
  float attenuatedSize = sizeBase * fade;
  gl_PointSize = max(attenuatedSize, 0.75) * step(0.001, fade);
  gl_Position = clipPosition;
}
    `,
    fragmentShader: `
precision highp float;

in float vFade;
in float vColorMix;

uniform vec3 uColorNear;
uniform vec3 uColorFar;

out vec4 fragColor;

void main() {
  vec2 coord = gl_PointCoord - vec2(0.5);
  float falloff = exp(-8.0 * dot(coord, coord));
  float alpha = vFade * falloff * 0.6;
  if (alpha < 0.004) {
    discard;
  }
  vec3 color = mix(uColorFar, uColorNear, vColorMix);
  fragColor = vec4(color, alpha);
}
    `,
  });

  const baseTargetDistance = controls
    ? camera.position.distanceTo(controls.target)
    : camera.position.length();
  particleMaterial.userData.baseSpread = baseSpread.clone();
  particleMaterial.userData.baseMaxDistance = baseMaxDistance;
  particleMaterial.userData.baseTargetDistance = Math.max(baseTargetDistance, 0.1);

  setParticleWorldBasis(particleMaterial.uniforms.uFlowDir.value);

  particlePoints = new THREE.Points(geometry, particleMaterial);
  particlePoints.frustumCulled = false;
  particlePoints.renderOrder = 2;
  scene.add(particlePoints);

  updateParticleUniforms();
}

// 粒子場の直交基底をワールド座標で構築
function setParticleWorldBasis(flowDir) {
  if (!particleMaterial || !flowDir) {
    return;
  }

  const dir = PARTICLE_BASIS_TEMP_DIR.copy(flowDir).normalize();
  const axis = Math.abs(dir.y) < 0.99
    ? PARTICLE_BASIS_TEMP_AXIS.set(0, 1, 0)
    : PARTICLE_BASIS_TEMP_AXIS.set(1, 0, 0);

  const lat1 = PARTICLE_BASIS_LAT1;
  lat1.crossVectors(dir, axis);
  if (lat1.lengthSq() < 1e-6) {
    lat1.set(0, 0, 1);
    lat1.crossVectors(dir, lat1);
  }
  lat1.normalize();

  const lat2 = PARTICLE_BASIS_LAT2;
  lat2.crossVectors(dir, lat1).normalize();

  const uniforms = particleMaterial.uniforms;
  uniforms.uFlowDir.value.copy(dir);
  uniforms.uLat1.value.copy(lat1);
  uniforms.uLat2.value.copy(lat2);
}

// カメラ操作に応じて粒子ボリュームの中心とスケールを更新
function updateParticleUniforms() {
  if (!particleMaterial || !camera) {
    return;
  }

  const uniforms = particleMaterial.uniforms;
  const camPos = camera.position;
  uniforms.uOrigin.value.copy(camPos);

  const userData = particleMaterial.userData || {};
  const baseSpread = userData.baseSpread;
  const baseMaxDistance = userData.baseMaxDistance;
  if (baseSpread && baseMaxDistance) {
    const baseTargetDistance = userData.baseTargetDistance || 1;
    const currentTargetDistance = controls
      ? camera.position.distanceTo(controls.target)
      : baseTargetDistance;
    const scale = THREE.MathUtils.clamp(
      currentTargetDistance / Math.max(baseTargetDistance, 1e-3),
      0.6,
      2.5
    );
    uniforms.uSpread.value.copy(baseSpread).multiplyScalar(scale);
    uniforms.uMaxDistance.value = baseMaxDistance * scale;
  }
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

function createAttributeSet(count, itemSize, ArrayType = Float32Array) {
  const length = count * itemSize;
  const attributes = [];
  for (let i = 0; i < TRIPLE_BUFFER_SIZE; i++) {
    const attr = new THREE.InstancedBufferAttribute(new ArrayType(length), itemSize);
    attr.setUsage(STREAM_USAGE);
    attributes.push(attr);
  }
  return attributes;
}

function createBufferSet(count) {
  // トリプルバッファ用の属性セットを作成（位置・姿勢・尾アニメ制御用）
  return {
    pos: createAttributeSet(count, 3),
    quat: createAttributeSet(count, 4),
    tailPhase: createAttributeSet(count, 1),
    tailParams: createAttributeSet(count, 3, getTailParamsArrayType()), // xyz = speed, turn, drive
  };
}

function resetBufferSetToHidden(bufferSet) {
  for (const attr of bufferSet.pos) {
    const array = attr.array;
    for (let i = 0; i < array.length; i += 3) {
      array[i] = HIDDEN_POSITION;
      array[i + 1] = HIDDEN_POSITION;
      array[i + 2] = HIDDEN_POSITION;
    }
  }
  for (const attr of bufferSet.quat) {
    const array = attr.array;
    for (let i = 0; i < array.length; i += 4) {
      array[i] = 0;
      array[i + 1] = 0;
      array[i + 2] = 0;
      array[i + 3] = 1;
    }
  }
  for (const attr of bufferSet.tailPhase) {
    attr.array.fill(0);
  }
  for (const attr of bufferSet.tailParams) {
    attr.array.fill(0);
  }
}

function applyBufferSet(mesh, bufferSet, index) {
  // トリプルバッファの指定インデックスから属性をジオメトリに設定
  mesh.geometry.setAttribute('instancePos', bufferSet.pos[index]);
  mesh.geometry.setAttribute('instanceQuat', bufferSet.quat[index]);
  mesh.geometry.setAttribute('instanceTailPhase', bufferSet.tailPhase[index]);
  mesh.geometry.setAttribute('instanceTailParams', bufferSet.tailParams[index]);
}

function patchQuaternionInstancing(material) {
  if (material.userData?.instancingPatched) return;

  material.onBeforeCompile = (shader) => {
    Object.entries(tailAnimation.uniforms).forEach(([key, uniform]) => {
      shader.uniforms[key] = uniform;
    });

    shader.vertexShader = shader.vertexShader
      .replace(
        '#include <common>',
        `#include <common>\nattribute vec3 instancePos;\nattribute vec4 instanceQuat;\nattribute float aBodyCoord;\nattribute float instanceTailPhase;\nattribute vec3 instanceTailParams;\nuniform float uTailTime;\nuniform float uTailAmplitude;\nuniform float uTailFrequency;\nuniform float uTailPhaseStride;\nuniform float uTailTurnStrength;\nuniform float uTailSpeedScale;\nuniform vec3 uTailRight;\nuniform vec3 uTailForward;\nuniform vec3 uTailUp;\nuniform float uTailEnable;\nuniform sampler2D uSinLut;\nuniform float uLutSize;\nvec3 quatTransform(vec3 v, vec4 q) {\n  vec3 t = 2.0 * cross(q.xyz, v);\n  return v + q.w * t + cross(q.xyz, t);\n}\nvec2 sampleSinCos(float angle) {\n  float u = fract(angle * 0.15915494309189535);\n  u = u * (uLutSize - 1.0) + 0.5;\n  vec4 lutSample = texture(uSinLut, vec2(u / uLutSize, 0.5));\n  return lutSample.rg * 2.0 - 1.0;\n}`
      )
      .replace('#include <instancing_vertex>', `#ifdef USE_INSTANCING\n#endif`)
      .replace(
        '#include <begin_vertex>',
        `#include <begin_vertex>\nvec3 tailParams = instanceTailParams;\nif (uTailEnable > 0.5) {\n  float driveRaw = tailParams.z;\n  if (driveRaw > 0.01) {\n    float drive = clamp(driveRaw, 0.0, 1.0);\n    vec3 originalPos = transformed;\n    vec3 right = normalize(uTailRight);\n    vec3 forward = normalize(uTailForward);\n    vec3 up = normalize(uTailUp);\n\n    float localX = dot(originalPos, right);\n    float localY = dot(originalPos, forward);\n    float localZ = dot(originalPos, up);\n\n    float bodyCoord = clamp(aBodyCoord, 0.0, 1.0);\n    float tailWeight = smoothstep(0.0, 0.35, bodyCoord);\n    float speedFactor = clamp(tailParams.x * uTailSpeedScale, 0.0, 2.0);\n\n    float phase = instanceTailPhase + uTailTime * uTailFrequency;\n    float wavePhase = phase + bodyCoord * uTailPhaseStride;\n    vec2 waveSC = sampleSinCos(wavePhase);\n    float wag = waveSC.x * uTailAmplitude * drive;\n    float turnOffset = tailParams.y * uTailTurnStrength * drive;\n    float motion = wag * (0.4 + 0.6 * speedFactor) + turnOffset;\n\n    float tipDamping = 1.0 - smoothstep(0.7, 1.0, bodyCoord) * 0.3;\n    float bendStrength = mix(0.02, 1.0, tailWeight) * tipDamping;\n\n    float bendAngle = motion * bendStrength;\n    vec2 bendSC = sampleSinCos(bendAngle);\n    float s = bendSC.x;\n    float c = bendSC.y;\n    float rotX = localX * c - localY * s;\n    float rotY = localX * s + localY * c;\n\n    float sway = motion * 0.4 * tipDamping;\n    rotX += sway * (0.05 + 0.95 * bodyCoord);\n\n    vec3 rotated = right * rotX + forward * rotY + up * localZ;\n    transformed = rotated;\n  }\n}\ntransformed = quatTransform(transformed, instanceQuat) + instancePos;`
      )
      .replace(
        '#include <beginnormal_vertex>',
        '#include <beginnormal_vertex>\nobjectNormal = quatTransform(objectNormal, instanceQuat);'
      )
      .replace(
        '#include <project_vertex>',
        `#include <project_vertex>\n#ifdef DEPTH_PACKING\nvec3 tailParams = instanceTailParams;\nif (uTailEnable > 0.5) {\n  float driveRaw = tailParams.z;\n  if (driveRaw > 0.01) {\n    float drive = clamp(driveRaw, 0.0, 1.0);\n    vec3 viewPos = mvPosition.xyz;\n    mat3 normalMatrix3 = mat3(normalMatrix);\n    vec3 right = normalize(normalMatrix3 * uTailRight);\n    vec3 forward = normalize(normalMatrix3 * uTailForward);\n    vec3 up = normalize(normalMatrix3 * uTailUp);\n\n    float localX = dot(viewPos, right);\n    float localY = dot(viewPos, forward);\n    float localZ = dot(viewPos, up);\n\n    float bodyCoord = clamp(aBodyCoord, 0.0, 1.0);\n    float tailWeight = smoothstep(0.0, 0.35, bodyCoord);\n    float speedFactor = clamp(tailParams.x * uTailSpeedScale, 0.0, 2.0);\n\n    float phase = instanceTailPhase + uTailTime * uTailFrequency;\n    float wavePhase = phase + bodyCoord * uTailPhaseStride;\n    vec2 waveSC = sampleSinCos(wavePhase);\n    float wag = waveSC.x * uTailAmplitude * drive;\n    float turnOffset = tailParams.y * uTailTurnStrength * drive;\n    float motion = wag * (0.4 + 0.6 * speedFactor) + turnOffset;\n\n    float tipDamping = 1.0 - smoothstep(0.7, 1.0, bodyCoord) * 0.3;\n    float bendStrength = mix(0.02, 1.0, tailWeight) * tipDamping;\n\n    float bendAngle = motion * bendStrength;\n    vec2 bendSC = sampleSinCos(bendAngle);\n    float s = bendSC.x;\n    float c = bendSC.y;\n    float rotX = localX * c - localY * s;\n    float rotY = localX * s + localY * c;\n\n    float sway = motion * 0.4 * tipDamping;\n    rotX += sway * (0.05 + 0.95 * bodyCoord);\n\n    vec3 rotated = right * rotX + forward * rotY + up * localZ;\n    mvPosition.xyz = rotated;\n    gl_Position = projectionMatrix * mvPosition;\n  }\n}\n#endif`
      );

    material.userData = {
      ...(material.userData || {}),
      instancingPatched: true,
    };
  };

  material.needsUpdate = true;
  instancingMaterials.add(material);
}

// シャドウレンダリングでもインスタンス属性を適用する深度マテリアル
function createInstancedDepthMaterial(sourceMaterial) {
  const depthMaterial = new THREE.MeshDepthMaterial({
    depthPacking: THREE.RGBADepthPacking,
    alphaTest: sourceMaterial.alphaTest ?? 0,
  });
  depthMaterial.map = sourceMaterial.map ?? null;
  depthMaterial.alphaMap = sourceMaterial.alphaMap ?? null;
  depthMaterial.transparent = sourceMaterial.transparent ?? false;
  patchQuaternionInstancing(depthMaterial);
  return depthMaterial;
}

// ポイントライト用の距離マテリアルもインスタンス挙動に合わせる
function createInstancedDistanceMaterial(sourceMaterial) {
  const distanceMaterial = new THREE.MeshDistanceMaterial({
    alphaTest: sourceMaterial.alphaTest ?? 0,
  });
  distanceMaterial.map = sourceMaterial.map ?? null;
  distanceMaterial.alphaMap = sourceMaterial.alphaMap ?? null;
  distanceMaterial.transparent = sourceMaterial.transparent ?? false;
  patchQuaternionInstancing(distanceMaterial);
  return distanceMaterial;
}

function stepSimulationAndUpdateState(deltaTime) {
  const statePtr = stepSimulation(deltaTime);
  if (!statePtr) {
    return latestBoidCountFromWasm;
  }

  latestStatePtr = statePtr;

  const heapU8Buffer = wasmModule.HEAPU8.buffer;
  if (
    !latestStateHeaderView ||
    latestStateHeaderPtr !== statePtr ||
    latestStateHeaderView.buffer !== heapU8Buffer
  ) {
    latestStateHeaderView = new DataView(heapU8Buffer, statePtr, 16);
    latestStateHeaderPtr = statePtr;
  }

  latestPositionsPtr = latestStateHeaderView.getUint32(0, true);
  latestVelocitiesPtr = latestStateHeaderView.getUint32(4, true);
  latestOrientationsPtr = latestStateHeaderView.getUint32(8, true);
  latestBoidCountFromWasm = latestStateHeaderView.getInt32(12, true);

  return latestBoidCountFromWasm;
}

function getWasmViews(count) {
  const heapBuffer = wasmModule.HEAPF32.buffer;

  if (cachedHeapBuffer !== heapBuffer) {
    cachedHeapBuffer = heapBuffer;
    cachedPositionsPtr = 0;
    cachedOrientationsPtr = 0;
    cachedVelocitiesPtr = 0;
    cachedPositionsView = null;
    cachedOrientationsView = null;
    cachedVelocitiesView = null;
    cachedPositionsCount = 0;
    cachedOrientationsCount = 0;
    cachedVelocitiesCount = 0;
  }

  const posPointer = latestPositionsPtr;
  const oriPointer = latestOrientationsPtr;
  const velPointer = latestVelocitiesPtr;

  if (!posPointer || !oriPointer || !velPointer || count <= 0) {
    return {
      positions: cachedPositionsView ?? new Float32Array(0),
      orientations: cachedOrientationsView ?? new Float32Array(0),
      velocities: cachedVelocitiesView ?? new Float32Array(0),
    };
  }

  if (
    cachedPositionsView === null ||
    cachedPositionsCount !== count ||
    cachedPositionsPtr !== posPointer
  ) {
    cachedPositionsPtr = posPointer;
    cachedPositionsView = new Float32Array(heapBuffer, cachedPositionsPtr, count * 3);
    cachedPositionsCount = count;
  }

  if (
    cachedOrientationsView === null ||
    cachedOrientationsCount !== count ||
    cachedOrientationsPtr !== oriPointer
  ) {
    cachedOrientationsPtr = oriPointer;
    cachedOrientationsView = new Float32Array(heapBuffer, cachedOrientationsPtr, count * 4);
    cachedOrientationsCount = count;
  }

  if (
    cachedVelocitiesView === null ||
    cachedVelocitiesCount !== count ||
    cachedVelocitiesPtr !== velPointer
  ) {
    cachedVelocitiesPtr = velPointer;
    cachedVelocitiesView = new Float32Array(heapBuffer, cachedVelocitiesPtr, count * 3);
    cachedVelocitiesCount = count;
  }

  return {
    positions: cachedPositionsView,
    orientations: cachedOrientationsView,
    velocities: cachedVelocitiesView,
  };
}

function getSpeciesIdView(count) {
  const ptr = speciesIdsPtr ? speciesIdsPtr() : 0;
  if (!ptr || count <= 0) {
    return cachedSpeciesIdsView ?? new Int32Array(0);
  }

  const heapBuffer32 = wasmModule.HEAP32.buffer;
  if (
    cachedSpeciesIdsView === null ||
    cachedSpeciesIdsPtr !== ptr ||
    cachedSpeciesIdsCount !== count ||
    cachedSpeciesIdsBuffer !== heapBuffer32
  ) {
    cachedSpeciesIdsPtr = ptr;
    cachedSpeciesIdsCount = count;
    cachedSpeciesIdsBuffer = heapBuffer32;
    cachedSpeciesIdsView = new Int32Array(heapBuffer32, ptr, count);
  }

  return cachedSpeciesIdsView;
}

function buildSpeciesRanges(list = []) {
  const ranges = [];
  let offset = 0;
  for (const item of list) {
    if (!item) continue;
    const count = Math.max(0, Number(item.count) || 0);
    ranges.push({
      name: item.species || '',
      start: offset,
      count,
    });
    offset += count;
  }
  return ranges;
}

function captureFlockState() {
  if (!wasmModule) return null;
  const currentCount = stepSimulationAndUpdateState(0);
  if (!currentCount || currentCount <= 0) {
    return { count: 0 };
  }

  const { positions, velocities, orientations } = getWasmViews(currentCount);
  const speciesIdsView = getSpeciesIdView(currentCount);

  return {
    count: currentCount,
    positions: positions ? new Float32Array(positions) : null,
    velocities: velocities ? new Float32Array(velocities) : null,
    orientations: orientations ? new Float32Array(orientations) : null,
    speciesIds: speciesIdsView ? new Int32Array(speciesIdsView) : null,
  };
}

function restoreFlockState(previousState, oldSettings, newSettings) {
  if (!previousState || previousState.count <= 0) {
    return;
  }

  const currentCount = stepSimulationAndUpdateState(0);
  if (!currentCount || currentCount <= 0) {
    return;
  }

  const { positions, velocities, orientations } = getWasmViews(currentCount);
  if (!positions || !velocities || !orientations) {
    return;
  }

  const prevPositions = previousState.positions;
  const prevVelocities = previousState.velocities;
  const prevOrientations = previousState.orientations;

  const oldRanges = buildSpeciesRanges(oldSettings);
  const newRanges = buildSpeciesRanges(newSettings);
  const oldRangeMap = new Map(oldRanges.map((info) => [info.name, info]));

  let restored = 0;

  for (const newInfo of newRanges) {
    if (!newInfo.count) continue;
    const oldInfo = oldRangeMap.get(newInfo.name);
    if (!oldInfo || !oldInfo.count) continue;

    const transferable = Math.min(oldInfo.count, newInfo.count);
    for (let i = 0; i < transferable; i += 1) {
      const srcIndex = oldInfo.start + i;
      const dstIndex = newInfo.start + i;
      if (srcIndex >= previousState.count || dstIndex >= currentCount) {
        break;
      }

      if (prevPositions) {
        const srcPosBase = srcIndex * 3;
        const dstPosBase = dstIndex * 3;
        positions[dstPosBase] = prevPositions[srcPosBase];
        positions[dstPosBase + 1] = prevPositions[srcPosBase + 1];
        positions[dstPosBase + 2] = prevPositions[srcPosBase + 2];
      }

      if (prevVelocities) {
        const srcVelBase = srcIndex * 3;
        const dstVelBase = dstIndex * 3;
        velocities[dstVelBase] = prevVelocities[srcVelBase];
        velocities[dstVelBase + 1] = prevVelocities[srcVelBase + 1];
        velocities[dstVelBase + 2] = prevVelocities[srcVelBase + 2];
      }

      if (prevOrientations) {
        const srcOriBase = srcIndex * 4;
        const dstOriBase = dstIndex * 4;
        orientations[dstOriBase] = prevOrientations[srcOriBase];
        orientations[dstOriBase + 1] = prevOrientations[srcOriBase + 1];
        orientations[dstOriBase + 2] = prevOrientations[srcOriBase + 2];
        orientations[dstOriBase + 3] = prevOrientations[srcOriBase + 3];
      }
      restored += 1;
    }
  }

  if (restored > 0 && typeof syncReadToWriteBuffers === 'function') {
    syncReadToWriteBuffers();
  }
}

// LOD用ジオメトリ・マテリアルを使い回す
const boidGeometryHigh = new THREE.SphereGeometry(1, 8, 8);
const boidGeometryLow = new THREE.SphereGeometry(1, 3, 2);
boidGeometryHigh.scale(0.5, 0.5, 2.0); // 少し小さくする
boidGeometryLow.scale(0.5, 0.5, 2.0); // 少し小さくする
let boidModel = null; // 読み込んだモデルを保持
let boidModelLod = null; // 読み込んだモデルを保持
let originalMaterial = null; // 元のマテリアルを保持
let originalMaterialLod = null; // 元のLODマテリアルを保持
let predatorModel = null;
let predatorMaterial = null;
let predatorMeshes = [];
let predatorMeshCountCache = -1;

// 起動時の正しいテクスチャマテリアルを保持
let originalHighMat = null;
let originalLowMat = null;

// 前回のshowUnitColorsの状態を保持（OFF→ONの検知用）
let lastShowUnitColors = false;

function getPredatorCount() {
  return settings.reduce((sum, s) => sum + ((s.isPredator && s.count) ? s.count : 0), 0);
}

function ensurePredatorMeshes(count) {
  if (!predatorModel) return false;

  while (predatorMeshes.length > count) {
    const mesh = predatorMeshes.pop();
    if (mesh) {
      scene.remove(mesh);
    }
  }

  while (predatorMeshes.length < count) {
    const clone = predatorModel.clone(true);
    clone.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    clone.visible = false;
    scene.add(clone);
    predatorMeshes.push(clone);
  }

  return predatorMeshes.length === count;
}

function createTailPhaseArray(count) {
  const array = new Float32Array(count);
  for (let i = 0; i < count; i++) {
    array[i] = Math.random() * Math.PI * 2;
  }
  return array;
}

function applyTailPhaseSeeds(bufferSet, seeds) {
  if (!bufferSet || !bufferSet.tailPhase) return;
  for (const attr of bufferSet.tailPhase) {
    attr.array.set(seeds);
    attr.needsUpdate = true;
  }
}

function ensureTailRuntimeBuffers(count) {
  if (!previousVelocities || previousVelocities.length !== count * 3) {
    previousVelocities = new Float32Array(count * 3);
  }
}

function ensureLodFlagBuffer(count) {
  if (count <= 0) {
    previousLodFlags = null;
    return null;
  }
  if (!previousLodFlags || previousLodFlags.length !== count) {
    previousLodFlags = new Uint8Array(count);
  }
  return previousLodFlags;
}

function ensureBodyCoordAttribute(geometry) {
  if (geometry.getAttribute('aBodyCoord')) return;
  const position = geometry.getAttribute('position');
  if (!position) return;

  const count = position.count;
  const array = position.array;
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;

  for (let i = 0; i < count; i++) {
    const ix = i * 3;
    const x = array[ix];
    const y = array[ix + 1];
    const z = array[ix + 2];
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
  }

  const rangeX = maxX - minX;
  const rangeY = maxY - minY;
  const rangeZ = maxZ - minZ;
  let axis = 'z';
  let min = minZ;
  let range = rangeZ;

  if (rangeX > rangeY && rangeX > rangeZ) {
    axis = 'x';
    min = minX;
    range = rangeX;
  } else if (rangeY > rangeZ) {
    axis = 'y';
    min = minY;
    range = rangeY;
  }

  const bodyCoord = new Float32Array(count);
  const denom = range > 0 ? range : 1;
  for (let i = 0; i < count; i++) {
    const ix = i * 3;
    const value = axis === 'x' ? array[ix] : axis === 'y' ? array[ix + 1] : array[ix + 2];
    bodyCoord[i] = (value - min) / denom;
  }

  geometry.setAttribute('aBodyCoord', new THREE.BufferAttribute(bodyCoord, 1));
}

function getTotalBoidCount() {
  return settings.reduce((sum, s) => sum + (s.count || 0), 0);
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
  const vector = new wasmModule.VectorSpeciesParams();
  settings.forEach((s, index) => {
    const tau = typeof s.tau === 'number' ? s.tau : 0.0;
    vector.push_back({
      species: s.species || `Species ${index + 1}`,
      count: s.count || 0,
      speciesId: index,
      cohesion: s.cohesion || 0.0,
      separation: s.separation || 0.0,
      alignment: s.alignment || 0.0,
      maxSpeed: s.maxSpeed || 1.0,
      minSpeed: s.minSpeed || 0.0,
      maxTurnAngle: s.maxTurnAngle || 0.0,
      separationRange: s.separationRange || 0.0,
      alignmentRange: s.alignmentRange || 0.0,
      cohesionRange: s.cohesionRange || 0.0,
      maxNeighbors: s.maxNeighbors || 0,
      lambda: s.lambda || 0.0,
      horizontalTorque: s.horizontalTorque || 0.0,
      velocityEpsilon: s.velocityEpsilon || 0.0,
      torqueStrength: s.torqueStrength || 0.0,
      tau,
      isPredator: s.isPredator || false,
    });
  });
  return vector;
}

function reinitializeFlockNow() {
  if (!wasmModule) return;

  const pendingState = pendingStateForReinitialize?.state || null;
  const oldSettingsRef = pendingStateForReinitialize?.oldSettings || previousSettingsSnapshot;
  const newSettingsRef = pendingSettingsSnapshot || snapshotSettingsList(settings);
  const targetSignature = getSpeciesSignature(newSettingsRef);

  const vector = createSpeciesParamsVector();
  wasmModule.callInitBoids(vector, 1, 6, 0.25);
  build(16, 0);
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
  if (!boidModel.children || !boidModel.children[0]) {
    console.error('Boid model does not have valid children.');
    return;
  }

  // 既存のメッシュを削除
  if (instancedMeshHigh) {
    scene.remove(instancedMeshHigh);
    instancingMaterials.delete(instancedMeshHigh.material);
  }
  if (instancedMeshLow) {
    scene.remove(instancedMeshLow);
    instancingMaterials.delete(instancedMeshLow.material);
  }
  instancingMaterials.clear();
  // InstancedMeshを作成（最初はvertexColors無効でテクスチャ表示）
  const highMaterial = originalMaterial.clone();
  highMaterial.vertexColors = false; // 最初はテクスチャ表示

  const lowMaterial = originalMaterialLod.clone();
  lowMaterial.vertexColors = false; // 最初はテクスチャ表示

  patchQuaternionInstancing(highMaterial);
  patchQuaternionInstancing(lowMaterial);

  const highGeometry = boidModel.children[0].geometry;
  const lowGeometry = boidModelLod.children[0].geometry;
  ensureBodyCoordAttribute(highGeometry);
  ensureBodyCoordAttribute(lowGeometry);

  instancedMeshHigh = new THREE.InstancedMesh(
    highGeometry,
    highMaterial,
    count
  );
  instancedMeshHigh.castShadow = true;
  instancedMeshHigh.receiveShadow = true;
  instancedMeshHigh.frustumCulled = false;
  instancedMeshHigh.customDepthMaterial = createInstancedDepthMaterial(highMaterial);
  instancedMeshHigh.customDistanceMaterial = createInstancedDistanceMaterial(highMaterial);

  instancedMeshLow = new THREE.InstancedMesh(
    lowGeometry,
    lowMaterial,
    count
  );
  instancedMeshLow.castShadow = true;
  instancedMeshLow.receiveShadow = true;
  instancedMeshLow.frustumCulled = false;
  instancedMeshLow.customDepthMaterial = createInstancedDepthMaterial(lowMaterial);
  instancedMeshLow.customDistanceMaterial = createInstancedDistanceMaterial(lowMaterial);

  // インスタンスカラーを白で初期化
  const whiteColor = new THREE.Color(1, 1, 1);
  for (let i = 0; i < count; i++) {
    instancedMeshHigh.setColorAt(i, whiteColor);
    instancedMeshLow.setColorAt(i, whiteColor);
  }
  instancedMeshHigh.instanceColor.needsUpdate = true;
  instancedMeshLow.instanceColor.needsUpdate = true;

  // シーンに追加
  scene.add(instancedMeshHigh);
  scene.add(instancedMeshLow);

  bufferSetHigh = createBufferSet(count);
  bufferSetLow = createBufferSet(count);
  resetBufferSetToHidden(bufferSetHigh);
  resetBufferSetToHidden(bufferSetLow);
  tailPhaseSeeds = createTailPhaseArray(count);
  applyTailPhaseSeeds(bufferSetHigh, tailPhaseSeeds);
  applyTailPhaseSeeds(bufferSetLow, tailPhaseSeeds);
  bufferCursor = 0;
  applyBufferSet(instancedMeshHigh, bufferSetHigh, bufferCursor);
  applyBufferSet(instancedMeshLow, bufferSetLow, bufferCursor);

  instancedMeshHigh.instanceMatrix.setUsage(THREE.StaticDrawUsage);
  instancedMeshLow.instanceMatrix.setUsage(THREE.StaticDrawUsage);
  instancedMeshHigh.count = count;
  instancedMeshLow.count = count;

  cachedPositionsView = null;
  cachedOrientationsView = null;
  cachedPositionsCount = 0;
  cachedOrientationsCount = 0;
  cachedPositionsPtr = 0;
  cachedOrientationsPtr = 0;
  cachedVelocitiesView = null;
  cachedVelocitiesCount = 0;
  cachedVelocitiesPtr = 0;
  previousVelocities = null;
  shaderTime = 0;
  cachedSpeciesIdsView = null;
  cachedSpeciesIdsPtr = 0;
  cachedSpeciesIdsCount = 0;
  cachedSpeciesIdsBuffer = null;
  ensureTailRuntimeBuffers(count);
  const lodFlags = ensureLodFlagBuffer(count);
  if (lodFlags) {
    lodFlags.fill(0);
  }

  const predators = getPredatorCount();
  if (ensurePredatorMeshes(predators)) {
    predatorMeshCountCache = predators;
  }
  for (const mesh of predatorMeshes) {
    mesh.visible = false;
  }

  console.log('InstancedMeshes created with vertex colors enabled');
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
    vertexColors: true, // 通常時は無効
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
      predatorMeshCountCache = -1;
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
    if (particleMaterial) {
      particleMaterial.uniforms.uTime.value += deltaTime;
    }
  }
  updateInstancingMaterialUniforms(shaderTime);

  const count = stepSimulationAndUpdateState(paused.value ? 0 : deltaTime);
  if (!instancedMeshHigh || !instancedMeshLow || !bufferSetHigh || !bufferSetLow) {
    controls.update();
    updateParticleUniforms();
    (isMobileDevice() ? renderer : composer).render(scene, camera);
    stats?.end();
    stats?.update();
    animationTimer = setTimeout(animate, FRAME_INTERVAL);
    return;
  }
  const { positions, orientations, velocities } = getWasmViews(count);
  if ((frameCounter++ & 63) === 0) {
    currentFirstBoidX();
  }
  ensureTailRuntimeBuffers(count);

  const currentIndex = bufferCursor;
  const nextIndex = (currentIndex + 1) % TRIPLE_BUFFER_SIZE;
  const highPosAttr = bufferSetHigh.pos[currentIndex];
  const highQuatAttr = bufferSetHigh.quat[currentIndex];
  const lowPosAttr = bufferSetLow.pos[currentIndex];
  const lowQuatAttr = bufferSetLow.quat[currentIndex];
  const highTailParamsAttr = bufferSetHigh.tailParams[currentIndex];
  const lowTailParamsAttr = bufferSetLow.tailParams[currentIndex];
  const highPosArray = highPosAttr.array;
  const highQuatArray = highQuatAttr.array;
  const lowPosArray = lowPosAttr.array;
  const lowQuatArray = lowQuatAttr.array;
  const highTailParamsArray = highTailParamsAttr.array;
  const lowTailParamsArray = lowTailParamsAttr.array;
  const lodFlags = ensureLodFlagBuffer(count);

  let highTransformTouched = false;
  let lowTransformTouched = false;
  let highTailTouched = false;
  let lowTailTouched = false;

  const camPos = camera.position;
  const camX = camPos.x;
  const camY = camPos.y;
  const camZ = camPos.z;
  // 捕食者の個体数と開始インデックスを計算（配列の末尾から捕食者を配置）
  const predatorCount = getPredatorCount();
  const predatorStartIndex = predatorCount > 0 ? Math.max(0, count - predatorCount) : count;
  if (predatorMeshCountCache !== predatorCount) {
    if (ensurePredatorMeshes(predatorCount)) {
      predatorMeshCountCache = predatorCount;
    }
  }
  for (const mesh of predatorMeshes) {
    mesh.visible = false;
  }

  const hiddenPos = HIDDEN_POSITION;

  // 各Boidの位置と姿勢を更新
  for (let i = 0; i < count; ++i) {
    const basePos = i * 3;
    const px = positions[basePos];
    const py = positions[basePos + 1];
    const pz = positions[basePos + 2];
    const vx = velocities ? velocities[basePos] : 0;
    const vy = velocities ? velocities[basePos + 1] : 0;
    const vz = velocities ? velocities[basePos + 2] : 0;
    const dx = px - camX;
    const dy = py - camY;
    const dz = pz - camZ;
    const distSq = dx * dx + dy * dy + dz * dz;
    // 3段階LOD: 近距離/中距離/遠距離を距離の平方で判定
    const isNear = distSq < LOD_NEAR_DISTANCE_SQ;
    const isMid = !isNear && distSq < LOD_MID_DISTANCE_SQ;
    const animateTail = isNear || isMid; // 遠距離では尾アニメを停止してGPU負荷削減
    const baseQuat = i * 4;
    const qx = orientations[baseQuat];
    const qy = orientations[baseQuat + 1];
    const qz = orientations[baseQuat + 2];
    const qw = orientations[baseQuat + 3];
    const tailIndex = i * 3;
    // 捕食者は専用メッシュで描画し、インスタンシングからは除外
    const isPredator = i >= predatorStartIndex && predatorCount > 0;
    if (isPredator) {
      const meshIndex = i - predatorStartIndex;
      const predatorMesh = predatorMeshes[meshIndex];
      if (predatorMesh) {
        predatorMesh.visible = true;
        predatorMesh.position.set(px, py, pz);
        predatorMesh.quaternion.set(qx, qy, qz, qw);
      }
      const highNeedsHide =
        highPosArray[basePos] !== hiddenPos ||
        highPosArray[basePos + 1] !== hiddenPos ||
        highPosArray[basePos + 2] !== hiddenPos ||
        highQuatArray[baseQuat] !== IDENTITY_QUATERNION[0] ||
        highQuatArray[baseQuat + 1] !== IDENTITY_QUATERNION[1] ||
        highQuatArray[baseQuat + 2] !== IDENTITY_QUATERNION[2] ||
        highQuatArray[baseQuat + 3] !== IDENTITY_QUATERNION[3];
      if (highNeedsHide) {
        highPosArray[basePos] = hiddenPos;
        highPosArray[basePos + 1] = hiddenPos;
        highPosArray[basePos + 2] = hiddenPos;
        highQuatArray[baseQuat] = IDENTITY_QUATERNION[0];
        highQuatArray[baseQuat + 1] = IDENTITY_QUATERNION[1];
        highQuatArray[baseQuat + 2] = IDENTITY_QUATERNION[2];
        highQuatArray[baseQuat + 3] = IDENTITY_QUATERNION[3];
        highTransformTouched = true;
      }
      const lowNeedsHide =
        lowPosArray[basePos] !== hiddenPos ||
        lowPosArray[basePos + 1] !== hiddenPos ||
        lowPosArray[basePos + 2] !== hiddenPos ||
        lowQuatArray[baseQuat] !== IDENTITY_QUATERNION[0] ||
        lowQuatArray[baseQuat + 1] !== IDENTITY_QUATERNION[1] ||
        lowQuatArray[baseQuat + 2] !== IDENTITY_QUATERNION[2] ||
        lowQuatArray[baseQuat + 3] !== IDENTITY_QUATERNION[3];
      if (lowNeedsHide) {
        lowPosArray[basePos] = hiddenPos;
        lowPosArray[basePos + 1] = hiddenPos;
        lowPosArray[basePos + 2] = hiddenPos;
        lowQuatArray[baseQuat] = IDENTITY_QUATERNION[0];
        lowQuatArray[baseQuat + 1] = IDENTITY_QUATERNION[1];
        lowQuatArray[baseQuat + 2] = IDENTITY_QUATERNION[2];
        lowQuatArray[baseQuat + 3] = IDENTITY_QUATERNION[3];
        lowTransformTouched = true;
      }
      const highTailNonZero =
        highTailParamsArray[tailIndex] !== 0 ||
        highTailParamsArray[tailIndex + 1] !== 0 ||
        highTailParamsArray[tailIndex + 2] !== 0;
      if (highTailNonZero) {
        highTailParamsArray[tailIndex] = 0;
        highTailParamsArray[tailIndex + 1] = 0;
        highTailParamsArray[tailIndex + 2] = 0;
        highTailTouched = true;
      }
      const lowTailNonZero =
        lowTailParamsArray[tailIndex] !== 0 ||
        lowTailParamsArray[tailIndex + 1] !== 0 ||
        lowTailParamsArray[tailIndex + 2] !== 0;
      if (lowTailNonZero) {
        lowTailParamsArray[tailIndex] = 0;
        lowTailParamsArray[tailIndex + 1] = 0;
        lowTailParamsArray[tailIndex + 2] = 0;
        lowTailTouched = true;
      }
      if (lodFlags) {
        lodFlags[i] = 0;
      }
      if (previousVelocities) {
        previousVelocities[basePos] = 0;
        previousVelocities[basePos + 1] = 0;
        previousVelocities[basePos + 2] = 0;
      }
      continue;
    }

    // 尾アニメーション用の速度と旋回量を計算
    const speed = Math.hypot(vx, vy, vz);
    const prevVx = previousVelocities ? previousVelocities[basePos] : 0;
    const prevVy = previousVelocities ? previousVelocities[basePos + 1] : 0;
    const prevVz = previousVelocities ? previousVelocities[basePos + 2] : 0;
    const prevLen = Math.hypot(prevVx, prevVy, prevVz);
    let turnAmount = 0;
    if (prevLen > 1e-5 && speed > 1e-5) {
      const crossY = prevVz * vx - prevVx * vz;
      const dot = prevVx * vx + prevVy * vy + prevVz * vz;
      turnAmount = Math.atan2(crossY, dot);
    }
    const tailSpeed = speed;
    const tailTurn = turnAmount;

    // LOD制御: 近距離ならハイポリ、中距離以降はLODモデル
    if (isNear) {
      highPosArray[basePos] = px;
      highPosArray[basePos + 1] = py;
      highPosArray[basePos + 2] = pz;
      highQuatArray[baseQuat] = qx;
      highQuatArray[baseQuat + 1] = qy;
      highQuatArray[baseQuat + 2] = qz;
      highQuatArray[baseQuat + 3] = qw;
      highTransformTouched = true;

      const lowNeedsHide =
        lowPosArray[basePos] !== hiddenPos ||
        lowPosArray[basePos + 1] !== hiddenPos ||
        lowPosArray[basePos + 2] !== hiddenPos ||
        lowQuatArray[baseQuat] !== IDENTITY_QUATERNION[0] ||
        lowQuatArray[baseQuat + 1] !== IDENTITY_QUATERNION[1] ||
        lowQuatArray[baseQuat + 2] !== IDENTITY_QUATERNION[2] ||
        lowQuatArray[baseQuat + 3] !== IDENTITY_QUATERNION[3];
      if (lowNeedsHide) {
        lowPosArray[basePos] = hiddenPos;
        lowPosArray[basePos + 1] = hiddenPos;
        lowPosArray[basePos + 2] = hiddenPos;
        lowQuatArray[baseQuat] = IDENTITY_QUATERNION[0];
        lowQuatArray[baseQuat + 1] = IDENTITY_QUATERNION[1];
        lowQuatArray[baseQuat + 2] = IDENTITY_QUATERNION[2];
        lowQuatArray[baseQuat + 3] = IDENTITY_QUATERNION[3];
        lowTransformTouched = true;
      }
      const lowTailNonZero =
        lowTailParamsArray[tailIndex] !== 0 ||
        lowTailParamsArray[tailIndex + 1] !== 0 ||
        lowTailParamsArray[tailIndex + 2] !== 0;
      if (lowTailNonZero) {
        lowTailParamsArray[tailIndex] = 0;
        lowTailParamsArray[tailIndex + 1] = 0;
        lowTailParamsArray[tailIndex + 2] = 0;
        lowTailTouched = true;
      }
    } else {
      lowPosArray[basePos] = px;
      lowPosArray[basePos + 1] = py;
      lowPosArray[basePos + 2] = pz;
      lowQuatArray[baseQuat] = qx;
      lowQuatArray[baseQuat + 1] = qy;
      lowQuatArray[baseQuat + 2] = qz;
      lowQuatArray[baseQuat + 3] = qw;
      lowTransformTouched = true;

      const highNeedsHide =
        highPosArray[basePos] !== hiddenPos ||
        highPosArray[basePos + 1] !== hiddenPos ||
        highPosArray[basePos + 2] !== hiddenPos ||
        highQuatArray[baseQuat] !== IDENTITY_QUATERNION[0] ||
        highQuatArray[baseQuat + 1] !== IDENTITY_QUATERNION[1] ||
        highQuatArray[baseQuat + 2] !== IDENTITY_QUATERNION[2] ||
        highQuatArray[baseQuat + 3] !== IDENTITY_QUATERNION[3];
      if (highNeedsHide) {
        highPosArray[basePos] = hiddenPos;
        highPosArray[basePos + 1] = hiddenPos;
        highPosArray[basePos + 2] = hiddenPos;
        highQuatArray[baseQuat] = IDENTITY_QUATERNION[0];
        highQuatArray[baseQuat + 1] = IDENTITY_QUATERNION[1];
        highQuatArray[baseQuat + 2] = IDENTITY_QUATERNION[2];
        highQuatArray[baseQuat + 3] = IDENTITY_QUATERNION[3];
        highTransformTouched = true;
      }
      const highTailNonZero =
        highTailParamsArray[tailIndex] !== 0 ||
        highTailParamsArray[tailIndex + 1] !== 0 ||
        highTailParamsArray[tailIndex + 2] !== 0;
      if (highTailNonZero) {
        highTailParamsArray[tailIndex] = 0;
        highTailParamsArray[tailIndex + 1] = 0;
        highTailParamsArray[tailIndex + 2] = 0;
        highTailTouched = true;
      }
    }

    // 尾アニメのパラメータ設定（遠距離ではdrive=0でシェーダ側で計算をスキップ）
    const tailSpeedValue = animateTail ? tailSpeed : 0;
    const tailTurnValue = animateTail ? tailTurn : 0;
    const driveValue = animateTail ? 1 : 0; // シェーダ側で尾アニメON/OFFを制御
    if (isNear) {
      // 近距離: ハイポリモデルにアニメ適用
      highTailParamsArray[tailIndex] = tailSpeedValue;
      highTailParamsArray[tailIndex + 1] = tailTurnValue;
      highTailParamsArray[tailIndex + 2] = driveValue;
      highTailTouched = true;
      const lowTailNonZero =
        lowTailParamsArray[tailIndex] !== 0 ||
        lowTailParamsArray[tailIndex + 1] !== 0 ||
        lowTailParamsArray[tailIndex + 2] !== 0;
      if (lowTailNonZero) {
        lowTailParamsArray[tailIndex] = 0;
        lowTailParamsArray[tailIndex + 1] = 0;
        lowTailParamsArray[tailIndex + 2] = 0;
        lowTailTouched = true;
      }
    } else {
      // 中距離/遠距離: LODモデルに切替、遠距離ではdriveValue=0で静止
      lowTailParamsArray[tailIndex] = tailSpeedValue;
      lowTailParamsArray[tailIndex + 1] = tailTurnValue;
      lowTailParamsArray[tailIndex + 2] = driveValue;
      lowTailTouched = true;
      const highTailNonZero =
        highTailParamsArray[tailIndex] !== 0 ||
        highTailParamsArray[tailIndex + 1] !== 0 ||
        highTailParamsArray[tailIndex + 2] !== 0;
      if (highTailNonZero) {
        highTailParamsArray[tailIndex] = 0;
        highTailParamsArray[tailIndex + 1] = 0;
        highTailParamsArray[tailIndex + 2] = 0;
        highTailTouched = true;
      }
    }

    if (lodFlags) {
      lodFlags[i] = isNear ? 1 : 2;
    }

    if (previousVelocities) {
      previousVelocities[basePos] = vx;
      previousVelocities[basePos + 1] = vy;
      previousVelocities[basePos + 2] = vz;
    }
  }
  // インスタンシングメッシュの表示個体数を設定（捕食者を除く）
  const visibleCount = predatorCount > 0 ? Math.max(0, count - predatorCount) : count;
  instancedMeshHigh.count = visibleCount;
  instancedMeshLow.count = visibleCount;

  applyBufferSet(instancedMeshHigh, bufferSetHigh, currentIndex);
  applyBufferSet(instancedMeshLow, bufferSetLow, currentIndex);
  // トリプルバッファの更新フラグを立てる
  highPosAttr.needsUpdate = highTransformTouched;
  highQuatAttr.needsUpdate = highTransformTouched;
  lowPosAttr.needsUpdate = lowTransformTouched;
  lowQuatAttr.needsUpdate = lowTransformTouched;
  highTailParamsAttr.needsUpdate = highTailTouched;
  lowTailParamsAttr.needsUpdate = lowTailTouched;

  bufferCursor = nextIndex;

  // 頂点カラーの更新（デバッグ用のUnit色分け表示）
  if (showUnitColors.value) {
    const mappingPtrValue = boidUnitMappingPtr();
    const heapI32 = wasmModule.HEAP32.buffer;
    const unitMappings = new Int32Array(heapI32, mappingPtrValue, count * 2);

    for (let i = 0; i < visibleCount; i++) {
      let unitId = -1;
      for (let j = 0; j < unitMappings.length; j += 2) {
        if (unitMappings[j] === i) {
          unitId = unitMappings[j + 1];
          break;
        }
      }

      const color = unitId >= 0 ?
        new THREE.Color().setHSL((unitId % 100) / 100, 0.8, 0.6) :
        new THREE.Color(1, 0, 0);

      instancedMeshHigh.setColorAt(i, color);
      instancedMeshLow.setColorAt(i, color);
    }
    instancedMeshHigh.instanceColor.needsUpdate = true;
    instancedMeshLow.instanceColor.needsUpdate = true;
  } else if (lastShowUnitColors) {
    // Unit色分けOFF: ON→OFFになった時のみ頂点カラーを白にリセット
    const whiteColor = new THREE.Color(1, 1, 1);
    for (let i = 0; i < visibleCount; i++) {
      instancedMeshHigh.setColorAt(i, whiteColor);
      instancedMeshLow.setColorAt(i, whiteColor);
    }
    instancedMeshHigh.instanceColor.needsUpdate = true;
    instancedMeshLow.instanceColor.needsUpdate = true;
    console.log('✓ Reset vertex colors to white (OFF mode)');
  }

  // 前回の状態を保存
  lastShowUnitColors = showUnitColors.value;// マトリクスの更新

  controls.update();
  updateParticleUniforms();

  if (heightFogPass) {
    heightFogPass.uniforms.cameraNear.value = camera.near;
    heightFogPass.uniforms.cameraFar.value = camera.far;
    heightFogPass.uniforms.projectionMatrixInverse.value.copy(camera.projectionMatrixInverse);
    heightFogPass.uniforms.cameraMatrixWorld.value.copy(camera.matrixWorld);
    if (heightFogRenderTarget?.depthTexture) {
      heightFogPass.uniforms.tDepth.value = heightFogRenderTarget.depthTexture;
    }
  }

  if (isMobileDevice() || !composer) {
    renderer.render(scene, camera);
  } else {
    composer.render();
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
      wasmModule.setGlobalSpeciesParamsFromJS(createSpeciesParamsVector(), 1);
      try {
        const plainSettings = settings.map((s) => ({ ...toRaw(s) }));
        localStorage.setItem('boids_settings', JSON.stringify(plainSettings));
      } catch (error) {
        console.error('Failed to save settings to localStorage:', error);
      }
    }

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
    if (ensurePredatorMeshes(predators)) {
      predatorMeshCountCache = predators;
    }
    for (const mesh of predatorMeshes) mesh.visible = false;
  },
  { deep: true }
);

function resetSettings() {
  settings.length = 0;
  DEFAULT_SETTINGS.forEach(s => settings.push({ ...s }));
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