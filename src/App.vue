<template>
  <div id="app">
    <div class="ui-overlay">
      <div class="ui-panel">
        <h1>Boids Simulation</h1>
        <details>
          <summary>Settings</summary>
          <div v-for="(s, i) in settings" :key="i">
            <Settings :settings="s" />
          </div>
          <button @click="resetSettings" style="margin-bottom:1em;">リセット</button>
          <div class="follow-controls">
            <button @click="handleFollowButtonClick" :disabled="totalBoids === 0">
              {{ followButtonLabel }}
            </button>
            <label class="follow-orientation">
              <input type="checkbox" v-model="alignCameraWithBoid" />
              追従時に魚の向きにカメラを合わせる
            </label>
            <p v-if="followStatusText" class="follow-status">{{ followStatusText }}</p>
          </div>
          <details style="margin-bottom:1em;">
            <summary>デバッグ</summary>
            <div>壊れてるよ</div>
            <div style="margin-top:0.5em;">
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
        </details>
        <div class="info">
          <p>Boids Count: {{ totalBoids }}</p>
        </div>
      </div>
    </div>
    <div ref="threeContainer" class="three-container" @touchstart="handleTouchStart" @touchmove="handleTouchMove"
      @touchend="handleTouchEnd" />
  </div>
</template>

<script setup>
import { inject, onMounted, onBeforeUnmount, reactive, ref, watch, toRaw, computed } from 'vue';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import Settings from './components/Settings.vue';
import Stats from 'three/examples/jsm/libs/stats.module'
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { SSAOPass } from 'three/examples/jsm/postprocessing/SSAOPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";
import { color } from 'three/tsl';
import { vertexColor } from 'three/tsl';

const wasmModule = inject('wasmModule');
if (!wasmModule) {
  console.error('wasmModule not provided');
}

const posPtr = wasmModule.cwrap('posPtr', 'number', [])
const velPtr = wasmModule.cwrap('velPtr', 'number', [])
const oriPtr = wasmModule.cwrap('oriPtr', 'number', [])
const boidCount = wasmModule.cwrap('boidCount', 'number', [])
const build = wasmModule.cwrap('build', 'void', ['number', 'number'])
const update = wasmModule.cwrap('update', 'void', ['number'])
const setFlockSize = wasmModule.cwrap('setFlockSize', 'void', ['number', 'number', 'number'])
const exportTreeStructure = wasmModule.cwrap('exportTreeStructure', 'object', [])
const boidUnitMappingPtr = wasmModule.cwrap('boidUnitMappingPtr', 'number', []);
// const getUnitCount = wasmModule.cwrap('getUnitCount', 'number', []);
// const getUnitCentersPtr = wasmModule.cwrap('getUnitCentersPtr', 'number', []);
// const getUnitParentIndicesPtr = wasmModule.cwrap('getUnitParentIndicesPtr', 'number', []);

function fetchTreeStructure() {
  const treeData = exportTreeStructure();
  return treeData;
}

function getDefaultSettings() {
  const boidCount = isMobileDevice() ? 3000 : 10000;

  return [{
    species: 'Boids',         // 種族名
    count: boidCount,         // 群れの数（スマホなら3000、PCなら10000）
    cohesion: 30,             // 凝集
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
}

function ensureSettingsFields(settingsArray) {
  const defaults = getDefaultSettings();
  const defaultMap = new Map(defaults.map((def, index) => [def.species || index, def]));

  for (let i = 0; i < settingsArray.length; i++) {
    const current = settingsArray[i];
    const fallback = defaultMap.get(current.species) || defaults[0] || {};

    if (current.lambda === undefined) {
      current.lambda = fallback.lambda ?? 0.05;
    }

    if (current.tau === undefined) {
      current.tau = fallback.tau ?? 0.2;
    }
  }

  return settingsArray;
}

function loadSettings() {
  try {
    const saved = localStorage.getItem('boids_settings');
    if (saved) {
      const parsed = JSON.parse(saved);
      if (Array.isArray(parsed)) {
        return ensureSettingsFields(parsed); // 配列として保存されている場合のみ返す
      }
    }
  } catch (error) {
    console.error('Failed to load settings from localStorage:', error);
  }
  return ensureSettingsFields(getDefaultSettings()); // デフォルト値を返す
}

const settings = reactive(loadSettings());

// 全Boidsの合計を計算するcomputed
const totalBoids = computed(() => {
  return settings.reduce((total, setting) => total + (setting.count || 0), 0);
});

const followButtonLabel = computed(() => {
  if (isFollowing.value) return '追従を停止';
  if (isFollowSelectionMode.value) return '追従選択をキャンセル';
  return '追従する魚を選ぶ';
});

const followStatusText = computed(() => {
  if (isFollowSelectionMode.value) return '追従したい魚をクリックしてください';
  if (isFollowing.value) return `Boid #${followedBoidId.value + 1} を追従中`;
  return '';
});

const threeContainer = ref(null);
let scene, camera, renderer, controls, composer;

const paused = ref(false);

const showUnits = ref(true);
const showUnitSpheres = ref(false);
const showUnitLines = ref(false);
const showUnitColors = ref(false);
const unitLayer = ref(1);

const isFollowSelectionMode = ref(false);
const isFollowing = ref(false);
const followedBoidId = ref(-1);
const alignCameraWithBoid = ref(false);

watch(alignCameraWithBoid, () => {
  if (!isFollowing.value) return;
  const idx = followedBoidId.value;
  if (idx < 0) return;
  refreshFollowOffsets(idx);
});

const raycaster = new THREE.Raycaster();
const pointerNdc = new THREE.Vector2();
const followOffset = new THREE.Vector3();
const followOffsetLocal = new THREE.Vector3();
const followTargetPosition = new THREE.Vector3();
const desiredCameraPosition = new THREE.Vector3();
const tmpBoidPosition = new THREE.Vector3();
const tmpRayToPoint = new THREE.Vector3();
const tmpClosestPoint = new THREE.Vector3();
const tmpQuaternion = new THREE.Quaternion();
const tmpInverseQuaternion = new THREE.Quaternion();
const tmpDesiredOffset = new THREE.Vector3();
const tmpCurrentOffset = new THREE.Vector3();
const tmpYawQuaternion = new THREE.Quaternion();
const tmpForwardVector = new THREE.Vector3();
const Y_AXIS = new THREE.Vector3(0, 1, 0);
const hiddenInstanceMatrix = new THREE.Matrix4().makeScale(0, 0, 0);

let lastRendererCanvas = null;
const previousControlsState = {
  enablePan: true,
  enableZoom: true,
};
let isUserAdjustingCamera = false;

let unitSpheres = [];
let unitLines = [];

let maxDepth = 1;
let stats = null;

let animationTimer = null;
let FRAME_INTERVAL = 1000 / 60;//1000 / 60; // 60FPS

// パフォーマンス最適化用変数（スマホ用調整）
let lastColorUpdateFrame = 0;
let lastPredatorUpdateFrame = 0;
let speciesIndexLookup = []; // 各BoidのspeciesIndexをキャッシュ
let frameCounter = 0;
const COLOR_UPDATE_INTERVAL = isMobileDevice() ? 20 : 10; // スマホは20フレーム、PCは10フレーム
const PREDATOR_UPDATE_INTERVAL = isMobileDevice() ? 10 : 5; // スマホは10フレーム、PCは5フレーム

// レンダリング最適化用変数（スマホ用調整）
let lastLodUpdateFrame = 0;
const LOD_UPDATE_INTERVAL = isMobileDevice() ? 5 : 3; // スマホは5フレーム、PCは3フレーム
let boidLodStates = []; // 各BoidのLOD状態をキャッシュ

// メモリプール最適化
let matrixPool = [];
let colorPool = [];
let vec3Pool = [];

// オブジェクトプールからオブジェクトを取得
function getFromPool(pool, createFn) {
  return pool.length > 0 ? pool.pop() : createFn();
}

// オブジェクトプールにオブジェクトを返却
function returnToPool(pool, obj) {
  if (pool.length < 100) { // プールサイズを制限
    pool.push(obj);
  }
}

// マテリアル変数をグローバルスコープで定義
let boidMaterial = null;
let boidLodMaterial = null;

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

function stopCameraFollow() {
  if (!isFollowing.value) return;
  isFollowing.value = false;
  followedBoidId.value = -1;
  controls.enablePan = previousControlsState.enablePan;
  controls.enableZoom = previousControlsState.enableZoom;
  isUserAdjustingCamera = false;
}

function readBoidPosition(index, outVector) {
  const count = boidCount();
  console.log(count);
  if (index < 0 || index >= count) return false;
  const heapF32 = wasmModule.HEAPF32.buffer;
  const positionsArray = new Float32Array(heapF32, posPtr(), count * 3);
  const base = index * 3;
  outVector.set(positionsArray[base], positionsArray[base + 1], positionsArray[base + 2]);
  return true;
}

function readBoidOrientation(index, outQuaternion) {
  const count = boidCount();
  if (index < 0 || index >= count) return false;
  const heapF32 = wasmModule.HEAPF32.buffer;
  const orientationsArray = new Float32Array(heapF32, oriPtr(), count * 4);
  const base = index * 4;
  outQuaternion.set(
    orientationsArray[base],
    orientationsArray[base + 1],
    orientationsArray[base + 2],
    orientationsArray[base + 3]
  );
  return true;
}

function extractYawQuaternion(sourceQuaternion, outQuaternion) {
  tmpForwardVector.set(0, 0, 1).applyQuaternion(sourceQuaternion);
  tmpForwardVector.y = 0;
  if (tmpForwardVector.lengthSq() < 1e-6) {
    outQuaternion.identity();
    return outQuaternion;
  }
  tmpForwardVector.normalize();
  const yaw = Math.atan2(tmpForwardVector.x, tmpForwardVector.z);
  outQuaternion.setFromAxisAngle(Y_AXIS, yaw);
  return outQuaternion;
}

function refreshFollowOffsets(index) {
  if (!camera) return false;
  if (!readBoidPosition(index, followTargetPosition)) return false;
  followOffset.copy(camera.position).sub(followTargetPosition);
  if (readBoidOrientation(index, tmpQuaternion)) {
    extractYawQuaternion(tmpQuaternion, tmpYawQuaternion);
    tmpInverseQuaternion.copy(tmpYawQuaternion).invert();
    followOffsetLocal.copy(followOffset).applyQuaternion(tmpInverseQuaternion);
  } else {
    followOffsetLocal.copy(followOffset);
  }
  return true;
}

function handleControlsInteractionStart() {
  if (isFollowing.value) {
    isUserAdjustingCamera = true;
  }
}

function handleControlsInteractionEnd() {
  isUserAdjustingCamera = false;
  if (isFollowing.value && followedBoidId.value >= 0) {
    refreshFollowOffsets(followedBoidId.value);
  }
}

function startCameraFollow(index) {
  if (!readBoidPosition(index, followTargetPosition)) {
    return;
  }
  followedBoidId.value = index;
  isFollowSelectionMode.value = false;
  isFollowing.value = true;
  previousControlsState.enablePan = controls.enablePan;
  previousControlsState.enableZoom = controls.enableZoom;
  controls.enablePan = false;
  controls.target.copy(followTargetPosition);
  isUserAdjustingCamera = false;
  refreshFollowOffsets(index);
}

function handleFollowButtonClick() {
  if (isFollowing.value) {
    stopCameraFollow();
    return;
  }

  if (isFollowSelectionMode.value) {
    isFollowSelectionMode.value = false;
  } else {
    isFollowSelectionMode.value = true;
  }
}

function findClosestBoidIndex(ray) {
  const count = boidCount();
  if (count <= 0) return -1;

  const heapF32 = wasmModule.HEAPF32.buffer;
  const positionsArray = new Float32Array(heapF32, posPtr(), count * 3);

  let bestIndex = -1;
  let bestDistance = 0.6; // 許容半径

  for (let i = 0; i < count; i++) {
    const base = i * 3;
    tmpBoidPosition.set(positionsArray[base], positionsArray[base + 1], positionsArray[base + 2]);

    tmpRayToPoint.subVectors(tmpBoidPosition, ray.origin);
    const t = tmpRayToPoint.dot(ray.direction);
    if (t < 0) continue; // カメラの後方

    tmpClosestPoint.copy(ray.direction).multiplyScalar(t).add(ray.origin);
    const distance = tmpBoidPosition.distanceTo(tmpClosestPoint);

    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = i;
    }
  }

  return bestIndex;
}

function handleCanvasClick(event) {
  if (!isFollowSelectionMode.value || !renderer) return;

  const rect = renderer.domElement.getBoundingClientRect();
  pointerNdc.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointerNdc.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointerNdc, camera);

  const boidIndex = findClosestBoidIndex(raycaster.ray);
  if (boidIndex >= 0) {
    startCameraFollow(boidIndex);
  } else {
    isFollowSelectionMode.value = false;
  }
}

function computeYawQuaternionFromArray(orientationsArray, index, outQuaternion) {
  if (!orientationsArray) return null;
  tmpQuaternion.fromArray(orientationsArray, index * 4);
  return extractYawQuaternion(tmpQuaternion, outQuaternion);
}

function updateCameraFollow(positionsArray, orientationsArray) {
  if (!isFollowing.value) return;
  const idx = followedBoidId.value;
  const count = boidCount();
  if (idx < 0 || idx >= count) {
    stopCameraFollow();
    return;
  }

  followTargetPosition.fromArray(positionsArray, idx * 3);
  tmpCurrentOffset.copy(camera.position).sub(followTargetPosition);

  if (alignCameraWithBoid.value && orientationsArray) {
    const yawQuat = computeYawQuaternionFromArray(orientationsArray, idx, tmpYawQuaternion);
    if (yawQuat) {
      tmpDesiredOffset.copy(followOffsetLocal).applyQuaternion(yawQuat);

      if (isUserAdjustingCamera) {
        tmpInverseQuaternion.copy(yawQuat).invert();
        followOffsetLocal.copy(tmpCurrentOffset).applyQuaternion(tmpInverseQuaternion);
        tmpDesiredOffset.copy(followOffsetLocal).applyQuaternion(yawQuat);
      }

      desiredCameraPosition.copy(followTargetPosition).add(tmpDesiredOffset);
      followOffset.copy(tmpDesiredOffset);
    } else {
      if (isUserAdjustingCamera) {
        followOffset.copy(tmpCurrentOffset);
      }
      desiredCameraPosition.copy(followTargetPosition).add(followOffset);
      followOffsetLocal.copy(followOffset);
    }
  } else {
    if (isUserAdjustingCamera) {
      followOffset.copy(tmpCurrentOffset);
    }
    desiredCameraPosition.copy(followTargetPosition).add(followOffset);

    if (orientationsArray) {
      const yawQuat = computeYawQuaternionFromArray(orientationsArray, idx, tmpYawQuaternion);
      if (yawQuat) {
        tmpInverseQuaternion.copy(yawQuat).invert();
        followOffsetLocal.copy(followOffset).applyQuaternion(tmpInverseQuaternion);
      } else {
        followOffsetLocal.copy(followOffset);
      }
    } else {
      followOffsetLocal.copy(followOffset);
    }
  }

  camera.position.lerp(desiredCameraPosition, 0.2);
  controls.target.lerp(followTargetPosition, 0.2);
}

function initThreeJS() {
  const width = window.innerWidth;
  const height = window.innerHeight; scene = new THREE.Scene();
  // フォグ（背景と同じ色で境界をなじませる）
  scene.fog = new THREE.Fog(toHex(OCEAN_COLORS.FOG), 3, 14);

  camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
  camera.position.set(2.19, -6.30, 5.76);
  camera.lookAt(0, 0, 0);
  camera.position.set(2.19, -5.80, 5.76);

  renderer = new THREE.WebGLRenderer({
    antialias: !isMobileDevice(), // スマホではアンチエイリアスを無効化
    depth: true, // 深度バッファを有効化
    powerPreference: isMobileDevice() ? "low-power" : "high-performance"
  });
  // スマホ用ピクセル比調整（パフォーマンス向上）
  renderer.setPixelRatio(isMobileDevice() ? Math.min(window.devicePixelRatio, 2) : window.devicePixelRatio);
  renderer.setSize(width, height);
  renderer.shadowMap.enabled = !isMobileDevice(); // スマホでは影を無効化
  if (!isMobileDevice()) {
    renderer.shadowMap.type = THREE.PCFSoftShadowMap; // 影を柔らかく（PCのみ）
  }
  renderer.outputColorSpace = THREE.SRGBColorSpace; // 色空間を統一

  threeContainer.value.appendChild(renderer.domElement);

  // WebGLコンテキストロス対策
  setupWebGLContextLossHandling();

  camera.aspect = width / height;
  camera.updateProjectionMatrix();

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.1;
  controls.addEventListener('start', handleControlsInteractionStart);
  controls.addEventListener('end', handleControlsInteractionEnd);

  lastRendererCanvas = renderer.domElement;
  lastRendererCanvas.addEventListener('click', handleCanvasClick);
  const groundGeo = new THREE.PlaneGeometry(100, 100);
  groundMaterial = createFadeOutGroundMaterial(); // グローバル変数に保存
  const ground = new THREE.Mesh(groundGeo, groundMaterial);
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -7;
  ground.receiveShadow = true; // 影を受ける
  scene.add(ground);

  // 大きなsphereで海中環境を作成
  createOceanSphere();

  // 海中の環境光（定数使用）
  const ambientLight = new THREE.AmbientLight(toHex(OCEAN_COLORS.AMBIENT_LIGHT), 1.3);
  scene.add(ambientLight);

  // メインの太陽光（定数使用、スマホでは影を無効化）
  const dirLight = new THREE.DirectionalLight(toHex(OCEAN_COLORS.SUN_LIGHT), 23.0);
  dirLight.position.set(0, 60, 30);
  dirLight.castShadow = !isMobileDevice(); // スマホでは影を無効化

  // 下からの反射光（定数使用）
  const bottomLight = new THREE.DirectionalLight(toHex(OCEAN_COLORS.BOTTOM_LIGHT), 0);
  bottomLight.position.set(0, -30, 0);
  scene.add(bottomLight);

  // 影カメラの範囲を広げる（PCのみ）
  if (!isMobileDevice()) {
    dirLight.shadow.camera.left = -100;
    dirLight.shadow.camera.right = 100;
    dirLight.shadow.camera.top = 100;
    dirLight.shadow.camera.bottom = -100;
    dirLight.shadow.camera.near = 1;
    dirLight.shadow.camera.far = 1000;

    // スマホでは影の解像度を下げる
    dirLight.shadow.mapSize.width = isMobileDevice() ? 512 : 2048;
    dirLight.shadow.mapSize.height = isMobileDevice() ? 512 : 2048;
    dirLight.shadow.bias = -0.001;
    dirLight.shadow.normalBias = 0.01;
  }
  scene.add(dirLight);
  // EffectComposer の初期化（スマホ以外の場合のみ）
  if (!isMobileDevice()) {
    // EffectComposer の初期化
    composer = new EffectComposer(renderer);

    // RenderPass を追加
    const renderPass = new RenderPass(scene, camera);
    composer.addPass(renderPass);

    const ssaoPass = new SSAOPass(scene, camera, width, height);
    ssaoPass.kernelRadius = 8; // サンプル半径（大きくすると効果が広がる）
    ssaoPass.minDistance = 0.001; // 最小距離（小さくすると近距離の効果が強調される）
    ssaoPass.maxDistance = 0.01; // 最大距離（大きくすると遠距離の効果が強調される）
    composer.addPass(ssaoPass);

    // UnrealBloomPass を追加（任意）
    const bloomPass = new UnrealBloomPass(new THREE.Vector2(width, height), 1.5, 0.4, 0.85);
    composer.addPass(bloomPass);
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
}

// 一時的に従来方式に戻す - instancedMeshを単一で使用
let instancedMeshHigh = null; // 高ポリゴン用
let instancedMeshLow = null;  // 低ポリゴン用
let groundMaterial = null;    // 地面マテリアルの参照

// LOD用ジオメトリ・マテリアルを使い回す
const boidGeometryHigh = new THREE.SphereGeometry(1, 8, 8);
const boidGeometryLow = new THREE.SphereGeometry(1, 3, 2);
boidGeometryHigh.scale(0.5, 0.5, 2.0); // 少し小さくする
boidGeometryLow.scale(0.5, 0.5, 2.0); // 少し小さくする
let boidModel = null; // 読み込んだモデルを保持
let boidModelLod = null; // 読み込んだモデルを保持
let predatorModel = null; // 捕食者モデルを保持
let originalMaterial = null; // 元のマテリアルを保持
let originalMaterialLod = null; // 元のLODマテリアルを保持
let predatorMaterial = null; // 捕食者用マテリアル

// 起動時の正しいテクスチャマテリアルを保持
let originalHighMat = null;
let originalLowMat = null;

// 前回のshowUnitColorsの状態を保持（OFF→ONの検知用）
let lastShowUnitColors = false;

function initInstancedBoids(count) {
  if (!boidModel.children || !boidModel.children[0]) {
    console.error('Boid model does not have valid children.');
    return;
  }

  // 既存のメッシュを削除
  if (instancedMeshHigh) scene.remove(instancedMeshHigh);
  if (instancedMeshLow) scene.remove(instancedMeshLow);
  // InstancedMeshを作成（最初はvertexColors無効でテクスチャ表示）
  const highMaterial = originalMaterial.clone();
  highMaterial.vertexColors = false; // 最初はテクスチャ表示

  const lowMaterial = originalMaterialLod.clone();
  lowMaterial.vertexColors = false; // 最初はテクスチャ表示

  instancedMeshHigh = new THREE.InstancedMesh(
    boidModel.children[0].geometry,
    highMaterial,
    count
  );
  instancedMeshHigh.castShadow = !isMobileDevice(); // スマホでは影を無効化
  instancedMeshHigh.receiveShadow = !isMobileDevice();

  instancedMeshLow = new THREE.InstancedMesh(
    boidModelLod.children[0].geometry,
    lowMaterial,
    count
  );
  instancedMeshLow.castShadow = !isMobileDevice(); // スマホでは影を無効化
  instancedMeshLow.receiveShadow = !isMobileDevice();

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

  console.log('InstancedMeshes created with vertex colors enabled');
}



// 初期化時のコールバック（頻度が低いため匿名関数でも問題なし）
function loadBoidModel(callback) {
  const loader = new GLTFLoader();
  const textureLoader = new THREE.TextureLoader();

  const texture = textureLoader.load(
    './models/fish.png',
    () => console.log('Texture loaded successfully.'),
    undefined,
    (error) => console.error('An error occurred while loading the texture:', error)
  );

  const textureLod = textureLoader.load(
    './models/fish_lod.png',
    () => console.log('Texture loaded successfully.'),
    undefined,
    (error) => console.error('An error occurred while loading the texture:', error)
  );

  const normalMapLod = textureLoader.load(
    './models/fish_lod_n.png',
    () => console.log('LOD Normal map loaded successfully.'),
    undefined,
    (error) => console.error('An error occurred while loading the LOD normal map:', error)
  );

  const predatorTexture = textureLoader.load(
    './models/fishPredetor.png',
    () => console.log('Predator texture loaded successfully.'),
    undefined,
    (error) => console.error('An error occurred while loading the predator texture:', error)
  );

  texture.flipY = false;
  texture.colorSpace = THREE.SRGBColorSpace;
  textureLod.flipY = false;
  textureLod.colorSpace = THREE.SRGBColorSpace;
  normalMapLod.flipY = false;
  normalMapLod.colorSpace = THREE.LinearSRGBColorSpace; // ノーマルマップはLinear色空間
  predatorTexture.flipY = false;
  predatorTexture.colorSpace = THREE.SRGBColorSpace;

  boidMaterial = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    roughness: 0.5,
    metalness: 0,
    transparent: false,
    alphaTest: 0.5,
    map: texture,
    vertexColors: true,
    vertexColor: 0xffffff
  });

  boidLodMaterial = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    roughness: 0.5,
    metalness: 0,
    transparent: false,
    alphaTest: 0.5,
    map: textureLod,
    normalMap: normalMapLod,
    vertexColors: false,
    vertexColor: 0xffffff
  });

  predatorMaterial = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    roughness: 0.5,
    metalness: 0,
    transparent: false,
    alphaTest: 0.5,
    map: predatorTexture,
    vertexColors: false,
    vertexColor: 0xffffff
  });

  originalMaterial = boidMaterial;
  originalMaterialLod = boidLodMaterial;

  let modelsLoaded = 0;
  const totalModels = 3;

  const checkAllLoaded = () => {
    modelsLoaded++;
    if (modelsLoaded === totalModels) {
      callback();
    }
  };

  loader.load(
    `./models/boidModel.glb`,
    (gltf) => {
      boidModel = gltf.scene;
      boidModel.traverse((child) => {
        if (child.isMesh) {
          child.material = boidMaterial;
        }
      });
      checkAllLoaded();
    },
    undefined,
    (error) => console.error('An error occurred while loading the model:', error)
  );

  loader.load(
    `./models/boidModel_lod.glb`,
    (gltf) => {
      boidModelLod = gltf.scene;
      boidModelLod.traverse((child) => {
        if (child.isMesh) {
          child.material = boidLodMaterial;
        }
      });
      checkAllLoaded();
    },
    undefined,
    (error) => console.error('An error occurred while loading the LOD model:', error)
  );

  loader.load(
    `./models/boidPredetorModel.glb`,
    (gltf) => {
      predatorModel = gltf.scene;
      predatorModel.traverse((child) => {
        if (child.isMesh) {
          child.material = predatorMaterial;
        }
      });
      checkAllLoaded();
    },
    undefined,
    (error) => console.error('An error occurred while loading the predator model:', error)
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
let positions, velocities, orientations;

let predatorMarkers = []; // Predator 用のマーカーを保持（複数対応）
let lastTime = performance.now(); // 前回のフレームのタイムスタンプ

// アニメーション継続用の命名関数（匿名関数を避けてパフォーマンス改善）
function scheduleNextFrame() {
  animationTimer = setTimeout(animate, FRAME_INTERVAL);
}

// 種族インデックスの事前計算（設定変更時のみ実行）
function buildSpeciesIndexLookup() {
  speciesIndexLookup = [];
  let currentIndex = 0;

  for (let s = 0; s < settings.length; s++) {
    for (let i = 0; i < settings[s].count; i++) {
      speciesIndexLookup[currentIndex] = s;
      currentIndex++;
    }
  }
}

function animate() {
  stats?.begin();
  frameCounter++;

  const currentTime = performance.now();
  const deltaTime = (currentTime - lastTime) / 1000;
  lastTime = currentTime;
  if (!paused.value) update(deltaTime);
  const count = boidCount();
    console.log(count);
  const heapF32 = wasmModule.HEAPF32.buffer;
  const positions = new Float32Array(heapF32, posPtr(), count * 3);
  const orientations = new Float32Array(heapF32, oriPtr(), count * 4);
  const dummy = new THREE.Object3D();
  const camPos = camera.position;

  updateCameraFollow(positions, orientations);

  // 使用するメッシュを決定（早期宣言）
  let activeMeshHigh = instancedMeshHigh;
  let activeMeshLow = instancedMeshLow;

  // 捕食者マーカーの最適化：5フレームに1回のみ更新
  if (frameCounter - lastPredatorUpdateFrame >= PREDATOR_UPDATE_INTERVAL) {
    const predatorCount = settings.filter(s => s.isPredator).reduce((total, s) => total + s.count, 0);

    // 必要な捕食者マーカー数を確保
    while (predatorMarkers.length < predatorCount && predatorModel) {
      const newPredatorMarker = predatorModel.clone();
      newPredatorMarker.traverse((child) => {
        if (child.isMesh) {
          child.material = predatorMaterial;
        }
      });
      scene.add(newPredatorMarker);
      predatorMarkers.push(newPredatorMarker);
    }

    // 余分なマーカーを削除
    while (predatorMarkers.length > predatorCount) {
      const marker = predatorMarkers.pop();
      scene.remove(marker);
    }

    lastPredatorUpdateFrame = frameCounter;
  }  // 各Boidの位置と色を更新（最適化版）
  let predatorIndex = 0; // 捕食者マーカーのインデックス

  // 種族インデックスルックアップが未構築なら構築
  if (speciesIndexLookup.length !== count) {
    buildSpeciesIndexLookup();
  }

  // LOD状態の初期化
  if (boidLodStates.length !== count) {
    boidLodStates = new Array(count).fill(false);
  }

  // LOD判定の最適化：3フレームに1回のみ更新
  const shouldUpdateLod = (frameCounter - lastLodUpdateFrame >= LOD_UPDATE_INTERVAL);
  if (shouldUpdateLod) {
    lastLodUpdateFrame = frameCounter;
  }

  for (let i = 0; i < count; ++i) {
    dummy.position.fromArray(positions, i * 3);
    dummy.quaternion.fromArray(orientations, i * 4);
    dummy.updateMatrix();

    // LOD判定の最適化
    let useHigh;
    if (shouldUpdateLod) {
      const distSq = camPos.distanceToSquared(dummy.position);
      useHigh = distSq < 4;
      boidLodStates[i] = useHigh;
    } else {
      useHigh = boidLodStates[i];
    }

    // マトリクスを設定
    if (useHigh) {
      activeMeshHigh.setMatrixAt(i, dummy.matrix);
      activeMeshLow.setMatrixAt(i, hiddenInstanceMatrix);
    } else {
      activeMeshHigh.setMatrixAt(i, hiddenInstanceMatrix);
      activeMeshLow.setMatrixAt(i, dummy.matrix);
    }

    // 捕食者の特別表示（最適化：事前計算されたルックアップを使用）
    const speciesIndex = speciesIndexLookup[i];

    // 捕食者の場合、対応するマーカーを更新
    if (speciesIndex >= 0 && settings[speciesIndex].isPredator && predatorIndex < predatorMarkers.length) {
      const marker = predatorMarkers[predatorIndex];
      marker.position.copy(dummy.position);
      marker.quaternion.copy(dummy.quaternion);
      predatorIndex++;
    }
  }// 頂点カラーの更新（最適化：10フレームに1回のみ）
  if (showUnitColors.value && (frameCounter - lastColorUpdateFrame >= COLOR_UPDATE_INTERVAL)) {
    const mappingPtrValue = boidUnitMappingPtr();
    const heapI32 = wasmModule.HEAP32.buffer;
    const unitMappings = new Int32Array(heapI32, mappingPtrValue, count * 2);

    for (let i = 0; i < count; i++) {
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
    lastColorUpdateFrame = frameCounter;
  } else if (lastShowUnitColors && !showUnitColors.value) {
    // Unit色分けOFF: ON→OFFになった時のみ頂点カラーを白にリセット
    const whiteColor = new THREE.Color(1, 1, 1);
    for (let i = 0; i < count; i++) {
      instancedMeshHigh.setColorAt(i, whiteColor);
      instancedMeshLow.setColorAt(i, whiteColor);
    }
    instancedMeshHigh.instanceColor.needsUpdate = true;
    instancedMeshLow.instanceColor.needsUpdate = true;
    console.log('✓ Reset vertex colors to white (OFF mode)');
  }  // 前回の状態を保存
  lastShowUnitColors = showUnitColors.value;

  // マトリクスの更新
  activeMeshHigh.instanceMatrix.needsUpdate = true;
  activeMeshLow.instanceMatrix.needsUpdate = true;

  controls.update();

  (isMobileDevice() ? renderer : composer).render(scene, camera);
  stats?.end();

  scheduleNextFrame();
}

// ツリー描画用の命名関数（匿名関数を避けてパフォーマンス改善）
function drawTreeNode(node, parentPosition = null) {
  const position = new THREE.Vector3(
    node.center[0],
    node.center[1],
    node.center[2]
  );

  if (parentPosition) {
    const geometry = new THREE.BufferGeometry().setFromPoints([
      parentPosition,
      position,
    ]);
    const material = new THREE.LineBasicMaterial({ color: 0xffffff });
    const line = new THREE.Line(geometry, material);
    scene.add(line);
  }

  if (node.children) {
    for (let i = 0; i < node.children.length; i++) {
      drawTreeNode(node.children[i], position);
    }
  }
}

function drawTreeStructure(treeData) {
  for (let i = 0; i < treeData.length; i++) {
    drawTreeNode(treeData[i]);
  }
}
// 設定からWASMパラメータを作成する命名関数（匿名関数を避けてパフォーマンス改善）
function createSpeciesParamsVector(settingsArray) {
  const vector = new wasmModule.VectorSpeciesParams();
  for (let i = 0; i < settingsArray.length; i++) {
    const s = settingsArray[i];
    vector.push_back({
      species: s.species || "default",
      count: s.count || 0,
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
      horizontalTorque: s.horizontalTorque || 0.0,
      torqueStrength: s.torqueStrength || 0.0,
      lambda: s.lambda ?? 0.05,
      tau: s.tau ?? 0.2,
      isPredator: s.isPredator || false,
    });
  }
  return vector;
}

// 設定数を計算する命名関数
function calculateTotalBoidCount(settingsArray) {
  let sum = 0;
  for (let i = 0; i < settingsArray.length; i++) {
    sum += settingsArray[i].count;
  }
  return sum;
}

function startSimulation() {
  // WebAssembly モジュール用に SpeciesParams を初期化
  const vector = createSpeciesParamsVector(settings);
  // callInitBoids に渡す（この vector は C++ 側で vector<SpeciesParams> になる）
  wasmModule.callInitBoids(vector, 1, 3, 0.25);
  build(16, 0);
  initInstancedBoids(calculateTotalBoidCount(settings));
  animate();
}

// 初期化時のコールバック（頻度が低いため匿名関数でも問題なし）
onMounted(() => {
  initThreeJS();
  loadBoidModel(() => {
    console.log('Boid model loaded successfully.');
    // stats.jsの初期化とDOM追加
    stats = new Stats();
    stats.showPanel(0);
    document.body.appendChild(stats.dom);
    // 右上に移動させる
    stats.dom.style.position = 'fixed';
    stats.dom.style.right = '0px';
    stats.dom.style.top = '0px';
    stats.dom.style.left = 'auto';
    stats.dom.style.zIndex = 1000;

    startSimulation();
    
    // メモリ監視を開始
    startMemoryMonitoring();
  });
  window.addEventListener('keydown', handleKeydown);
});

onBeforeUnmount(() => {
  stopCameraFollow();
  window.removeEventListener('keydown', handleKeydown);
  window.removeEventListener('resize', onWindowResize);
  if (lastRendererCanvas) {
    lastRendererCanvas.removeEventListener('click', handleCanvasClick);
    lastRendererCanvas = null;
  }
  controls?.removeEventListener('start', handleControlsInteractionStart);
  controls?.removeEventListener('end', handleControlsInteractionEnd);
  if (animationTimer) {
    clearTimeout(animationTimer);
    animationTimer = null;
  }
  if (stats?.dom?.parentNode) {
    stats.dom.parentNode.removeChild(stats.dom);
  }
  stats = null;
  stopMemoryMonitoring();
});

function isMobileDevice() {
  return /Mobi|Android/i.test(navigator.userAgent);
}

// スマホ向けのより詳細なデバイス性能チェック
function getDevicePerformanceLevel() {
  const userAgent = navigator.userAgent;
  const hardwareConcurrency = navigator.hardwareConcurrency || 4;
  const deviceMemory = navigator.deviceMemory || 4; // GB
  
  // 低性能デバイスの検出
  const isLowEnd = deviceMemory <= 2 || hardwareConcurrency <= 2;
  const isTablet = /iPad|Android.*(?!.*Mobile)/i.test(userAgent);
  
  return {
    isLowEnd,
    isTablet,
    cores: hardwareConcurrency,
    memory: deviceMemory,
    suggestedBoidCount: isLowEnd ? 1500 : (isTablet ? 5000 : 3000)
  };
}

// スマホタップでの停止/再開機能（ドラッグとタップを区別）
let touchStartTime = 0;
let touchStartPos = { x: 0, y: 0 };
let hasMoved = false;

function handleTouchStart(event) {
  if (isMobileDevice()) {
    touchStartTime = Date.now();
    const touch = event.touches[0];
    touchStartPos = { x: touch.clientX, y: touch.clientY };
    hasMoved = false;
  }
}

function handleTouchMove(event) {
  if (isMobileDevice() && touchStartTime > 0) {
    const touch = event.touches[0];
    const moveDistance = Math.sqrt(
      Math.pow(touch.clientX - touchStartPos.x, 2) +
      Math.pow(touch.clientY - touchStartPos.y, 2)
    );

    // 5px以上動いたらドラッグと判定
    if (moveDistance > 5) {
      hasMoved = true;
    }
  }
}

function handleTouchEnd(event) {
  if (isMobileDevice() && touchStartTime > 0) {
    const touchDuration = Date.now() - touchStartTime;

    // 短時間（500ms以下）で、動いていない場合のみタップと判定
    if (touchDuration < 500 && !hasMoved) {
      const isThreeContainer = event.target === threeContainer.value;
      const isCanvas = event.target.tagName === 'CANVAS';
      const isInThreeContainer = threeContainer.value && threeContainer.value.contains(event.target);

      if (isThreeContainer || isCanvas || isInThreeContainer) {
        paused.value = !paused.value;
      }
    }

    // リセット
    touchStartTime = 0;
    hasMoved = false;
  }
}
// 設定監視用のハンドラ関数（匿名関数を避けてパフォーマンス改善）
function handleSettingsChange(val) {
  if (wasmModule && wasmModule.setGlobalSpeciesParamsFromJS) {
    const vector = createSpeciesParamsVector(settings);
    wasmModule.setGlobalSpeciesParamsFromJS(vector, 1);
    try {
      localStorage.setItem('boids_settings', JSON.stringify(toRaw(settings)));
    } catch (error) {
      console.error('Failed to save settings to localStorage:', error);
    }
  }
}

// settingsの変更をwasmModuleに反映
watch(
  settings,
  handleSettingsChange,
  { deep: true } // 深い変更も監視
);

// flockSize変更のハンドラ関数（匿名関数を避けてパフォーマンス改善）
function handleFlockSizeChange(newSize) {
  if (setFlockSize) {
    // flockSize変更時
    setFlockSize(newSize, 40, 0.25);
    // Three.js 側の初期化
    initInstancedBoids(newSize);
  }
}

// flockSizeの変更を監視
watch(
  () => settings.flockSize,
  handleFlockSizeChange
);

// 設定リセット用の命名関数（forEach を for ループに置き換え）
function resetSettings() {
  const defaultSettings = getDefaultSettings();
  settings.length = 0;
  for (let i = 0; i < defaultSettings.length; i++) {
    settings.push({ ...defaultSettings[i] });
  }
  // 設定変更時に種族インデックスルックアップを再構築
  speciesIndexLookup.length = 0;
}

// Unit可視化変更のハンドラ関数（匿名関数を避けてパフォーマンス改善）
function handleUnitsVisibilityChange(newValue) {
  console.log('showUnits changed to:', newValue);
  if (!newValue) {
    // Unit可視化をオフにした場合、既存の可視化要素をクリア
    clearUnitVisuals();
  }
}

// Unit表示モード変更のハンドラ関数（匿名関数を避けてパフォーマンス改善）
function handleUnitDisplayModeChange([newSpheres, newLines]) {
  console.log('Unit display mode changed - Spheres:', newSpheres, 'Lines:', newLines);
  // 表示モードが変更されたら既存の表示をクリアして再描画
  if (showUnits.value) {
    clearUnitVisuals();
  }
}

// Unit可視化の変更を監視
watch(showUnits, handleUnitsVisibilityChange);

// Unit表示モードの変更を監視
watch([showUnitSpheres, showUnitLines], handleUnitDisplayModeChange);

// 海中色彩テーマ定数（調整しやすくするため統一管理）
// VS Codeのカラーピッカーで色を確認・調整できます
const OCEAN_COLORS = {
  // 背景とフォグ系
  SKY_HIGHLIGHT: '#4fbaff',      // 明るい海中ブルー（上層）
  SKY_BLUE: '#15a1ff',      // 明るい海中ブルー（上層）
  DEEP_BLUE: '#002968',     // 深い海中ブルー（中層）- より暗く調整
  SEAFLOOR: '#777465',      // 海底色（下層）
  FOG: '#153a6c',           // フォグ色（中層）- より暗く調整


  // ライティング系
  AMBIENT_LIGHT: '#2c9aff', // 環境光
  SUN_LIGHT: '#5389b7',     // 太陽光
  SIDE_LIGHT1: '#6ba3d0',   // 補助光1
  SIDE_LIGHT2: '#2d5f7a',   // 補助光2
  BOTTOM_LIGHT: '#0f2635',  // 底部反射光
};

// 色を16進数に変換する関数
const toHex = (colorStr) => parseInt(colorStr.replace('#', '0x'), 16);
// 単一sphereにグラデーションを適用した海中環境を作成
function createOceanSphere() {
  // グラデーションテクスチャを作成
  const canvas = document.createElement('canvas');
  const context = canvas.getContext('2d');
  canvas.width = 512;
  canvas.height = 512;

  // 縦方向のグラデーション（上から下へ）
  const gradient = context.createLinearGradient(0, 0, 0, canvas.height);
  gradient.addColorStop(0, OCEAN_COLORS.SKY_HIGHLIGHT);    // 上部：明るい海中ブルー
  gradient.addColorStop(0.2, OCEAN_COLORS.SKY_BLUE);   // 上部：明るい海中ブルー
  gradient.addColorStop(0.6, OCEAN_COLORS.DEEP_BLUE); // 中央：深い海中ブルー
  gradient.addColorStop(1, OCEAN_COLORS.DEEP_BLUE);                // 底部：最も暗い

  context.fillStyle = gradient;
  context.fillRect(0, 0, canvas.width, canvas.height);
  // テクスチャとして作成
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.generateMipmaps = false; // 色の精度を保つため
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;

  const sphereGeo = new THREE.SphereGeometry(300, 32, 32);
  // フォグの影響を受けるマテリアル（補助的な役割）
  const sphereMat = new THREE.MeshBasicMaterial({
    map: texture,
    side: THREE.BackSide, // 内側から見えるように
    fog: false // フォグの影響を受ける
  });

  const oceanSphere = new THREE.Mesh(sphereGeo, sphereMat);
  oceanSphere.position.set(0, 0, 0);
  scene.add(oceanSphere);

  return oceanSphere;
}

// 端がフェードアウトする地面マテリアルを作成
function createFadeOutGroundMaterial() {
  // 外部のアルファテクスチャを使用
  const textureLoader = new THREE.TextureLoader();
  const alphaMap = textureLoader.load('./models/groundAlfa.png');

  // テクスチャ設定
  alphaMap.minFilter = THREE.LinearFilter;
  alphaMap.magFilter = THREE.LinearFilter;
  alphaMap.wrapS = THREE.ClampToEdgeWrapping;
  alphaMap.wrapT = THREE.ClampToEdgeWrapping;

  const material = new THREE.MeshStandardMaterial({
    color: toHex(OCEAN_COLORS.SEAFLOOR),
    transparent: true,
    alphaMap: alphaMap,
    depthWrite: false // デプスバッファへの書き込み可否
  });

  return material;
}

// WebGLコンテキストロス対策（軽量版）
function setupWebGLContextLossHandling() {
  if (!renderer) return;

  const canvas = renderer.domElement;
  
  // コンテキストロス時の処理
  canvas.addEventListener('webglcontextlost', (event) => {
    console.warn('WebGLコンテキストが失われました - ページをリロードします');
    event.preventDefault();
    
    // シンプルに2秒後にリロード
    setTimeout(() => {
      location.reload();
    }, 2000);
  }, false);
}



// 軽量メモリ監視（モバイル向け）
function monitorWebGLMemory() {
  if (!renderer || !renderer.info || !isMobileDevice()) return;
  
  const info = renderer.info;
  const memoryInfo = info.memory || {};
  const renderInfo = info.render || {};
  
  const geometries = memoryInfo.geometries || 0;
  const textures = memoryInfo.textures || 0;
  
  // メモリ使用量が非常に高い場合のみ警告
  if (geometries > 100 || textures > 50) {
    console.warn('WebGLメモリ使用量が高いため、ページをリロードします');
    location.reload();
  }
}

// メモリ監視を軽量化
let memoryMonitorInterval;
function startMemoryMonitoring() {
  if (isMobileDevice()) {
    memoryMonitorInterval = setInterval(monitorWebGLMemory, 10000); // 10秒ごと
  }
}

function stopMemoryMonitoring() {
  if (memoryMonitorInterval) {
    clearInterval(memoryMonitorInterval);
    memoryMonitorInterval = null;
  }
}
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

.follow-controls {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.follow-controls button {
  align-self: flex-start;
}

.follow-orientation {
  display: flex;
  align-items: center;
  gap: 0.4rem;
  font-size: 0.9rem;
}

.follow-status {
  font-size: 0.9rem;
  color: #ffd28f;
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
  touch-action: manipulation;
  user-select: none;
  -webkit-touch-callout: none;
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

.ui-panel {
  pointer-events: auto;
  display: inline-flex;
  flex-direction: column;
  gap: 1rem;
  max-width: min(90vw, 420px);
  user-select: none;
}

.ui-panel input,
.ui-panel button,
.ui-panel select,
.ui-panel textarea {
  user-select: text;
}

.ui-overlay * {
  pointer-events: auto;
}
</style>