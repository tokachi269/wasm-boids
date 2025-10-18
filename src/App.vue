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
                <input type="checkbox" v-model="enableTailAnimation" />
                尾のくねり
              </label>
              <label style="margin-left:1em;">
                <input type="checkbox" v-model="enableTailOnLod" />
                LODもくねり
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

const useInstancedRendering = ref(true);
const enableTailAnimation = ref(true);
const enableTailOnLod = ref(true);

// WebGL復旧機能の状態管理
let rendererRetryTimer = null;
let rendererInitAttempt = 0;
// 段階的フォールバック設定（モバイル・コンテキストロス対応）
const rendererConfigs = [
  (isMobile) => ({
    antialias: !isMobile,
    depth: true,
    alpha: false,
    powerPreference: isMobile ? 'low-power' : 'high-performance'
  }),
  () => ({
    antialias: false,
    depth: true,
    stencil: false,
    alpha: false,
    powerPreference: 'low-power'
  }),
  () => ({
    antialias: false,
    depth: false,
    stencil: false,
    alpha: false,
    powerPreference: 'low-power'
  })
];

// シミュレーション初期化の制御フラグ
let simulationPending = false;
let simulationInitialized = false;
let pendingSimulationReset = true;

// 尾のアニメーション管理オブジェクト
const tailAnimation = {
  baseHighMaterial: null,        // 元の高品質マテリアル
  baseLowMaterial: null,         // 元のLODマテリアル
  animatedHighMaterial: null,    // 尾アニメ付き高品質マテリアル
  animatedLowMaterial: null,     // 尾アニメ付きLODマテリアル
  applyToLod: true,              // LODメッシュにも適用するかのフラグ
  phaseAttribute: null,          // 各インスタンスの位相オフセット
  speedAttribute: null,          // 各インスタンスの速度
  turnAttribute: null,           // 各インスタンスの旋回量
  previousVelocities: null,      // 前フレームの速度（旋回計算用）
  smoothedSpeed: null,           // 速度のスムージング用バッファ
  smoothedTurn: null,            // 旋回量のスムージング用バッファ
  elapsedTime: 0,                // 尾アニメーション用の累積時間（停止時も保持）
  uniforms: {
    uTailTime: { value: 0 },            // 時間（波形生成用）
    uTailAmplitude: { value: 0.08 },    // 振幅（全身の揺れ幅）
    uTailFrequency: { value: 14.0 },    // 周波数（くねり速度）
    uTailPhaseStride: { value: 4.0 },   // 体の長さ方向の位相差（波長に相当）
    uTailTurnStrength: { value: 0.1 },  // 旋回時の強度
    uTailSpeedScale: { value: 3 },      // 速度による影響度
    uTailRight: { value: new THREE.Vector3(1, 0, 0) },     // 尾アニメの右方向ベクトル
    uTailForward: { value: new THREE.Vector3(0, 1, 0) },   // 尾アニメの進行方向ベクトル
    uTailUp: { value: new THREE.Vector3(0, 0, 1) },        // 尾アニメの上方向ベクトル
    uTailEnable: { value: 0 }           // アニメーション有効/無効
  },
  smoothing: {
    speed: 0.25,  // 速度の追従係数（0に近いほど滑らか）
    turn: 0.1    // 旋回の追従係数
  }
};

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

// 魚のジオメトリに体の位置座標属性を追加する関数
function augmentFishGeometry(geometry) {
  if (!geometry) return geometry;
  if (geometry.getAttribute('aBodyCoord')) return geometry; // 既に追加済み

  const cloned = geometry.clone();
  cloned.computeBoundingBox();
  const bbox = cloned.boundingBox;
  const pos = cloned.attributes.position;
  const count = pos.count;

  const rangeX = bbox.max.x - bbox.min.x;
  const rangeY = bbox.max.y - bbox.min.y;
  const rangeZ = bbox.max.z - bbox.min.z;

  let axisIndex = 0;
  let axisRange = rangeX;
  if (rangeY >= axisRange && rangeY >= rangeZ) {
    axisIndex = 1;
    axisRange = rangeY;
  } else if (rangeZ >= axisRange && rangeZ >= rangeY) {
    axisIndex = 2;
    axisRange = rangeZ;
  }

  const axisMin = axisIndex === 0 ? bbox.min.x : axisIndex === 1 ? bbox.min.y : bbox.min.z;
  const axisMax = axisIndex === 0 ? bbox.max.x : axisIndex === 1 ? bbox.max.y : bbox.max.z;
  const range = Math.max(1e-4, axisMax - axisMin);

  const bodyCoord = new Float32Array(count);
  for (let i = 0; i < count; i++) {
    let coordSource;
    if (axisIndex === 0) {
      coordSource = pos.getX(i);
    } else if (axisIndex === 1) {
      coordSource = pos.getY(i);
    } else {
      coordSource = pos.getZ(i);
    }
    // 魚の頭が-Y方向なので、座標を反転（頭=0、尾=1）
    const coord = 1 - (coordSource - axisMin) / range;
    bodyCoord[i] = THREE.MathUtils.clamp(coord, 0, 1);
  }

  const forward = new THREE.Vector3(axisIndex === 0 ? 1 : 0, axisIndex === 1 ? 1 : 0, axisIndex === 2 ? 1 : 0);
  const upRef = Math.abs(forward.dot(new THREE.Vector3(0, 1, 0))) > 0.999 ? new THREE.Vector3(0, 0, 1) : new THREE.Vector3(0, 1, 0);
  const right = new THREE.Vector3().crossVectors(upRef, forward).normalize();
  const up = new THREE.Vector3().crossVectors(forward, right).normalize();

  cloned.userData.tailBasis = {
    forward: forward.toArray(),
    right: right.toArray(),
    up: up.toArray(),
    axisIndex
  };

  cloned.setAttribute('aBodyCoord', new THREE.BufferAttribute(bodyCoord, 1));
  return cloned;
}

// 尾のアニメーション用カスタムマテリアルを作成する関数
function createTailMaterial(sourceMaterial) {
  const material = sourceMaterial.clone();
  material.onBeforeCompile = (shader) => {
    // シェーダにuniformを追加
    shader.uniforms.uTailTime = tailAnimation.uniforms.uTailTime;
    shader.uniforms.uTailAmplitude = tailAnimation.uniforms.uTailAmplitude;
    shader.uniforms.uTailFrequency = tailAnimation.uniforms.uTailFrequency;
    shader.uniforms.uTailPhaseStride = tailAnimation.uniforms.uTailPhaseStride;
    shader.uniforms.uTailTurnStrength = tailAnimation.uniforms.uTailTurnStrength;
    shader.uniforms.uTailSpeedScale = tailAnimation.uniforms.uTailSpeedScale;
    shader.uniforms.uTailRight = tailAnimation.uniforms.uTailRight;
    shader.uniforms.uTailForward = tailAnimation.uniforms.uTailForward;
    shader.uniforms.uTailUp = tailAnimation.uniforms.uTailUp;
    shader.uniforms.uTailEnable = tailAnimation.uniforms.uTailEnable;

    // 頂点シェーダにuniformとattributeを追加
    shader.vertexShader = shader.vertexShader.replace(
      '#include <common>',
      `#include <common>
      uniform float uTailTime;
      uniform float uTailAmplitude;
      uniform float uTailFrequency;
      uniform float uTailPhaseStride;
      uniform float uTailTurnStrength;
      uniform float uTailSpeedScale;
      uniform vec3 uTailRight;
      uniform vec3 uTailForward;
      uniform vec3 uTailUp;
      uniform float uTailEnable;
      attribute float aBodyCoord;
      attribute float instanceTailPhase;
      attribute float instanceTailSpeed;
      attribute float instanceTailTurn;
      `
    );

    // 頂点変換処理に尾のアニメーションを追加（メイン描画用）
    shader.vertexShader = shader.vertexShader.replace(
      '#include <begin_vertex>',
      `#include <begin_vertex>
      if (uTailEnable > 0.5) {
        vec3 originalPos = transformed;
        vec3 right = normalize(uTailRight);
        vec3 forward = normalize(uTailForward);
        vec3 up = normalize(uTailUp);
        
        float localX = dot(originalPos, right);
        float localY = dot(originalPos, forward);
        float localZ = dot(originalPos, up);
        
        float bodyCoord = clamp(aBodyCoord, 0.0, 1.0);
        float tailWeight = smoothstep(0.0, 0.35, bodyCoord);
        float speedFactor = clamp(instanceTailSpeed * uTailSpeedScale, 0.0, 2.0);
        
        // 波形計算
        float phase = instanceTailPhase + uTailTime * uTailFrequency;
        float wavePhase = phase + bodyCoord * uTailPhaseStride;
        float wag = sin(wavePhase) * uTailAmplitude;
        float turnOffset = instanceTailTurn * uTailTurnStrength;
        float motion = wag * (0.4 + 0.6 * speedFactor) + turnOffset;
        
        // 先端部分で揺れを減衰（0.8以上で減衰開始）
        float tipDamping = 1.0 - smoothstep(0.7, 1.0, bodyCoord) * 0.3;
        float bendStrength = mix(0.02, 1.0, tailWeight) * tipDamping;
        
        // 回転変形
        float bendAngle = motion * bendStrength;
        float s = sin(bendAngle);
        float c = cos(bendAngle);
        float rotX = localX * c - localY * s;
        float rotY = localX * s + localY * c;
        
        // 横揺れ
        float sway = motion * 0.4 * tipDamping;
        rotX += sway * (0.05 + 0.95 * bodyCoord);
        
        vec3 rotated = right * rotX + forward * rotY + up * localZ;
        transformed = rotated;
      }
      `
    );

    // 深度パス（SSAO、シャドウマップ用）での頂点変換
    shader.vertexShader = shader.vertexShader.replace(
      '#include <project_vertex>',
      `#include <project_vertex>
      // 深度パス用の尾アニメーション変形
      #ifdef DEPTH_PACKING
      if (uTailEnable > 0.5) {
        vec3 viewPos = mvPosition.xyz;
        mat3 normalMatrix3 = mat3(normalMatrix);
        vec3 right = normalize(normalMatrix3 * uTailRight);
        vec3 forward = normalize(normalMatrix3 * uTailForward);
        vec3 up = normalize(normalMatrix3 * uTailUp);
        
        float localX = dot(viewPos, right);
        float localY = dot(viewPos, forward);
        float localZ = dot(viewPos, up);
        
        float bodyCoord = clamp(aBodyCoord, 0.0, 1.0);
        float tailWeight = smoothstep(0.0, 0.35, bodyCoord);
        float speedFactor = clamp(instanceTailSpeed * uTailSpeedScale, 0.0, 2.0);
        
        // 波形計算
        float phase = instanceTailPhase + uTailTime * uTailFrequency;
        float wavePhase = phase + bodyCoord * uTailPhaseStride;
        float wag = sin(wavePhase) * uTailAmplitude;
        float turnOffset = instanceTailTurn * uTailTurnStrength;
        float motion = wag * (0.4 + 0.6 * speedFactor) + turnOffset;
        
        // 先端部分で揺れを減衰（深度パス用）
        float tipDamping = 1.0 - smoothstep(0.7, 1.0, bodyCoord) * 0.3;
        float bendStrength = mix(0.02, 1.0, tailWeight) * tipDamping;
        
        // 回転変形
        float bendAngle = motion * bendStrength;
        float s = sin(bendAngle);
        float c = cos(bendAngle);
        float rotX = localX * c - localY * s;
        float rotY = localX * s + localY * c;
        
        // 横揺れ
        float sway = motion * 0.4 * tipDamping;
        rotX += sway * (0.05 + 0.95 * bodyCoord);
        
        vec3 rotated = right * rotX + forward * rotY + up * localZ;
        mvPosition.xyz = rotated;
        gl_Position = projectionMatrix * mvPosition;
      }
      #endif
      `
    );
  };
  return material;
}

// 尾のアニメーション用インスタンス属性を初期化する関数
function ensureTailAttributes(count) {
  // 各インスタンスにランダムな位相オフセットを設定
  const phaseArray = new Float32Array(count);
  for (let i = 0; i < count; i++) {
    phaseArray[i] = Math.random() * Math.PI * 2;
  }
  
  // インスタンス属性を作成
  tailAnimation.phaseAttribute = new THREE.InstancedBufferAttribute(phaseArray, 1);
  tailAnimation.speedAttribute = new THREE.InstancedBufferAttribute(new Float32Array(count), 1);
  tailAnimation.turnAttribute = new THREE.InstancedBufferAttribute(new Float32Array(count), 1);
  
  // 動的更新を有効にする
  tailAnimation.speedAttribute.setUsage(THREE.DynamicDrawUsage);
  tailAnimation.turnAttribute.setUsage(THREE.DynamicDrawUsage);
  
  // 前フレームの速度バッファを初期化
  tailAnimation.previousVelocities = new Float32Array(count * 3);
  tailAnimation.previousVelocities.fill(0);
  tailAnimation.smoothedSpeed = new Float32Array(count);
  tailAnimation.smoothedTurn = new Float32Array(count);
  
  // 高品質メッシュに属性を追加
  if (instancedMeshHigh) {
    instancedMeshHigh.geometry.setAttribute('instanceTailPhase', tailAnimation.phaseAttribute);
    instancedMeshHigh.geometry.setAttribute('instanceTailSpeed', tailAnimation.speedAttribute);
    instancedMeshHigh.geometry.setAttribute('instanceTailTurn', tailAnimation.turnAttribute);
  }
  
  // LODメッシュに属性を追加
  if (instancedMeshLow) {
    instancedMeshLow.geometry.setAttribute('instanceTailPhase', tailAnimation.phaseAttribute);
    instancedMeshLow.geometry.setAttribute('instanceTailSpeed', tailAnimation.speedAttribute);
    instancedMeshLow.geometry.setAttribute('instanceTailTurn', tailAnimation.turnAttribute);
  }
  
  // 更新フラグを設定
  tailAnimation.phaseAttribute.needsUpdate = true;
  tailAnimation.speedAttribute.needsUpdate = true;
  tailAnimation.turnAttribute.needsUpdate = true;
  tailAnimation.elapsedTime = 0;
  tailAnimation.uniforms.uTailTime.value = 0;
}

// ジオメトリから尾アニメーションの基準ベクトルを設定
function setTailBasisFromGeometry(geometry) {
  if (!geometry || !geometry.userData?.tailBasis) return;
  const basis = geometry.userData.tailBasis;
  if (basis.right) {
    tailAnimation.uniforms.uTailRight.value.fromArray(basis.right);
  }
  if (basis.forward) {
    tailAnimation.uniforms.uTailForward.value.fromArray(basis.forward);
  }
  if (basis.up) {
    tailAnimation.uniforms.uTailUp.value.fromArray(basis.up);
  }
}

// 尾のアニメーションマテリアルの有効/無効を切り替える関数
function updateTailAnimationMaterials() {
  const enabled = enableTailAnimation.value && tailAnimation.animatedHighMaterial && instancedMeshHigh;
  tailAnimation.uniforms.uTailEnable.value = enabled ? 1 : 0;
  
  if (!instancedMeshHigh) return;
  
  if (enabled) {
    // アニメーション有効時：カスタムマテリアルに切り替え
    if (tailAnimation.animatedHighMaterial) {
      instancedMeshHigh.material = tailAnimation.animatedHighMaterial;
      instancedMeshHigh.material.needsUpdate = true;
    }
    if (instancedMeshLow && tailAnimation.animatedLowMaterial) {
      instancedMeshLow.material = tailAnimation.animatedLowMaterial;
      instancedMeshLow.material.needsUpdate = true;
    }
  } else {
    // アニメーション無効時：元のマテリアルに戻す
    if (tailAnimation.baseHighMaterial) {
      instancedMeshHigh.material = tailAnimation.baseHighMaterial;
      instancedMeshHigh.material.needsUpdate = true;
    }
    if (instancedMeshLow && tailAnimation.baseLowMaterial) {
      instancedMeshLow.material = tailAnimation.baseLowMaterial;
      instancedMeshLow.material.needsUpdate = true;
    }
  }
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

// WebGLレンダラー初期化（段階的フォールバック対応）
function initThreeJS({ rebuildScene = false } = {}) {
  const width = window.innerWidth;
  const height = window.innerHeight;
  const isMobile = isMobileDevice();

  // シーン再構築またはコンテキストロス復旧時
  if (rebuildScene || !scene) {
    scene = new THREE.Scene();
    scene.fog = new THREE.Fog(toHex(OCEAN_COLORS.FOG), 3, 14);

    camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.position.set(2.19, -5.80, 5.76);
    camera.lookAt(0, 0, 0);

    const groundGeo = new THREE.PlaneGeometry(100, 100);
    groundMaterial = createFadeOutGroundMaterial();
    const ground = new THREE.Mesh(groundGeo, groundMaterial);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -7;
    ground.receiveShadow = true;
    scene.add(ground);

    createOceanSphere();

    const ambientLight = new THREE.AmbientLight(toHex(OCEAN_COLORS.AMBIENT_LIGHT), 1.3);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(toHex(OCEAN_COLORS.SUN_LIGHT), 23.0);
    dirLight.position.set(0, 60, 30);
    dirLight.castShadow = !isMobile;

    const bottomLight = new THREE.DirectionalLight(toHex(OCEAN_COLORS.BOTTOM_LIGHT), 0);
    bottomLight.position.set(0, -30, 0);
    scene.add(bottomLight);

    if (!isMobile) {
      dirLight.shadow.camera.left = -100;
      dirLight.shadow.camera.right = 100;
      dirLight.shadow.camera.top = 100;
      dirLight.shadow.camera.bottom = -100;
      dirLight.shadow.camera.near = 0.2;
      dirLight.shadow.camera.far = 1000;
      dirLight.shadow.mapSize.width = 2048;
      dirLight.shadow.mapSize.height = 2048;
      dirLight.shadow.bias = -0.001;
      dirLight.shadow.normalBias = 0.01;
    }

    scene.add(dirLight);
  } else {
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }

  // 失敗回数に応じて軽量設定に切り替え
  const configFactory = rendererConfigs[Math.min(rendererInitAttempt, rendererConfigs.length - 1)];
  const rendererOptions = configFactory(isMobile);

  let newRenderer = null;
  try {
    newRenderer = new THREE.WebGLRenderer(rendererOptions);
  } catch (error) {
    console.warn('WebGLRenderer creation failed:', error);
    rendererInitAttempt = Math.min(rendererInitAttempt + 1, rendererConfigs.length - 1);
    scheduleRendererRetry();
    return false;
  }

  const gl = newRenderer.getContext();
  if (!gl) {
    console.warn('WebGL context could not be created.');
    newRenderer.dispose();
    rendererInitAttempt = Math.min(rendererInitAttempt + 1, rendererConfigs.length - 1);
    scheduleRendererRetry();
    return false;
  }

  renderer = newRenderer;
  if (rendererRetryTimer) {
    clearTimeout(rendererRetryTimer);
    rendererRetryTimer = null;
  }

  const basePixelRatio = isMobile ? Math.min(window.devicePixelRatio, 2) : window.devicePixelRatio;
  const pixelRatio = rendererInitAttempt === 0 ? basePixelRatio : 1;
  renderer.setPixelRatio(pixelRatio);
  renderer.setSize(width, height);
  // 復旧時は影を無効化してパフォーマンス向上
  const enableShadows = !isMobile && rendererInitAttempt === 0;
  renderer.shadowMap.enabled = enableShadows;
  if (enableShadows) {
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  }
  renderer.outputColorSpace = THREE.SRGBColorSpace;

  if (threeContainer.value) {
    threeContainer.value.appendChild(renderer.domElement);
  }

  lastRendererCanvas = renderer.domElement;
  lastRendererCanvas.addEventListener('click', handleCanvasClick);

  setupWebGLContextLossHandling();

  if (controls) {
    controls.removeEventListener('start', handleControlsInteractionStart);
    controls.removeEventListener('end', handleControlsInteractionEnd);
    controls.dispose();
  }
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.1;
  controls.addEventListener('start', handleControlsInteractionStart);
  controls.addEventListener('end', handleControlsInteractionEnd);

  // 復旧時はポストプロセシングを無効化
  const enableComposer = !isMobile && rendererInitAttempt === 0;
  if (enableComposer) {
    composer = new EffectComposer(renderer);
    const renderPass = new RenderPass(scene, camera);
    composer.addPass(renderPass);

    const ssaoPass = new SSAOPass(scene, camera, width, height);
    ssaoPass.kernelRadius = 8;
    ssaoPass.minDistance = 0.001;
    ssaoPass.maxDistance = 0.01;
    composer.addPass(ssaoPass);

    const bloomPass = new UnrealBloomPass(new THREE.Vector2(width, height), 1.5, 0.4, 0.85);
    composer.addPass(bloomPass);
  } else {
    composer = null;
  }

  window.removeEventListener('resize', onWindowResize);
  window.addEventListener('resize', onWindowResize);

  rendererInitAttempt = 0;
  tryStartSimulation();
  return true;
}

function onWindowResize() {
  if (!camera || !renderer) return;
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

  if (tailAnimation.animatedHighMaterial) {
    tailAnimation.animatedHighMaterial.dispose();
  }
  if (tailAnimation.animatedLowMaterial) {
    tailAnimation.animatedLowMaterial.dispose();
  }
  tailAnimation.baseHighMaterial = instancedMeshHigh.material;
  tailAnimation.baseLowMaterial = instancedMeshLow.material;
  setTailBasisFromGeometry(instancedMeshHigh.geometry);
  setTailBasisFromGeometry(instancedMeshLow.geometry);
  tailAnimation.animatedHighMaterial = createTailMaterial(instancedMeshHigh.material);
  tailAnimation.animatedLowMaterial = createTailMaterial(instancedMeshLow.material);
  ensureTailAttributes(count);
  updateTailAnimationMaterials();

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
          child.geometry = augmentFishGeometry(child.geometry);
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
          child.geometry = augmentFishGeometry(child.geometry);
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
  // シーン復旧中の場合は配列のみクリア
  if (!scene) {
    unitSpheres = [];
    unitLines = [];
    return;
  }
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
  animationTimer = setTimeout(() => {
    animationTimer = null;
    animate();
  }, FRAME_INTERVAL);
}

// アニメーションループを安全に停止
function stopAnimationLoop() {
  if (animationTimer) {
    clearTimeout(animationTimer);
    animationTimer = null;
  }
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
  // レンダラー復旧中は描画をスキップ
  if (!renderer || !scene || !camera) {
    return;
  }
  const count = boidCount();
  const heapF32 = wasmModule.HEAPF32.buffer;
  const positions = new Float32Array(heapF32, posPtr(), count * 3);
  const orientations = new Float32Array(heapF32, oriPtr(), count * 4);
  const velocities = new Float32Array(heapF32, velPtr(), count * 3);
  const dummy = new THREE.Object3D();
  const camPos = camera.position;

  const isInstanced = useInstancedRendering.value && instancedMeshHigh && instancedMeshLow;

  // 尾のアニメーション時間を更新
  if (!paused.value) {
    tailAnimation.elapsedTime += deltaTime;
  }
  tailAnimation.uniforms.uTailTime.value = tailAnimation.elapsedTime;

  // 尾アニメーションの利用可否を判定
  const tailArraysAvailable = enableTailAnimation.value && isInstanced && tailAnimation.speedAttribute && tailAnimation.turnAttribute;
  const tailUpdateEnabled = tailArraysAvailable && !paused.value;
  tailAnimation.uniforms.uTailEnable.value = tailArraysAvailable ? 1 : 0;
  
  // 尾アニメーション用バッファを取得
  let tailSpeedArray = null;
  let tailTurnArray = null;
  let prevVelocities = null;
  let smoothedSpeedBuffer = null;
  let smoothedTurnBuffer = null;
  let speedSmoothing = 1;
  let turnSmoothing = 1;
  if (tailArraysAvailable) {
    tailSpeedArray = tailAnimation.speedAttribute.array;
    tailTurnArray = tailAnimation.turnAttribute.array;
    prevVelocities = tailAnimation.previousVelocities;
    smoothedSpeedBuffer = tailAnimation.smoothedSpeed;
    smoothedTurnBuffer = tailAnimation.smoothedTurn;
    speedSmoothing = THREE.MathUtils.clamp(tailAnimation.smoothing?.speed ?? 1, 0, 1);
    turnSmoothing = THREE.MathUtils.clamp(tailAnimation.smoothing?.turn ?? 1, 0, 1);
  }
  let tailDataDirty = false; // バッファ更新フラグ

  updateCameraFollow(positions, orientations);

  // 使用するメッシュを決定（早期宣言）
  let activeMeshHigh = isInstanced ? instancedMeshHigh : null;
  let activeMeshLow = isInstanced ? instancedMeshLow : null;
  let highMatrixArray = null;
  let lowMatrixArray = null;
  if (isInstanced) {
    highMatrixArray = activeMeshHigh.instanceMatrix.array;
    lowMatrixArray = activeMeshLow.instanceMatrix.array;
  }

  if (!isInstanced && boidLODs.length !== count) {
    initBoidLODs(count);
  }

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

    // 現在フレームの速度ベクトルを取得
    const vBase = i * 3;
    const vx = velocities[vBase];
    const vy = velocities[vBase + 1];
    const vz = velocities[vBase + 2];
    
    // 前フレームの速度ベクトルを取得（旋回量計算用）
    const hasPrevVelocityBuffer = tailArraysAvailable && !!prevVelocities;
    let prevVx = 0;
    let prevVy = 0;
    let prevVz = 0;
    if (hasPrevVelocityBuffer) {
      prevVx = prevVelocities[vBase];
      prevVy = prevVelocities[vBase + 1];
      prevVz = prevVelocities[vBase + 2];
    }

    // 尾のアニメーションに参加するかを判定（高品質またはLOD適用時）
    const participatesInTail = tailArraysAvailable && (useHigh || tailAnimation.applyToLod);
    if (participatesInTail && tailUpdateEnabled) {
      // 現在の速度を計算
      const rawSpeed = Math.sqrt(vx * vx + vy * vy + vz * vz);
      
      // 旋回量を計算（前フレームとの向きの変化から角速度を求める）
      let turnValue = 0;
      if (hasPrevVelocityBuffer) {
        const prevLen = Math.hypot(prevVx, prevVz); // 前フレームの水平速度
        const currLen = Math.hypot(vx, vz);         // 現在フレームの水平速度
        if (currLen > 1e-4 && prevLen > 1e-4) {
          // 正規化された速度ベクトルで角度差を計算
          const prevX = prevVx / prevLen;
          const prevZ = prevVz / prevLen;
          const currX = vx / currLen;
          const currZ = vz / currLen;
          const det = prevX * currZ - prevZ * currX; // 外積のY成分（回転方向）
          const dot = THREE.MathUtils.clamp(prevX * currX + prevZ * currZ, -1, 1); // 内積
          const angle = Math.atan2(det, dot);        // 角度差
          const dt = Math.max(deltaTime, 1e-3);      // フレーム間時間
          turnValue = THREE.MathUtils.clamp(angle / dt, -2.5, 2.5); // 角速度に変換
        }
      }

      // スムージング処理（カクつきを抑えるため前フレームとの線形補間）
      const previousSpeed = smoothedSpeedBuffer ? smoothedSpeedBuffer[i] : rawSpeed;
      const previousTurn = smoothedTurnBuffer ? smoothedTurnBuffer[i] : turnValue;
      const smoothedSpeed = THREE.MathUtils.lerp(previousSpeed, rawSpeed, speedSmoothing);
      const smoothedTurn = THREE.MathUtils.lerp(previousTurn, turnValue, turnSmoothing);

      // スムージング用バッファを更新
      if (smoothedSpeedBuffer) smoothedSpeedBuffer[i] = smoothedSpeed;
      if (smoothedTurnBuffer) smoothedTurnBuffer[i] = smoothedTurn;

      // インスタンス属性に反映し、変化があればGPU更新フラグを立てる
      const prevSpeedAttr = tailSpeedArray[i];
      const prevTurnAttr = tailTurnArray[i];
      tailSpeedArray[i] = smoothedSpeed;
      tailTurnArray[i] = smoothedTurn;
      if (!tailDataDirty && (Math.abs(prevSpeedAttr - smoothedSpeed) > 1e-4 || Math.abs(prevTurnAttr - smoothedTurn) > 1e-4)) {
        tailDataDirty = true;
      }

      // 次フレーム用に現在の速度を保存
      if (hasPrevVelocityBuffer) {
        prevVelocities[vBase] = vx;
        prevVelocities[vBase + 1] = vy;
        prevVelocities[vBase + 2] = vz;
      }
    } else if (participatesInTail && hasPrevVelocityBuffer && !tailUpdateEnabled) {
      // 停止中でも速度バッファを更新（再生時の初期値として使用）
      prevVelocities[vBase] = vx;
      prevVelocities[vBase + 1] = vy;
      prevVelocities[vBase + 2] = vz;
      if (smoothedSpeedBuffer) smoothedSpeedBuffer[i] = Math.sqrt(vx * vx + vy * vy + vz * vz);
      if (smoothedTurnBuffer) smoothedTurnBuffer[i] = 0;
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

  // 尾のアニメーションバッファが更新されたらGPUに送信
  if (tailDataDirty) {
    tailAnimation.speedAttribute.needsUpdate = true;
    tailAnimation.turnAttribute.needsUpdate = true;
  }

  controls?.update();

  // ポストプロセシングかダイレクトレンダリング
  if (composer) {
    composer.render(scene, camera);
  } else {
    renderer.render(scene, camera);
  }
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

// シミュレーション開始をキューに登録（非同期初期化対応）
function queueSimulationStart(resetSimulation = true) {
  pendingSimulationReset = resetSimulation;
  simulationPending = true;
  simulationInitialized = false;
  tryStartSimulation();
}

// 条件が整った時点でシミュレーションを開始
function tryStartSimulation() {
  if (!simulationPending) return;
  if (!renderer || !scene || !camera) return;
  if (!boidModel || !boidModelLod) return;
  simulationPending = false;
  simulationInitialized = true;
  const shouldReset = pendingSimulationReset;
  pendingSimulationReset = true;
  startSimulation(shouldReset);
}

function startSimulation(resetSimulation = true) {
  // WebAssembly モジュール用に SpeciesParams を初期化
  if (resetSimulation) {
    const vector = createSpeciesParamsVector(settings);
    wasmModule.callInitBoids(vector, 1, 3, 0.25);
  }
  build(16, 0);
  initInstancedBoids(calculateTotalBoidCount(settings));
  // 既存のループを停止してから新しいループを開始
  stopAnimationLoop();
  lastTime = performance.now();
  animate();
}

// 初期化時のコールバック（頻度が低いため匿名関数でも問題なし）
onMounted(() => {
  initThreeJS({ rebuildScene: true });
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

    queueSimulationStart();
    
    // メモリ監視を開始
    startMemoryMonitoring();
  });
  window.addEventListener('keydown', handleKeydown);
});

onBeforeUnmount(() => {
  window.removeEventListener('keydown', handleKeydown);
  disposeThreeResources();
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

watch(enableTailAnimation, () => {
  updateTailAnimationMaterials();
}, { immediate: true });

watch(enableTailOnLod, (value) => {
  tailAnimation.applyToLod = value;
  updateTailAnimationMaterials();
}, { immediate: true });

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

// Three.jsリソースを安全に破棄（メモリリーク防止）
function disposeThreeResources() {
  stopCameraFollow();
  stopAnimationLoop();
  if (composer) {
    composer.renderTarget1?.dispose?.();
    composer.renderTarget2?.dispose?.();
    composer = null;
  }
  if (controls) {
    controls.removeEventListener('start', handleControlsInteractionStart);
    controls.removeEventListener('end', handleControlsInteractionEnd);
    controls.dispose();
    controls = null;
  }
  if (renderer) {
    const canvas = renderer.domElement;
    canvas.removeEventListener('click', handleCanvasClick);
    canvas.removeEventListener('webglcontextlost', handleWebGLContextLost);
    canvas.removeEventListener('webglcontextrestored', handleWebGLContextRestored);
    if (canvas.parentNode) {
      canvas.parentNode.removeChild(canvas);
    }
    renderer.dispose();
    renderer = null;
  }
  window.removeEventListener('resize', onWindowResize);
  lastRendererCanvas = null;
  scene = null;
  camera = null;
  unitSpheres = [];
  unitLines = [];
  instancedMeshHigh = null;
  instancedMeshLow = null;
  predatorMarkers = [];
  boidLodStates = [];
  speciesIndexLookup = [];
  if (rendererRetryTimer) {
    clearTimeout(rendererRetryTimer);
    rendererRetryTimer = null;
  }
}

// 遅延してレンダラー復旧を試行（モバイル考慮）
function scheduleRendererRetry() {
  if (rendererRetryTimer) {
    return;
  }
  const delay = isMobileDevice() ? 1500 : 800;
  rendererRetryTimer = setTimeout(() => {
    rendererRetryTimer = null;
    initThreeJS({ rebuildScene: true });
  }, delay);
}

// WebGLコンテキストロス時の自動復旧処理
function handleWebGLContextLost(event) {
  console.warn('WebGLコンテキストが失われました。自動復旧を試みます。');
  event.preventDefault();
  rendererInitAttempt = Math.min(rendererInitAttempt + 1, rendererConfigs.length - 1);
  disposeThreeResources();
  queueSimulationStart(true);
  scheduleRendererRetry();
}

function handleWebGLContextRestored() {
  console.info('WebGLコンテキストが復旧しました。');
}

// メモリ不足時の復旧処理（ページリロードの代替）
function handleLowMemoryRecovery() {
  console.warn('WebGLメモリ使用量が高いため、レンダラーの再初期化を試みます。');
  rendererInitAttempt = Math.min(rendererInitAttempt + 1, rendererConfigs.length - 1);
  disposeThreeResources();
  queueSimulationStart(true);
  scheduleRendererRetry();
}

// WebGLコンテキストロス・復旧イベントの設定
function setupWebGLContextLossHandling() {
  if (!renderer) return;

  const canvas = renderer.domElement;
  canvas.removeEventListener('webglcontextlost', handleWebGLContextLost);
  canvas.removeEventListener('webglcontextrestored', handleWebGLContextRestored);
  canvas.addEventListener('webglcontextlost', handleWebGLContextLost, false);
  canvas.addEventListener('webglcontextrestored', handleWebGLContextRestored, false);
}



// 軽量メモリ監視（モバイル向け）
// WebGLメモリ監視（モバイル向け軽量版）
function monitorWebGLMemory() {
  if (!renderer || !renderer.info || !isMobileDevice()) return;
  
  const info = renderer.info;
  const memoryInfo = info.memory || {};
  const renderInfo = info.render || {};
  
  const geometries = memoryInfo.geometries || 0;
  const textures = memoryInfo.textures || 0;
  
  // メモリ使用量が非常に高い場合のみ復旧を試みる
  if (geometries > 100 || textures > 50) {
    handleLowMemoryRecovery();
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