<template>
  <div id="app">
    <div class="ui-overlay">
      <div class="ui-panel">
        <h1>Boids Simulation</h1>
        <details>
          <summary>Settings</summary>
          <div v-for="(s, i) in settings" :key="i">
            <Settings
              :settings="s"
              :can-remove="settings.length > 1"
              @remove="removeSpecies(i)"
            />
          </div>
          <button class="add-species-button" @click="addSpecies">
            種族を追加
          </button>
          <button @click="resetSettings" style="margin-bottom: 1em">
            リセット
          </button>
          <br />
          <div class="settings tuning-settings">
            <details class="species-section" :open="false">
              <summary class="species-header">
                <span class="species-title">Adjustment</span>
              </summary>
              <div class="species-content">
                <div class="setting-row">
                  <label :title="tuningHelp.threatDecay">脅威減衰<br />(threatDecay):</label>
                  <input
                    type="range"
                    min="0"
                    max="5"
                    step="0.05"
                    v-model.number="systemSettings.threatDecay"
                    :title="tuningHelp.threatDecay"
                  />
                  <input
                    class="value-input"
                    type="number"
                    step="0.05"
                    v-model.number="systemSettings.threatDecay"
                    :title="tuningHelp.threatDecay"
                  />
                </div>
                <div class="setting-row">
                  <label :title="tuningHelp.maxEscapeWeight">逃避優先度<br />(maxEscapeWeight):</label>
                  <input
                    type="range"
                    min="0"
                    max="1"
                    step="0.01"
                    v-model.number="systemSettings.maxEscapeWeight"
                    :title="tuningHelp.maxEscapeWeight"
                  />
                  <input
                    class="value-input"
                    type="number"
                    step="0.01"
                    v-model.number="systemSettings.maxEscapeWeight"
                    :title="tuningHelp.maxEscapeWeight"
                  />
                </div>
                <div class="setting-row">
                  <label :title="tuningHelp.baseEscapeStrength">逃避舵取り強度<br />(baseEscapeStrength):</label>
                  <input
                    type="range"
                    min="0"
                    max="15"
                    step="0.1"
                    v-model.number="systemSettings.baseEscapeStrength"
                    :title="tuningHelp.baseEscapeStrength"
                  />
                  <input
                    class="value-input"
                    type="number"
                    step="0.1"
                    v-model.number="systemSettings.baseEscapeStrength"
                    :title="tuningHelp.baseEscapeStrength"
                  />
                </div>
                <div class="setting-row">
                  <label :title="tuningHelp.fastAttractStrength">補助凝集強度<br />(fastAttractStrength):</label>
                  <input
                    type="range"
                    min="0"
                    max="3"
                    step="0.05"
                    v-model.number="systemSettings.fastAttractStrength"
                    :title="tuningHelp.fastAttractStrength"
                  />
                  <input
                    class="value-input"
                    type="number"
                    step="0.05"
                    v-model.number="systemSettings.fastAttractStrength"
                    :title="tuningHelp.fastAttractStrength"
                  />
                </div>
                <div class="setting-row">
                  <label :title="tuningHelp.schoolPullCoefficient">大クラスタ引力係数<br />(schoolPullCoefficient):</label>
                  <input
                    type="range"
                    min="0"
                    max="0.005"
                    step="0.00001"
                    v-model.number="systemSettings.schoolPullCoefficient"
                    :title="tuningHelp.schoolPullCoefficient"
                  />
                  <input
                    class="value-input"
                    type="number"
                    min="0"
                    step="0.00001"
                    v-model.number="systemSettings.schoolPullCoefficient"
                    :title="tuningHelp.schoolPullCoefficient"
                  />
                </div>
              </div>
            </details>
          </div>
          <br />
          <div class="settings tuning-settings debug-settings">
            <details class="species-section" :open="false">
              <summary class="species-header">
                <span class="species-title">Debug</span>
              </summary>
              <div class="species-content debug-content">
                <label class="debug-checkbox" :title="debugHelp.enableFogPipeline">
                  <input
                    type="checkbox"
                    v-model="debugControls.enableFogPipeline"
                    :title="debugHelp.enableFogPipeline"
                  />
                  フォグ/SSAO/ブルームを有効化
                </label>
                <label class="debug-checkbox" :title="debugHelp.enableShadows">
                  <input type="checkbox" v-model="debugControls.enableShadows" :title="debugHelp.enableShadows" />
                  影描画を有効化
                </label>
                <label class="debug-checkbox" :title="debugHelp.enableTailAnimation">
                  <input
                    type="checkbox"
                    v-model="debugControls.enableTailAnimation"
                    :title="debugHelp.enableTailAnimation"
                  />
                  尾びれアニメーションを有効化
                </label>
                <label class="debug-checkbox" :title="debugHelp.showSpeciesEnvelopes">
                  <input type="checkbox" v-model="showSpeciesEnvelopes" :title="debugHelp.showSpeciesEnvelopes" />
                  種族エンベロープ表示
                </label>
                <label class="debug-checkbox" :title="debugHelp.showSpeciesClusters">
                  <input type="checkbox" v-model="showSpeciesClusters" :title="debugHelp.showSpeciesClusters" />
                  クラスター表示
                </label>
                <label class="debug-checkbox" :title="debugHelp.showSpeciesSchoolClusters">
                  <input type="checkbox" v-model="showSpeciesSchoolClusters" :title="debugHelp.showSpeciesSchoolClusters" />
                  大クラスター表示
                </label>
                <label class="debug-checkbox" :title="debugHelp.showWorldAxisGrid">
                  <input type="checkbox" v-model="showWorldAxisGrid" :title="debugHelp.showWorldAxisGrid" />
                  ワールド座標グリッド/軸（目盛り）表示
                </label>
              </div>
            </details>
          </div>
        </details>
      <div class="info">
        <p>Boids Count: {{ totalBoids }}</p>
      </div>
    </div>
    </div>

    <!--
      デバッグHUD（ログではなく画面表示）
      種族エンベロープの中心座標と半径を確認する用途。
      - showSpeciesEnvelopes が ON のときだけ表示する。
      - 文字列更新はフレーム毎ではなく間引き、性能劣化を避ける。
    -->
    <pre v-if="showSpeciesEnvelopes && speciesEnvelopeHudText" class="debug-hud">
{{ speciesEnvelopeHudText }}
    </pre>

    <div ref="threeContainer" class="three-container" />
    <audio
      ref="backgroundAudio"
      src="/UnderWater.mp3"
      loop
      style="display: none"
    />
  </div>
</template>

<script setup>
import { inject, onMounted, onUnmounted, reactive, ref, watch, toRaw } from "vue";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import Settings from "./components/Settings.vue";
import StatsGl from "stats-gl";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";
import { BoidInstancing } from "./rendering/BoidInstancing.js";
import { FogPipeline } from "./rendering/FogPipeline.js";
import { ParticleField } from "./rendering/ParticleField.js";
import { WasmtimeBridge } from "./simulation/WasmtimeBridge.js";
import { createFlockSettingsStore } from "./state/FlockSettingsStore.js";

const wasmModule = inject("wasmModule");
if (!wasmModule) {
  console.error("wasmModule not provided");
}

const wasmBridge = wasmModule ? new WasmtimeBridge(wasmModule) : null;

// const getUnitCount = wasmModule.cwrap('getUnitCount', 'number', []);
// const getUnitParentIndicesPtr = wasmModule.cwrap('getUnitParentIndicesPtr', 'number', []);

function detectDeviceProfile() {
  const profile = {
    isMobile: false,
    hasIntegratedGpu: false,
  };

  if (typeof navigator === "undefined") {
    return profile;
  }

  // モバイルデバイスを最優先で認識
  profile.isMobile =
    /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(
      navigator.userAgent
    );
  if (profile.isMobile) {
    return profile;
  }

  // Intel内蔵GPU（ノートPCでのエントリーGPU）を検出
  try {
    const canvas = document.createElement("canvas");
    const gl = canvas.getContext("webgl2") || canvas.getContext("webgl");
    if (gl) {
      const debugInfo = gl.getExtension("WEBGL_debug_renderer_info");
      if (debugInfo) {
        const renderer = gl.getParameter(debugInfo.UNMASKED_RENDERER_WEBGL);
        if (renderer && /intel/i.test(renderer)) {
          console.log("Detected Intel integrated GPU:", renderer);
          profile.hasIntegratedGpu = true;
        }
      }
    }
  } catch (error) {
    console.warn("Failed to inspect GPU characteristics:", error);
  }

  return profile;
}

function fetchTreeStructure() {
  return wasmBridge?.getDiagnostics?.({ treeStructure: true })?.treeStructure ?? null;
}
const deviceProfile = detectDeviceProfile();
const useLowSpecPreset =
  deviceProfile.isMobile || deviceProfile.hasIntegratedGpu;

// 高DPI（devicePixelRatio>1）をそのまま使うと、ピクセル塗りつぶしが増えて
// GPU/ポストプロセスがボトルネックになりやすい。
// 60fpsを優先し、環境に応じて pixelRatio の上限を設ける。
const MAX_RENDER_PIXEL_RATIO = useLowSpecPreset ? 1.0 : 1.5;
// 画面ガワのデフォルト（画像の値）
// NOTE: 個体数は重い環境でも動かしやすい値を優先する。
const defaultBoidCount = useLowSpecPreset ? 5000 : 10000;

const DEFAULT_SETTINGS = [
  {
    species: "Boids", // 種族名
    count: defaultBoidCount, // 群れの数（低スペックでは軽量化）
    // 画面ガワの初期値（画像の値）
    cohesion: 3.66, // 凝集
    cohesionRange: 5, // 凝集範囲
    separation: 2.15, // 分離
    separationRange: 0.4, // 分離範囲
    alignment: 10.0, // 整列
    alignmentRange: 1, // 整列範囲
    maxSpeed: 0.35, // 最大速度
    maxTurnAngle: 0.75, // 最大曲がり（曲率）
    maxNeighbors: 4, // 最大近傍数
    horizontalTorque: 0.03, // 水平化トルク
    torqueStrength: 1.0, // 回転トルク強度
    lambda: 0.1, // 速度調整係数（減衰係数）
    tau: 0.5, // 記憶時間
    predatorAlertRadius: 1.5, // 捕食者を察知する距離
    densityReturnStrength: 0.0, // 密度復帰強度
    isPredator: false,
  },
  {
    species: "Predator",
    count: 3,
    cohesion: 0.0, // 捕食者は群集力学を使わない
    cohesionRange: 5.0,
    separation: 0.0,
    separationRange: 0.1,
    alignment: 0.0,
    alignmentRange: 1.0,
    predatorAlertRadius: 0.0,
    densityReturnStrength: 0.0,
    maxSpeed: 1.0,
    minSpeed: 0.5,
    maxTurnAngle: 0.4,
    maxNeighbors: 0,
    lambda: 0.05,
    tau: 1.0, // 捕食者は常に追いかける
    horizontalTorque: 0.01,
    torqueStrength: 2.5,
    isPredator: true, // 捕食者フラグ
  },
];

const DEFAULT_TUNING_SETTINGS = {
  threatDecay: 1.0, // 脅威減衰速度（1/sec）
  maxEscapeWeight: 0.7, // 逃避方向の最大割合（0〜1）
  baseEscapeStrength: 5.0, // 逃避舵取り強度（目標速度へ寄せる強さ）
  fastAttractStrength: 2.0, // 近傍不足時の補助凝集強度（0で無効）
  schoolPullCoefficient: 0.00003, // 大クラスタ引力係数
};
  
// 調整スライダーの説明（ユーザ目線）。ホバー時に title として表示する。
// NOTE: 実装の内部用語ではなく「何がどう変わるか」を短く書く。
const tuningHelp = {
  threatDecay: '脅威（捕食者などの危険度）が時間でどれだけ早く消えるか。大きいほど早く落ち着きます。',
  maxEscapeWeight: '逃避行動をどれだけ優先するか（0〜1）。1 に近いほど、危険時はほぼ逃げが優先されます。',
  baseEscapeStrength: '逃避の舵取り強度（目標速度へ寄せる強さ）。大きいほど素早く逃げ方向へ乗ります。',
  fastAttractStrength: '近くの仲間が少ないときに、群れへ戻す補助の凝集強度（0で無効）。',
  schoolPullCoefficient: '大きな群れ（大クラスタ）へ引き寄せる強さ。大きいほど大群にまとまりやすいです。',
};

// デバッグ表示/負荷設定の説明（ユーザ目線）。
const debugHelp = {
  enableFogPipeline: '見た目（フォグ/SSAO/ブルーム）を有効にします。重い場合は OFF。',
  enableShadows: '影描画を有効にします。見た目は良くなりますが負荷が上がりやすいです。',
  enableTailAnimation: '尾びれのアニメーションを有効にします。負荷が気になるなら OFF。',
  showSpeciesEnvelopes: '各種族の分布（中心/半径）を可視化します。',
  showSpeciesClusters: 'クラスタ検出結果を可視化します。',
  showSpeciesSchoolClusters: '大クラスタ（大きな群れ）の結果を可視化します。',
  showWorldAxisGrid: 'ワールド座標のグリッド/軸（目盛り）を表示します。空間スケール確認用。',
};

const flockStore = createFlockSettingsStore(
  DEFAULT_SETTINGS,
  DEFAULT_TUNING_SETTINGS
);
const {
  settings,
  systemSettings,
  assignSystemSettings: syncSystemSettings,
  totalBoids,
  replaceSettings,
  resetToDefaults,
  addSpecies: addSpeciesFromStore,
  removeSpecies: removeSpeciesFromStore,
  saveToStorage,
} = flockStore;

const tuningInitialized = ref(false);

// シミュレーション調整値を正規化し、欠損時はデフォルトで補完する。
function sanitizeTuningParams(raw = {}) {
  const sanitized = {};
  for (const [key, fallback] of Object.entries(DEFAULT_TUNING_SETTINGS)) {
    const value = Number(raw[key]);
    sanitized[key] = Number.isFinite(value) ? value : fallback;
  }
  return sanitized;
}

// reactive な systemSettings へ値を反映し、そのまま wasm へ渡せるプレーンオブジェクトを返す。
function updateSystemSettings(newValues) {
  const payload = sanitizeTuningParams(newValues);
  syncSystemSettings(payload);
  return payload;
}

// wasm 側へ現在のシミュレーション調整パラメータを送信する。
function applySystemSettingsToWasm() {
  if (!wasmBridge) {
    return;
  }
  wasmBridge.applyTuningParams(sanitizeTuningParams(systemSettings));
}

// 設定配列のディープコピーを作成（再初期化時の状態復元用）
function snapshotSettingsList(list) {
  return list.map((item) => JSON.parse(JSON.stringify(toRaw(item))));
}

let cachedTotalBoidCount = totalBoids.value;
let lastSpeciesSignature = getSpeciesSignature(settings);
let previousSettingsSnapshot = snapshotSettingsList(settings);
let pendingStateForReinitialize = null; // 再初期化待ちの状態スナップショット
let pendingSettingsSnapshot = null; // 再初期化待ちの設定スナップショット

// 外部スナップショットで設定を上書きし、localStorage に保存
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

// Three.js レンダリング用の DOM 参照と主要オブジェクト
const threeContainer = ref(null);
const backgroundAudio = ref(null);
let scene, camera, renderer, controls;
let fogPipeline = null; // 深度フォグパイプライン
let particleField = null; // 背景パーティクルフィールド
let dirLight = null; // ディレクショナルライトの参照
let groundMesh = null; // 地面メッシュの参照

/*
  起動直後（ユーザーがカメラ操作する前）は、群れ全体が画面中央に来るように
  「クラスタ（中心座標）」へ OrbitControls の target を滑らかに合わせる。
  - ユーザーが操作を開始したら即座に無効化する（意図しない自動追従を避ける）。
  - cluster は WASM 側の集計で得られるため、個体数に対して軽量。
  - 追従中だけ clusterData を取得し、不要な負荷を避ける。
*/
const startupCameraLookAt = {
  active: true,
  userInteracted: false,
  controlsHooked: false,
  startedAtMs: 0,
  maxDurationMs: 6000,
  smoothingSpeed: 4.0, // 大きいほど速く追従（指数平滑）
};

// GC を避けるため、Vector3 は使い回す。
const startupClusterTarget = new THREE.Vector3(0, 0, 0);
const startupClusterTargetScratch = new THREE.Vector3(0, 0, 0);

const paused = ref(false);

// タブ/ウィンドウが非アクティブのときは、背景音を自動でミュートする。
// - ユーザーが別タブを見ている間に音が鳴り続けるのを防ぐ。
// - 停止(pause)ではなくミュートで対応し、復帰時は直前の音量/ミュート状態へ戻す。
const tabAudioAutoMuteState = {
  active: false,
  previousMuted: false,
  previousVolume: 0.1,
};

function shouldAutoMuteByTabState() {
  if (typeof document === "undefined") {
    return false;
  }
  const hidden = Boolean(document.hidden);
  const unfocused =
    typeof document.hasFocus === "function" ? !document.hasFocus() : false;
  return hidden || unfocused;
}

function applyBackgroundAudioAutoMute() {
  const audioEl = backgroundAudio.value;
  if (!audioEl) {
    return;
  }

  const shouldMute = shouldAutoMuteByTabState();

  if (shouldMute) {
    // 既にミュート/無音なら、ユーザー側の状態として尊重して上書きしない。
    if (audioEl.muted || !(audioEl.volume > 0)) {
      return;
    }
    if (!tabAudioAutoMuteState.active) {
      tabAudioAutoMuteState.previousMuted = audioEl.muted;
      tabAudioAutoMuteState.previousVolume = audioEl.volume;
      tabAudioAutoMuteState.active = true;
    }
    audioEl.muted = true;
    return;
  }

  // 復帰時は「自動ミュートした分だけ」元へ戻す。
  if (tabAudioAutoMuteState.active) {
    audioEl.muted = tabAudioAutoMuteState.previousMuted;
    audioEl.volume = tabAudioAutoMuteState.previousVolume;
    tabAudioAutoMuteState.active = false;
  }
}

// デバッグHUD（Species envelope の中心/半径を画面表示する）
const speciesEnvelopeHudText = ref("");

// デバッグ用 Unit 可視化フラグ
const showUnits = ref(true);
const showUnitSpheres = ref(false);
const showUnitLines = ref(false);
const showUnitColors = ref(false);
const showSpeciesEnvelopes = ref(false);
const showSpeciesClusters = ref(false);
const showSpeciesSchoolClusters = ref(false);

// Blenderのビューポートのように、座標系の目盛り（グリッド/軸）を表示するデバッグトグル。
// - 画面上の数値HUDではなく、3D空間に基準を置くことでスケール感を掴みやすくする。
// - 描画負荷が極小のため、ON/OFFで即時に切り替える。
const showWorldAxisGrid = ref(false);
const unitLayer = ref(1);

let unitSpheres = []; // デバッグ用スフィアメッシュ
let unitLines = []; // デバッグ用ラインメッシュ
let envelopeSpheres = []; // 種族エンベロープ可視化用メッシュ
let envelopeGeometry = null;

// ワールド座標の目盛り（グリッド/軸）
let worldGridHelper = null;
let worldAxesHelper = null;

// Species clusters デバッグ可視化（クラスター中心に球を置く）
let clusterMesh = null;
let clusterGeometry = null;
let clusterMaterial = null;
let clusterMaxInstances = 0;

// Species school clusters（大クラスター/群れ）デバッグ可視化
// 種族エンベロープ表示と同じ見た目（ワイヤーフレーム/半透明）に揃える。
let schoolClusterMesh = null;
let schoolClusterGeometry = null;
let schoolClusterMaterial = null;
let schoolClusterMaxInstances = 0;

// InstancedMesh 更新用の一時オブジェクト（GC削減）
const clusterDummy = new THREE.Object3D();
const clusterColor = new THREE.Color();

// InstancedMesh 更新用の一時オブジェクト（GC削減）
const schoolClusterDummy = new THREE.Object3D();
const schoolClusterColor = new THREE.Color();

// GPU 負荷計測用に主要機能を切り替えるデバッグトグル群
const debugControls = reactive({
  enableFogPipeline: !deviceProfile.isMobile,
  enableShadows: true,
  enableTailAnimation: true,
});

/**
 * Blender風の「座標の目盛り」を3D空間に表示する。
 * - GridHelper: XZ平面に格子を描く（距離感/中心の把握用）
 * - AxesHelper: XYZ軸を描く（向きの把握用）
 */
function applyWorldAxisGridState() {
  if (!scene) {
    return;
  }

  const enabled = showWorldAxisGrid.value;

  if (enabled) {
    // 既に生成済みなら再利用してGCを避ける。
    if (!worldGridHelper) {
      // size=200, divisions=200 -> 1ユニット刻みのグリッド。
      // デバッグ用途なので色は控えめな明度にする。
      worldGridHelper = new THREE.GridHelper(
        200,
        200,
        toHex(OCEAN_COLORS.AMBIENT_LIGHT),
        toHex(OCEAN_COLORS.BOTTOM_LIGHT)
      );
    }
    if (!worldAxesHelper) {
      // 軸の長さは過剰に大きくしない（画面を占有しやすい）
      worldAxesHelper = new THREE.AxesHelper(12);
    }

    if (!worldGridHelper.parent) {
      scene.add(worldGridHelper);
    }
    if (!worldAxesHelper.parent) {
      scene.add(worldAxesHelper);
    }
  } else {
    // removeのみ行い、オブジェクトは保持して次回ON時に再利用する。
    if (worldGridHelper?.parent) {
      scene.remove(worldGridHelper);
    }
    if (worldAxesHelper?.parent) {
      scene.remove(worldAxesHelper);
    }
  }
}

let maxDepth = 1;
let stats = null; // stats-gl パフォーマンス表示
let glContext = null;
let frameCounter = 0;
let flockReinitTimer = null; // 群れ再初期化の遅延タイマー

let animationTimer = null;
// requestAnimationFrame で vsync に同期して更新する。
// setTimeout の高頻度ループは余計なスケジューリング負荷を生みやすい。
const COUNT_REINIT_DELAY_MS = 400; // 個体数変更後の再初期化待機時間

function applyRendererPixelRatio() {
  if (!renderer || typeof window === "undefined") {
    return;
  }
  const dpr = window.devicePixelRatio || 1;
  renderer.setPixelRatio(Math.min(dpr, MAX_RENDER_PIXEL_RATIO));
}

function positionStatsOverlay(element) {
  if (!element) return;
  element.style.position = "fixed";
  element.style.top = "0px";
  element.style.right = "0px";
  element.style.left = "auto";
  element.style.bottom = "auto";
  element.style.zIndex = "9999";
  element.style.width = "270px";
  element.style.height = "48px";
  element.style.pointerEvents = "auto";
  element.style.transform = "none";
}

/**
 * Species envelope（中心座標+半径）を HUD 文字列として整形する。
 * - ログではなく画面に出す用途。
 * - 必要最小限の情報（xyz と半径）だけ表示する。
 */
function updateSpeciesEnvelopeHud(envelopeData) {
  const buffer = envelopeData?.buffer;
  const floatCount = buffer?.length ?? 0;
  const envelopeCount = Math.floor(floatCount / 5);

  if (!buffer || envelopeCount <= 0) {
    speciesEnvelopeHudText.value = "";
    return;
  }

  // 表示は軽量な文字列組み立てに留める。
  // env[i] : center=(x,y,z) radius=r count=n
  const lines = [];
  for (let i = 0; i < envelopeCount; i += 1) {
    const base = i * 5;
    const cx = buffer[base];
    const cy = buffer[base + 1];
    const cz = buffer[base + 2];
    const radius = buffer[base + 3];
    const population = buffer[base + 4];

    // 半径が0のエンベロープは未確定扱いなので省略しても良いが、
    // ここでは「有効なものだけ」表示し、情報量を抑える。
    if (!(radius > 0.0001) || !(population > 0.0)) {
      continue;
    }

    lines.push(
      `env[${i}] center=(${cx.toFixed(2)}, ${cy.toFixed(2)}, ${cz.toFixed(
        2
      )}) radius=${radius.toFixed(2)} count=${Math.floor(population)}`
    );
  }

  speciesEnvelopeHudText.value = lines.join("\n");
}

// ツリーの最大深さを計算
function calcMaxDepth(unit, depth = 0) {
  if (
    !unit ||
    !unit.children ||
    typeof unit.children.size !== "function" ||
    unit.children.size() === 0
  ) {
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
  if (e.code === "Space") {
    paused.value = !paused.value;
  }
}

function applyTailAnimationDebugState() {
  const enabled = debugControls.enableTailAnimation ? 1 : 0;
  if (tailAnimation?.uniforms?.uTailEnable) {
    tailAnimation.uniforms.uTailEnable.value = enabled;
  }
}

function applyShadowDebugState() {
  const enabled = debugControls.enableShadows;
  if (renderer) {
    renderer.shadowMap.enabled = enabled;
  }
  if (dirLight) {
    dirLight.castShadow = enabled;
    if (dirLight.shadow) {
      dirLight.shadow.autoUpdate = enabled;
      // ON に戻した直後に「前回の shadow map のまま」にならないよう、更新要求を明示する。
      // （autoUpdate が OFF の間は shadow map が更新されないため）
      dirLight.shadow.needsUpdate = true;
    }
  }
  if (groundMesh) {
    groundMesh.receiveShadow = enabled;
  }
  if (instancedMeshHigh) {
    instancedMeshHigh.castShadow = enabled;
    instancedMeshHigh.receiveShadow = enabled;
  }
  if (instancedMeshLow) {
    instancedMeshLow.castShadow = enabled;
    instancedMeshLow.receiveShadow = enabled;
  }
  const predatorMeshes =
    typeof boidInstancing.getPredatorMeshes === "function"
      ? boidInstancing.getPredatorMeshes()
      : [];
  for (const mesh of predatorMeshes) {
    if (!mesh) {
      continue;
    }
    mesh.traverse?.((child) => {
      if (child.isMesh) {
        child.castShadow = enabled;
        child.receiveShadow = enabled;
      }
    });
  }
}

function rebuildFogPipeline() {
  if (!renderer || !scene || !camera) {
    if (!debugControls.enableFogPipeline && fogPipeline) {
      fogPipeline.dispose();
      fogPipeline = null;
    }
    return;
  }

  const supportsPostProcess = renderer.capabilities?.isWebGL2;
  const shouldEnable = Boolean(
    debugControls.enableFogPipeline && supportsPostProcess
  );

  if (shouldEnable) {
    if (!fogPipeline) {
      fogPipeline = new FogPipeline(heightFogConfig);
    }
    const size = new THREE.Vector2();
    renderer.getSize(size);
    fogPipeline.init(renderer, scene, camera, size.x, size.y);
  } else if (fogPipeline) {
    fogPipeline.dispose();
    fogPipeline = null;
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
  // トーンマッピングを有効化し水中ライティングのダイナミクスを保つ。
  renderer.toneMapping = THREE.CineonToneMapping;
  renderer.toneMappingExposure = 0.8;
  applyRendererPixelRatio();
  renderer.setSize(width, height);
  renderer.shadowMap.enabled = debugControls.enableShadows;
  renderer.shadowMap.type = THREE.PCFShadowMap; // 影を柔らかく

  glContext = renderer.getContext();

  threeContainer.value.appendChild(renderer.domElement);

  camera.aspect = width / height;
  camera.updateProjectionMatrix();

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; // なめらかな操作
  controls.dampingFactor = 0.1;

  // 起動直後だけ、クラスタ中心へ注視点を滑らかに合わせる。
  // OrbitControls の start イベントでユーザー操作開始を検出できる。
  startupCameraLookAt.active = true;
  startupCameraLookAt.userInteracted = false;
  startupCameraLookAt.startedAtMs = performance.now();
  if (!startupCameraLookAt.controlsHooked) {
    startupCameraLookAt.controlsHooked = true;
    controls.addEventListener("start", () => {
      startupCameraLookAt.userInteracted = true;
    });
  }

  // 地面メッシュ追加
  const groundGeo = new THREE.PlaneGeometry(300, 300);
  const groundMat = createFadeOutGroundMaterial();
  groundMat.depthTest = true;
  groundMesh = new THREE.Mesh(groundGeo, groundMat);
  groundMesh.rotation.x = -Math.PI / 2;
  groundMesh.position.y = -10;
  groundMesh.receiveShadow = true; // 影を受ける
  scene.add(groundMesh);

  // ライト
  const ambientLight = new THREE.AmbientLight(
    toHex(OCEAN_COLORS.AMBIENT_LIGHT),
    0.9
  );
  scene.add(ambientLight);

  // 太陽光（やや暖色のDirectionalLight）
  dirLight = new THREE.DirectionalLight(toHex(OCEAN_COLORS.SUN_LIGHT), 25); // 暖色＆強め
  dirLight.position.set(300, 500, 200); // 高い位置から照らす
  dirLight.castShadow = true;

  // 影カメラの範囲を広げる
  dirLight.shadow.camera.left = -100;
  dirLight.shadow.camera.right = 100;
  dirLight.shadow.camera.top = 100;
  dirLight.shadow.camera.bottom = -100;
  dirLight.shadow.camera.near = 1;
  // NOTE:
  // DirectionalLight の shadow camera は光源位置を基準に depth 範囲を切る。
  // ここが短すぎると「シーン全体が shadow map の far でクリップ」され、
  // 影が常に OFF のように見える。
  // （光源が (300,500,200) のため far=400 だと原点近傍まで届かない）
  dirLight.shadow.camera.far = 1200;
  dirLight.shadow.camera.updateProjectionMatrix();

  dirLight.shadow.mapSize.width = 768;
  dirLight.shadow.mapSize.height = 768;
  dirLight.shadow.bias = -0.01;
  dirLight.shadow.normalBias = 0.01;

  scene.add(dirLight);
  initParticleSystem();
  rebuildFogPipeline();
  applyShadowDebugState();
  applyTailAnimationDebugState();
  applyWorldAxisGridState();
  // ウィンドウリサイズ対応
  window.addEventListener("resize", onWindowResize);
}

/**
 * species clusters バッファ（6 float/1 cluster）から、全体の注視点を計算する。
 * バッファ形式は描画と同じ: (speciesId, cx, cy, cz, radius, weight)
 * - weight を重みとして中心を加重平均する。
 * - 有効な cluster（weight>0）のみを採用する。
 */
function computeSpeciesClusterWeightedCenter(clusterData, outCenter) {
  const buffer = clusterData?.buffer;
  const floatCount = buffer?.length ?? 0;
  const clusterCount = Math.floor(floatCount / 6);

  if (!buffer || clusterCount <= 0) {
    return false;
  }

  let sumX = 0;
  let sumY = 0;
  let sumZ = 0;
  let sumW = 0;

  for (let i = 0; i < clusterCount; i += 1) {
    const base = i * 6;
    const cx = buffer[base + 1];
    const cy = buffer[base + 2];
    const cz = buffer[base + 3];
    const weight = Math.max(0, buffer[base + 5] || 0);

    if (!(weight > 0.0)) {
      continue;
    }

    // weight は大きくなりうるため、念のため上限を設けて暴走を避ける。
    const clamped = Math.min(weight, 50000);
    sumX += cx * clamped;
    sumY += cy * clamped;
    sumZ += cz * clamped;
    sumW += clamped;
  }

  if (!(sumW > 0)) {
    return false;
  }

  outCenter.set(sumX / sumW, sumY / sumW, sumZ / sumW);
  return true;
}

function shouldAutoLookAtClusterCenter() {
  if (!startupCameraLookAt.active || startupCameraLookAt.userInteracted) {
    return false;
  }
  if (!camera || !controls) {
    return false;
  }
  const elapsed = performance.now() - startupCameraLookAt.startedAtMs;
  return elapsed >= 0 && elapsed <= startupCameraLookAt.maxDurationMs;
}

function updateStartupCameraLookAt(clusterData, deltaTime) {
  if (!shouldAutoLookAtClusterCenter()) {
    return;
  }

  // deltaTime が極端に大きいフレームでは平滑係数が跳ねるので、上限を付ける。
  const safeDeltaTime = Math.min(Math.max(deltaTime, 0), 0.1);
  const smoothing = 1.0 - Math.exp(-startupCameraLookAt.smoothingSpeed * safeDeltaTime);

  const hasTarget = computeSpeciesClusterWeightedCenter(
    clusterData,
    startupClusterTargetScratch
  );

  if (hasTarget) {
    startupClusterTarget.copy(startupClusterTargetScratch);
  }

  // まだ有効な cluster が無い場合は、最後のターゲット（初期は原点）へ寄せる。
  controls.target.lerp(startupClusterTarget, smoothing);
}

function onWindowResize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  applyRendererPixelRatio();
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

const TRIPLE_BUFFER_SIZE = 3; // BoidInstancing のトリプルバッファ数
const HIDDEN_POSITION = 1e6; // 非表示インスタンスを退避させる座標値
const IDENTITY_QUATERNION = [0, 0, 0, 1]; // 非表示インスタンスに適用する無回転クォータニオン
const SIN_LUT_SIZE = 256; // 尾びれアニメ用サイン LUT サイズ
const sinCosLutTexture = createSinCosLutTexture(SIN_LUT_SIZE);
// LOD距離閾値（平方距離）: 近距離はハイポリ、中距離はLOD+アニメ、遠距離はLOD静止
const LOD_DISTANCE_PRESET = deviceProfile.isMobile
  ? { nearSq: 1.5, midSq: 9 } // モバイルは近距離のみハイLODにして描画負荷を抑える
  : { nearSq: 4, midSq: 25 }; // PC は従来値
const LOD_NEAR_DISTANCE_SQ = LOD_DISTANCE_PRESET.nearSq;
const LOD_MID_DISTANCE_SQ = LOD_DISTANCE_PRESET.midSq;
const tailAnimation = {
  uniforms: {
    uTailTime: { value: 0 }, // 時間（波形生成用）
    uTailAmplitude: { value: 0.14 }, // 振幅（全身の揺れ幅）
    uTailFrequency: { value: 10.0 }, // 周波数（くねり速度）
    uTailPhaseStride: { value: 5.0 }, // 体の長さ方向の位相差（波長に相当）
    uTailTurnStrength: { value: 0.1 }, // 旋回時の強度
    uTailSpeedScale: { value: 1 }, // 速度による影響度
    uTailRight: { value: new THREE.Vector3(1, 0, 0) }, // 尾アニメの右方向ベクトル
    uTailForward: { value: new THREE.Vector3(0, 0, 1) }, // 尾アニメの進行方向ベクトル
    uTailUp: { value: new THREE.Vector3(0, 1, 0) }, // 尾アニメの上方向ベクトル
    uTailEnable: { value: 1 }, // アニメーション有効/無効
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
  SKY_HIGHLIGHT: "#4fbaff",
  SKY_BLUE: "#15a1ff",
  DEEP_BLUE: "#002968",
  SEAFLOOR: "#777465",
  FOG: "#153a6c",
  AMBIENT_LIGHT: "#59a5eb",
  SUN_LIGHT: "#5389b7",
  SIDE_LIGHT1: "#6ba3d0",
  SIDE_LIGHT2: "#2d5f7a",
  BOTTOM_LIGHT: "#0f2635",
};

// '#rrggbb' 形式の色を three.js の整数表現に変換
const toHex = (colorStr) => parseInt(colorStr.replace("#", "0x"), 16);

// 距離と深度で濃さが変わる海中フォグ設定
const heightFogConfig = {
  color: new THREE.Color(OCEAN_COLORS.DEEP_BLUE), // 遠景で溶け込む深海色
  distanceStart: 4.0, // カメラからこの距離まではフォグゼロ
  distanceEnd: 60.0, // この距離でフォグが最大になる
  distanceExponent: 0.4, // 距離カーブの滑らかさ
  distanceControlPoint1: new THREE.Vector2(0.2, 0.8), // 距離ベジェ曲線の制御点（開始側）
  distanceControlPoint2: new THREE.Vector2(0.75, 0.95), // 距離ベジェ曲線の制御点（終端側）
  surfaceLevel: 100.0, // 水面の高さ。ここから下がるほど暗くなる
  heightFalloff: 0.01, // 深度による減衰率
  heightExponent: 1, // 深度カーブの強さ
  maxOpacity: 1.2, // 最大フォグ率
};

function createOceanSphere() {
  if (!scene) return null;

  // 上層→深層のグラデーションで海中の空気感を演出
  const canvas = document.createElement("canvas");
  const context = canvas.getContext("2d");
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
  const alphaMap = textureLoader.load("./models/groundAlfa.png");

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
    particleField = new ParticleField(useLowSpecPreset);
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
    if (playResult && typeof playResult.then === "function") {
      playResult.catch(() => {
        const resume = () => {
          document.removeEventListener("pointerdown", resume);
          document.removeEventListener("keydown", resume);
          audioEl.play().catch(() => {
            /* ignored */
          });
        };
        document.addEventListener("pointerdown", resume, { once: true });
        document.addEventListener("keydown", resume, { once: true });
      });
    }
  };

  tryPlay();

  // 初期状態が「タブ非選択」の場合にも音が出ないようにする。
  applyBackgroundAudioAutoMute();
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

// 現在の群れ状態（位置・速度・向き）をスナップショット
function captureFlockState() {
  return wasmBridge?.captureState() ?? null;
}

// 過去の群れ状態を復元（個体数変更時などに種ごとに引き継ぐ）
function restoreFlockState(previousState, oldSettings, newSettings) {
  wasmBridge?.restoreState(previousState, oldSettings, newSettings);
}

// 3D モデルとマテリアルの保持用変数
let boidModel = null;
let boidModelLod = null;
let originalMaterial = null;
let originalMaterialLod = null;
let predatorModel = null;
let predatorMaterial = null;

// 前回の Unit 色分け状態を保持（OFF→ON 検知用）
let lastShowUnitColors = false;

// 捕食者の総数を計算
function getPredatorCount() {
  return settings.reduce(
    (sum, s) => sum + (s.isPredator && s.count ? s.count : 0),
    0
  );
}

// 全 Boid の総数を取得
function getTotalBoidCount() {
  return totalBoids.value;
}

// 種族構成の署名文字列を生成（個体数変更検知用）
function getSpeciesSignature(specList = settings) {
  if (!Array.isArray(specList)) {
    return "";
  }
  return specList
    .map(
      (s, index) =>
        `${index}:${(s && s.count) || 0}:${s && s.isPredator ? 1 : 0}`
    )
    .join("|");
}

// 群れを再初期化（個体数・種族変更時に即座実行）
function reinitializeFlockNow() {
  if (!wasmModule || !wasmBridge) return;

  const pendingState = pendingStateForReinitialize?.state || null;
  const oldSettingsRef =
    pendingStateForReinitialize?.oldSettings || previousSettingsSnapshot;
  const newSettingsRef =
    pendingSettingsSnapshot || snapshotSettingsList(settings);
  const targetSignature = getSpeciesSignature(newSettingsRef);

  try {
    // 初期化手順は Bridge に集約
    wasmBridge.initializeFlock(newSettingsRef, {
      spatialScale: 1,
      posRange: 4,
      velRange: 0.25,
      groundPlane: {
        enabled: true,
        height: groundMesh?.position?.y ?? -10,
        blendDistance: 1,
        stiffness: 22,
        damping: 10,
      },
    });
  } catch (error) {
    console.error("群れの再初期化に失敗しました", error);
    applySettingsSnapshot(previousSettingsSnapshot);
    pendingStateForReinitialize = null;
    pendingSettingsSnapshot = null;
    return;
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

// 群れ再初期化を遅延実行（設定変更の連打を吸収）
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

// インスタンシングメッシュを初期化（個体数に応じてバッファを確保）
function initInstancedBoids(count) {
  if (
    !scene ||
    !boidModel ||
    !boidModelLod ||
    !originalMaterial ||
    !originalMaterialLod
  ) {
    console.warn("initInstancedBoids: required assets are not ready");
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
    console.error("Failed to initialize boid instancing");
    return;
  }

  const meshes = boidInstancing.getMeshes();
  instancedMeshHigh = meshes.high;
  instancedMeshLow = meshes.low;

  // 捕食者メッシュも最新設定に合わせて生成しておく
  const predatorCount = getPredatorCount();
  if (typeof boidInstancing.ensurePredatorMeshes === "function") {
    boidInstancing.ensurePredatorMeshes(predatorCount);
  }

  applyShadowDebugState();

  if (instancedMeshHigh?.instanceColor) {
    instancedMeshHigh.instanceColor.needsUpdate = true;
  }
  if (instancedMeshLow?.instanceColor) {
    instancedMeshLow.instanceColor.needsUpdate = true;
  }
}

function loadBoidModel(callback) {
  const loader = new GLTFLoader();
  const basePath = process.env.BASE_URL || "/"; // publicPath を取得
  const textureLoader = new THREE.TextureLoader();
  let pendingAssets = 3;
  const notifyReady = () => {
    pendingAssets = Math.max(0, pendingAssets - 1);
    if (pendingAssets === 0) {
      callback();
    }
  };
  const texture = textureLoader.load(
    "./models/fish.png", // テクスチャのパス
    () => {
      console.log("Texture loaded successfully.");
    },
    undefined,
    (error) => {
      console.error("An error occurred while loading the texture:", error);
    }
  );
  const textureLod = textureLoader.load(
    "./models/fish_lod.png", // テクスチャのパス
    () => {
      console.log("Texture loaded successfully.");
    },
    undefined,
    (error) => {
      console.error("An error occurred while loading the texture:", error);
    }
  );
  texture.flipY = false;
  texture.colorSpace = THREE.SRGBColorSpace; // sRGBカラー空間を使用
  textureLod.flipY = false;
  textureLod.colorSpace = THREE.SRGBColorSpace;

  const predatorTexture = textureLoader.load(
    "./models/fishPredetor.png",
    () => {
      console.log("Predator texture loaded successfully.");
    },
    undefined,
    (error) => {
      console.error(
        "An error occurred while loading the predator texture:",
        error
      );
    }
  );
  predatorTexture.flipY = false;
  predatorTexture.colorSpace = THREE.SRGBColorSpace;

  let boidMaterial = new THREE.MeshStandardMaterial({
    color: 0xffffff, // 白色
    roughness: 0.3,
    metalness: 0.3,
    transparent: false, // 半透明を有効化
    alphaTest: 0.5, // アルファテストを設定
    map: texture, // テクスチャを設定
    vertexColors: false, // 通常時は無効
    vertexColor: 0xffffff,
  });

  let boidLodMaterial = new THREE.MeshStandardMaterial({
    color: 0xffffff, // 白色
    roughness: 0.5,
    metalness: 0.2,
    transparent: false, // 半透明を有効化
    alphaTest: 0.5, // アルファテストを設定
    map: textureLod, // テクスチャを設定
    vertexColors: false, // 通常時は無効
    vertexColor: 0xffffff,
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
      console.error("An error occurred while loading the model:", error);
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
      console.error("An error occurred while loading the LOD model:", error);
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
      console.error(
        "An error occurred while loading the predator model:",
        error
      );
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

function clearSpeciesEnvelopeVisuals() {
  for (const mesh of envelopeSpheres) scene.remove(mesh);
  envelopeSpheres = [];
}

function clearClusterVisuals(disposeSharedResources = true) {
  if (clusterMesh) {
    scene.remove(clusterMesh);
    // InstancedMesh 自体の破棄。geometry/material は共有しているので、
    // disposeSharedResources=true の時だけ共有リソースを破棄する。
    clusterMesh = null;
    clusterMaxInstances = 0;
  }
  if (disposeSharedResources) {
    clusterGeometry?.dispose?.();
    clusterMaterial?.dispose?.();
    clusterGeometry = null;
    clusterMaterial = null;
  }
}

function clearSchoolClusterVisuals(disposeSharedResources = true) {
  if (schoolClusterMesh) {
    scene.remove(schoolClusterMesh);
    // InstancedMesh 自体の破棄。geometry/material は共有しているので、
    // disposeSharedResources=true の時だけ共有リソースを破棄する。
    schoolClusterMesh = null;
    schoolClusterMaxInstances = 0;
  }

  if (disposeSharedResources) {
    schoolClusterGeometry?.dispose?.();
    schoolClusterMaterial?.dispose?.();
    schoolClusterGeometry = null;
    schoolClusterMaterial = null;
  }
}

// レイヤ制限付き再帰描画
function drawUnitTree(unit, layer = 0) {
  // スフィア: スライダで制御
  if (
    layer >= maxDepth - unitLayer.value + 1 &&
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
  if (
    showUnitLines.value &&
    unit.children &&
    typeof unit.children.size === "function" &&
    unit.children.size() > 0
  ) {
    for (let i = 0; i < unit.children.size(); i++) {
      const child = unit.children.get(i);
      let line;
      if (unitLines.length > 0) {
        line = unitLines.pop(); // 再利用
        line.visible = true;
      } else {
        const lineGeometry = new THREE.BufferGeometry();
        const lineMaterial = new THREE.LineBasicMaterial({
          color: new THREE.Color().setHSL(
            0.35,
            1,
            0.7 - 0.4 * (layer / maxDepth)
          ),
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
  if (
    unit.children &&
    typeof unit.children.size === "function" &&
    unit.children.size() > 0
  ) {
    for (let i = 0; i < unit.children.size(); i++) {
      const child = unit.children.get(i);
      drawUnitTree(child, layer + 1);
    }
  }
}

function renderSpeciesEnvelopes(envelopeData) {
  if (!showSpeciesEnvelopes.value) {
    clearSpeciesEnvelopeVisuals();
    return;
  }

  const buffer = envelopeData?.buffer;
  const floatCount = buffer?.length ?? 0;
  const envelopeCount = Math.floor(floatCount / 5);

  if (!buffer || envelopeCount <= 0) {
    clearSpeciesEnvelopeVisuals();
    return;
  }

  if (!envelopeGeometry) {
    envelopeGeometry = new THREE.SphereGeometry(1, 24, 18);
  }

  const hueSpan = 0.65;
  for (let i = 0; i < envelopeCount; i += 1) {
    const base = i * 5;
    const radius = buffer[base + 3];
    const population = buffer[base + 4];
    if (radius <= 0.0001 || population <= 0.0) {
      if (envelopeSpheres[i]) {
        envelopeSpheres[i].visible = false;
      }
      continue;
    }

    let mesh = envelopeSpheres[i];
    if (!mesh) {
      const material = new THREE.MeshBasicMaterial({
        color: new THREE.Color().setHSL(0.6, 0.8, 0.45),
        wireframe: true,
        transparent: true,
        opacity: 0.3,
        depthWrite: false,
      });
      mesh = new THREE.Mesh(envelopeGeometry, material);
      scene.add(mesh);
      envelopeSpheres[i] = mesh;
    }

    mesh.visible = true;
    mesh.position.set(buffer[base], buffer[base + 1], buffer[base + 2]);
    mesh.scale.set(radius, radius, radius);
    mesh.material.color.setHSL((i % 7) / 7 * hueSpan, 0.85, 0.5);
    mesh.material.opacity = 0.22 + Math.min(population / 1000, 1) * 0.25;
  }

  for (let i = envelopeCount; i < envelopeSpheres.length; i += 1) {
    if (envelopeSpheres[i]) {
      envelopeSpheres[i].visible = false;
    }
  }
}

/**
 * Species clusters の中心を球でデバッグ表示する。
 * - 1クラスター = 1インスタンス
 * - 半径は cluster.radius に比例
 * - speciesId で色分け
 */
function renderSpeciesClusters(clusterData) {
  if (!showSpeciesClusters.value) {
    clearClusterVisuals();
    return;
  }

  const buffer = clusterData?.buffer;
  const floatCount = buffer?.length ?? 0;
  const clusterCount = Math.floor(floatCount / 6);

  if (!buffer || clusterCount <= 0) {
    clearClusterVisuals();
    return;
  }

  if (!clusterGeometry) {
    // 種族エンベロープ表示と同じ見た目に揃える（ワイヤーフレームの密度も一致させる）。
    // InstancedMesh で共有されるので、生成は1回だけ。
    clusterGeometry = new THREE.SphereGeometry(1, 24, 18);
  }
  if (!clusterMaterial) {
    clusterMaterial = new THREE.MeshBasicMaterial({
      transparent: true,
      // 種族エンベロープと同じ “薄いワイヤーフレーム” の見た目に合わせる。
      opacity: 0.3,
      depthWrite: false,
      vertexColors: true,
      wireframe: true,
    });
  }

  // InstancedMesh は作り直しが高コストなので、capacity方式で必要なときだけ拡張する。
  if (!clusterMesh || clusterMaxInstances < clusterCount) {
    clearClusterVisuals(false);
    clusterMesh = new THREE.InstancedMesh(
      clusterGeometry,
      clusterMaterial,
      clusterCount
    );
    clusterMesh.frustumCulled = false;
    clusterMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    scene.add(clusterMesh);
    clusterMaxInstances = clusterCount;
  }

  // 実際に描画するインスタンス数だけ切り替える（clusterCount が減っても作り直さない）
  clusterMesh.count = clusterCount;

  const hueSpan = 0.65;
  // デバッグ表示の球は「クラスターの影響半径」をそのまま描くと大きく見えやすい。
  // 視認性と直感（周辺の近い個体が入る程度）を優先して表示倍率をかける。
  const radiusVisualScale = 0.5;
  const minRadius = 0.25;

  for (let i = 0; i < clusterCount; i += 1) {
    const base = i * 6;
    const speciesId = Math.max(0, Math.floor(buffer[base] || 0));
    const cx = buffer[base + 1];
    const cy = buffer[base + 2];
    const cz = buffer[base + 3];
    const radius = Math.max(minRadius, (buffer[base + 4] || 0) * radiusVisualScale);
    const weight = Math.max(0, buffer[base + 5] || 0);

    clusterDummy.position.set(cx, cy + 0.02, cz);
    clusterDummy.scale.setScalar(radius);
    clusterDummy.updateMatrix();
    clusterMesh.setMatrixAt(i, clusterDummy.matrix);

    // weight が大きいほど明るく（生きているクラスターが見やすい）
    const normalized = Math.min(weight / 5000, 1);
    const hue = (speciesId % 7) / 7 * hueSpan;
    clusterColor.setHSL(hue, 0.9, 0.18 + normalized * 0.62);
    clusterMesh.setColorAt(i, clusterColor);
  }

  clusterMesh.instanceMatrix.needsUpdate = true;
  if (clusterMesh.instanceColor) {
    clusterMesh.instanceColor.needsUpdate = true;
  }
}

/**
 * Species school clusters（大クラスター/群れ）の中心を球でデバッグ表示する。
 * - 1クラスター = 1インスタンス
 * - 半径は cluster.radius に比例
 * - 小クラスターと重なっても見やすいように wireframe で描く
 */
function renderSpeciesSchoolClusters(clusterData) {
  if (!showSpeciesSchoolClusters.value) {
    clearSchoolClusterVisuals();
    return;
  }

  const buffer = clusterData?.buffer;
  const floatCount = buffer?.length ?? 0;
  const clusterCount = Math.floor(floatCount / 6);

  if (!buffer || clusterCount <= 0) {
    clearSchoolClusterVisuals();
    return;
  }

  if (!schoolClusterGeometry) {
    // 種族エンベロープ表示と同じ見た目に揃える。
    schoolClusterGeometry = new THREE.SphereGeometry(1, 24, 18);
  }
  if (!schoolClusterMaterial) {
    schoolClusterMaterial = new THREE.MeshBasicMaterial({
      transparent: true,
      opacity: 0.3,
      depthWrite: false,
      vertexColors: true,
      wireframe: true,
    });
  }

  // InstancedMesh は作り直しが高コストなので、capacity方式で必要なときだけ拡張する。
  if (!schoolClusterMesh || schoolClusterMaxInstances < clusterCount) {
    clearSchoolClusterVisuals(false);
    schoolClusterMesh = new THREE.InstancedMesh(
      schoolClusterGeometry,
      schoolClusterMaterial,
      clusterCount
    );
    schoolClusterMesh.frustumCulled = false;
    schoolClusterMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    scene.add(schoolClusterMesh);
    schoolClusterMaxInstances = clusterCount;
  }

  // 実際に描画するインスタンス数だけ切り替える（clusterCount が減っても作り直さない）
  schoolClusterMesh.count = clusterCount;

  const hueSpan = 0.65;
  // 大クラスターは半径が大きくなりやすいので、視認性優先で少し縮めて表示する。
  const radiusVisualScale = 0.6;
  const minRadius = 0.6;

  for (let i = 0; i < clusterCount; i += 1) {
    const base = i * 6;
    const speciesId = Math.max(0, Math.floor(buffer[base] || 0));
    const cx = buffer[base + 1];
    const cy = buffer[base + 2];
    const cz = buffer[base + 3];
    const radius = Math.max(minRadius, (buffer[base + 4] || 0) * radiusVisualScale);
    const weight = Math.max(0, buffer[base + 5] || 0);

    schoolClusterDummy.position.set(cx, cy + 0.03, cz);
    schoolClusterDummy.scale.setScalar(radius);
    schoolClusterDummy.updateMatrix();
    schoolClusterMesh.setMatrixAt(i, schoolClusterDummy.matrix);

    // weight が大きいほど明るく（追跡が安定した群れが見やすい）
    const normalized = Math.min(weight / 12000, 1);
    const hue = (speciesId % 7) / 7 * hueSpan;
    schoolClusterColor.setHSL(hue, 0.85, 0.12 + normalized * 0.75);
    schoolClusterMesh.setColorAt(i, schoolClusterColor);
  }

  schoolClusterMesh.instanceMatrix.needsUpdate = true;
  if (schoolClusterMesh.instanceColor) {
    schoolClusterMesh.instanceColor.needsUpdate = true;
  }
}
let lastTime = performance.now(); // 前回のフレームのタイムスタンプ
let unitIdScratch = null;
let unitIdScratchSize = 0;
const unitColor = new THREE.Color();

function scheduleNextFrame() {
  if (typeof requestAnimationFrame === "function") {
    animationTimer = requestAnimationFrame(animate);
  }
}

function animate(frameTimeMs) {
  stats?.begin();
  const currentTime =
    typeof frameTimeMs === "number" ? frameTimeMs : performance.now();
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
    if ((frameCounter & 1) === 0) {
      stats?.update();
    }
    scheduleNextFrame();
    return;
  }

  const { positions, orientations, velocities } = getWasmViews(count);
  if ((frameCounter++ & 63) === 0) {
    wasmBridge?.getDiagnostics?.({ firstBoidX: true });
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

  const visibleCount =
    updateInfo.visibleCount ?? Math.max(0, count - predatorCount);

  if (
    showUnitColors.value &&
    instancedMeshHigh.instanceColor &&
    instancedMeshLow.instanceColor
  ) {
    const diagnostics = wasmBridge?.getDiagnostics?.({ boidCount: count, unitMappings: true, unitDensities: true }) ?? null;
    const unitMappings = diagnostics?.unitMappings ?? null;
    const highCount = updateInfo?.highCount ?? visibleCount;
    const lowCount = updateInfo?.lowCount ?? visibleCount;
    const highMap = updateInfo?.highInstanceToBoid ?? null;
    const lowMap = updateInfo?.lowInstanceToBoid ?? null;

    if (unitMappings) {
      if (unitIdScratchSize !== count) {
        unitIdScratch = new Int32Array(count);
        unitIdScratchSize = count;
      }
      unitIdScratch.fill(-1);
      for (let j = 0; j < unitMappings.length; j += 2) {
        const boidIndex = unitMappings[j];
        if (boidIndex >= 0 && boidIndex < unitIdScratchSize) {
          unitIdScratch[boidIndex] = unitMappings[j + 1];
        }
      }

      const unitDensities = diagnostics?.unitDensities ?? null;
      const densityCount = unitDensities?.length ?? 0;
      const densityScale = 0.18;
      const hueSpan = 0.45;
      const hueMax = 0.58;

      const applyDensityColor = Boolean(unitDensities && densityCount > 0);

      for (let inst = 0; inst < highCount; inst++) {
        const boidIndex = highMap ? highMap[inst] : inst;
        const unitId =
          boidIndex >= 0 && boidIndex < unitIdScratchSize
            ? unitIdScratch[boidIndex]
            : -1;

        if (applyDensityColor && unitId >= 0 && unitId < densityCount) {
          const density = unitDensities[unitId];
          const normalized = Math.min(Math.max(density * densityScale, 0), 1);
          const hue = hueMax - normalized * hueSpan;
          unitColor.setHSL(hue, 0.85, 0.55);
        } else if (unitId >= 0) {
          unitColor.setHSL((unitId % 100) / 100, 0.8, 0.6);
        } else {
          unitColor.setRGB(1, 0, 0);
        }

        instancedMeshHigh.setColorAt(inst, unitColor);
      }

      for (let inst = 0; inst < lowCount; inst++) {
        const boidIndex = lowMap ? lowMap[inst] : inst;
        const unitId =
          boidIndex >= 0 && boidIndex < unitIdScratchSize
            ? unitIdScratch[boidIndex]
            : -1;

        if (applyDensityColor && unitId >= 0 && unitId < densityCount) {
          const density = unitDensities[unitId];
          const normalized = Math.min(Math.max(density * densityScale, 0), 1);
          const hue = hueMax - normalized * hueSpan;
          unitColor.setHSL(hue, 0.85, 0.55);
        } else if (unitId >= 0) {
          unitColor.setHSL((unitId % 100) / 100, 0.8, 0.6);
        } else {
          unitColor.setRGB(1, 0, 0);
        }

        instancedMeshLow.setColorAt(inst, unitColor);
      }

      instancedMeshHigh.instanceColor.needsUpdate = true;
      instancedMeshLow.instanceColor.needsUpdate = true;
    }
  } else if (
    lastShowUnitColors &&
    instancedMeshHigh.instanceColor &&
    instancedMeshLow.instanceColor
  ) {
    const whiteColor = new THREE.Color(1, 1, 1);
    const highCount = updateInfo?.highCount ?? visibleCount;
    const lowCount = updateInfo?.lowCount ?? visibleCount;

    for (let i = 0; i < highCount; i++) {
      instancedMeshHigh.setColorAt(i, whiteColor);
    }
    for (let i = 0; i < lowCount; i++) {
      instancedMeshLow.setColorAt(i, whiteColor);
    }
    instancedMeshHigh.instanceColor.needsUpdate = true;
    instancedMeshLow.instanceColor.needsUpdate = true;
    console.log("✓ Reset vertex colors to white (OFF mode)");
  }

  lastShowUnitColors = showUnitColors.value;

  // species envelope は「表示ON」のときだけ取得する。
  let envelopeData = null;
  if (wasmBridge && showSpeciesEnvelopes.value) {
    envelopeData = wasmBridge.getDiagnostics?.({ speciesEnvelopes: true })?.speciesEnvelopes ?? null;
  }

  if (showSpeciesEnvelopes.value) {
    renderSpeciesEnvelopes(envelopeData);

    // HUD 更新も間引く（文字列更新を毎フレーム行わない）
    if ((frameCounter & 3) === 0) {
      updateSpeciesEnvelopeHud(envelopeData);
    }
  } else if (envelopeSpheres.length > 0) {
    clearSpeciesEnvelopeVisuals();
    speciesEnvelopeHudText.value = "";
  }

  // 起動直後だけ、クラスタ中心に注視点を滑らかに合わせる。
  let startupClusterData = null;
  if (wasmBridge && shouldAutoLookAtClusterCenter()) {
    startupClusterData = wasmBridge.getDiagnostics?.({ speciesClusters: true })?.speciesClusters ?? null;
  }
  updateStartupCameraLookAt(startupClusterData, deltaTime);

  // クラスター表示も更新頻度を間引く（中心推定の確認用途）
  if (showSpeciesClusters.value && wasmBridge) {
    if ((frameCounter & 3) === 0) {
      const clusterData = wasmBridge.getDiagnostics?.({ speciesClusters: true })?.speciesClusters ?? null;
      renderSpeciesClusters(clusterData);
    }
  } else if (clusterMesh) {
    clearClusterVisuals();
  }

  // 大クラスター（群れクラスタ）表示も更新頻度を間引く
  if (showSpeciesSchoolClusters.value && wasmBridge) {
    if ((frameCounter & 3) === 0) {
      const schoolClusterData = wasmBridge.getDiagnostics?.({ speciesSchoolClusters: true })?.speciesSchoolClusters ?? null;
      renderSpeciesSchoolClusters(schoolClusterData);
    }
  } else if (schoolClusterMesh) {
    clearSchoolClusterVisuals();
  }

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

  scheduleNextFrame();
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

// シミュレーションを開始（群れ初期化 + アニメーションループ起動）
function startSimulation() {
  reinitializeFlockNow();
  scheduleNextFrame();
}

onMounted(() => {
  // 初回マウント時にシステム調整値を WASM に反映
  if (!tuningInitialized.value) {
    updateSystemSettings(toRaw(systemSettings));
    tuningInitialized.value = true;
    applySystemSettingsToWasm();
    saveToStorage();
  }

  initThreeJS();
  loadBoidModel(() => {
    console.log("Boid model loaded successfully.");

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
        (typeof stats?.domElement !== "undefined" ? stats.domElement : null) ||
        (typeof stats?.getDom === "function" ? stats.getDom() : null) ||
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
      stats && typeof stats.init === "function"
        ? Promise.resolve(stats.init(statsInitTarget))
        : Promise.resolve();

    initPromise
      .then(() => {
        if (typeof stats?.patchThreeRenderer === "function") {
          stats.patchThreeRenderer(renderer);
        }
        ensureStatsOverlay();
      })
      .catch((error) => {
        console.error("Failed to initialize stats-gl:", error);
        ensureStatsOverlay();
      });

    startSimulation();
    initBackgroundAudioPlayback();
  });

  window.addEventListener("keydown", handleKeydown);

  // タブ/ウィンドウのアクティブ状態に応じて背景音を自動ミュートする。
  // visibilitychange: タブ切替/最小化など
  // blur/focus: アプリをアクティブにしているか
  document.addEventListener("visibilitychange", applyBackgroundAudioAutoMute);
  window.addEventListener("blur", applyBackgroundAudioAutoMute);
  window.addEventListener("focus", applyBackgroundAudioAutoMute);
});

onUnmounted(() => {
  window.removeEventListener("keydown", handleKeydown);
  document.removeEventListener("visibilitychange", applyBackgroundAudioAutoMute);
  window.removeEventListener("blur", applyBackgroundAudioAutoMute);
  window.removeEventListener("focus", applyBackgroundAudioAutoMute);

  if (animationTimer && typeof cancelAnimationFrame === "function") {
    cancelAnimationFrame(animationTimer);
    animationTimer = null;
  }
});

watch(
  () => debugControls.enableFogPipeline,
  () => {
    rebuildFogPipeline();
  }
);

watch(
  () => debugControls.enableShadows,
  () => {
    applyShadowDebugState();
  }
);

watch(
  () => debugControls.enableTailAnimation,
  () => {
    applyTailAnimationDebugState();
  }
);

watch(showWorldAxisGrid, () => {
  // ON/OFFの即時反映
  applyWorldAxisGridState();
});

// グローバル調整値の変更を監視し、wasm 側へ逐次反映する。
watch(
  systemSettings,
  () => {
    if (!tuningInitialized.value) {
      return;
    }
    applySystemSettingsToWasm();
    saveToStorage();
  },
  { deep: true }
);

// 種族設定の変更を監視し、WASM 側へ反映 + 必要なら再初期化
watch(
  settings,
  () => {
    wasmBridge?.applySpeciesParams(toRaw(settings), { spatialScale: 1 });

    saveToStorage();

    // 種族構成（個体数・捕食者フラグ）が変わった場合は群れ再初期化
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
    if (typeof boidInstancing.ensurePredatorMeshes === "function") {
      boidInstancing.ensurePredatorMeshes(predators);
    }
    const predatorMeshes =
      typeof boidInstancing.getPredatorMeshes === "function"
        ? boidInstancing.getPredatorMeshes()
        : [];
    for (const mesh of predatorMeshes) {
      mesh.visible = false;
    }
  },
  { deep: true }
);

// 設定をリセット（プリセットまたはデフォルトに戻す）
function resetSettings(presetList) {
  if (Array.isArray(presetList) && presetList.length > 0) {
    applySettingsSnapshot(presetList);
  } else {
    resetToDefaults();
  }
  updateSystemSettings(DEFAULT_TUNING_SETTINGS);
  saveToStorage();
  if (tuningInitialized.value) {
    applySystemSettingsToWasm();
  }
}

// Unit 可視化の変更を監視（デバッグ用）
watch(showUnits, (newValue) => {
  console.log("showUnits changed to:", newValue);
  if (!newValue) {
    // Unit可視化をオフにした場合、既存の可視化要素をクリア
    clearUnitVisuals();
  }
});

watch(showSpeciesEnvelopes, (enabled) => {
  if (!enabled) {
    clearSpeciesEnvelopeVisuals();
  }
});

// Unit表示モードの変更を監視
watch([showUnitSpheres, showUnitLines], ([newSpheres, newLines]) => {
  console.log(
    "Unit display mode changed - Spheres:",
    newSpheres,
    "Lines:",
    newLines
  );
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
  pointer-events: none;
}

.settings {
  margin-bottom: 20px;
  pointer-events: none;
  display: inline-block;
}

.settings > * {
  pointer-events: auto;
  display: inline-block;
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

/*
  UI はカメラ操作（ドラッグ等）を妨げないため、ヒットテストは
  左上のパネル領域(ui-panel)に限定する。
  details を開いた際に子要素が幅100%になっても、画面全体を覆って
  pointer を奪わないようにする。
*/
.ui-panel {
  display: inline-block;
  pointer-events: auto;
}

/*
  設定の説明はブラウザ標準の title ツールチップを使用する。
  - 見た目を増やさず、OS/ブラウザの一貫したUIに任せる。
*/

/*
  デバッグHUD（画面表示）
  - シミュレーションの視認性を損なわないよう、控えめな半透明パネルで表示する。
  - クリック等の入力は奪わない（pointer-events: none）。
*/
.debug-hud {
  position: fixed;
  left: 12px;
  bottom: 12px;
  z-index: 3;
  pointer-events: none;

  max-width: 520px;
  max-height: 45vh;
  overflow: hidden;
  white-space: pre;

  padding: 10px 12px;
  border: 1px solid rgba(255, 255, 255, 0.2);
  border-radius: 5px;
  background-color: rgba(255, 255, 255, 0.05);
  color: #fff;

  font-size: 12px;
  line-height: 1.35;
}

.add-species-button {
  margin-top: 10px;
  margin-bottom: 10px;
}

.tuning-settings {
  pointer-events: none;
  display: inline-block;
  margin-top: 10px;
}

.tuning-settings * {
  pointer-events: auto;
}

.tuning-settings .species-section {
  border: 1px solid rgba(255, 255, 255, 0.2);
  border-radius: 5px;
  background-color: rgba(255, 255, 255, 0.05);
  pointer-events: auto;
  position: relative;
  overflow: visible;
  max-width: 100%;
  width: fit-content;
  min-width: 260px;
}

.tuning-settings .species-header {
  padding: 10px;
  font-weight: bold;
  cursor: pointer;
  background-color: rgba(255, 255, 255, 0.1);
  border-radius: 5px 5px 0 0;
  user-select: none;
  pointer-events: auto;
  display: flex;
  align-items: center;
}

.tuning-settings .species-header:hover {
  background-color: rgba(255, 255, 255, 0.15);
}

.tuning-settings .species-title {
  flex: 1;
}

.tuning-settings .species-content {
  padding: 10px;
  pointer-events: auto;
  box-sizing: border-box;
  width: 100%;
}

.tuning-settings .setting-row {
  display: flex;
  align-items: center;
  margin-bottom: 8px;
  width: 100%;
}

.tuning-settings .setting-row label {
  width: 150px;
  text-align: left;
  margin-right: 10px;
  flex-shrink: 0;
  line-height: 1.3;
}

.tuning-settings .setting-row input[type="range"] {
  width: 140px;
  min-width: 100px;
  max-width: 220px;
  margin: 0 10px;
}

.tuning-settings .value-input {
  width: 70px;
  padding: 2px 4px;
  border: 1px solid #ccc;
  border-radius: 3px;
  background: transparent;
  color: inherit;
  font-size: inherit;
}

.tuning-settings .value-input:focus {
  outline: none;
  border-color: #007bff;
  box-shadow: 0 0 3px rgba(0, 123, 255, 0.3);
}
</style>