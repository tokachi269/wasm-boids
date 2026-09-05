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
                  <label :title="tuningHelp.threatPropagationRate">反応伝播速度<br />(threatPropagationRate):</label>
                  <input
                    type="range"
                    min="0"
                    max="12"
                    step="0.1"
                    v-model.number="systemSettings.threatPropagationRate"
                    :title="tuningHelp.threatPropagationRate"
                  />
                  <input
                    class="value-input"
                    type="number"
                    min="0"
                    step="0.1"
                    v-model.number="systemSettings.threatPropagationRate"
                    :title="tuningHelp.threatPropagationRate"
                  />
                </div>
                <div class="setting-row">
                  <label :title="tuningHelp.threatTransmission">反応伝達率<br />(threatTransmission):</label>
                  <input
                    type="range"
                    min="0"
                    max="1"
                    step="0.01"
                    v-model.number="systemSettings.threatTransmission"
                    :title="tuningHelp.threatTransmission"
                  />
                  <input
                    class="value-input"
                    type="number"
                    min="0"
                    max="1"
                    step="0.01"
                    v-model.number="systemSettings.threatTransmission"
                    :title="tuningHelp.threatTransmission"
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
                  水中の吸収/散乱を有効化
                </label>
                <details class="fog-tuning-section">
                  <summary>Fog / Sky tuning（再読込でリセット）</summary>
                  <div class="fog-tuning-content">
                    <label class="debug-checkbox">
                      <input type="checkbox" v-model="fogTuning.showOceanSphere" />
                      天球を表示
                    </label>
                    <div class="setting-row compact-setting-row">
                      <label>Fog色:</label>
                      <input type="color" v-model="fogTuning.color" />
                      <input class="color-value-input" v-model="fogTuning.color" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>天球・水面色:</label>
                      <input type="color" v-model="fogTuning.skyHighlight" />
                      <input class="color-value-input" v-model="fogTuning.skyHighlight" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>天球・中層色:</label>
                      <input type="color" v-model="fogTuning.skyBlue" />
                      <input class="color-value-input" v-model="fogTuning.skyBlue" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>天球・深部色:</label>
                      <input type="color" v-model="fogTuning.deepBlue" />
                      <input class="color-value-input" v-model="fogTuning.deepBlue" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>開始距離:</label>
                      <input type="range" min="0" max="10" step="0.1" v-model.number="fogTuning.distanceStart" />
                      <input class="value-input" type="number" step="0.1" v-model.number="fogTuning.distanceStart" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>最大距離:</label>
                      <input type="range" min="2" max="50" step="0.5" v-model.number="fogTuning.distanceEnd" />
                      <input class="value-input" type="number" step="0.5" v-model.number="fogTuning.distanceEnd" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>距離カーブ:</label>
                      <input type="range" min="0.1" max="2" step="0.05" v-model.number="fogTuning.distanceExponent" />
                      <input class="value-input" type="number" step="0.05" v-model.number="fogTuning.distanceExponent" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>最大濃度:</label>
                      <input type="range" min="0" max="1" step="0.01" v-model.number="fogTuning.maxOpacity" />
                      <input class="value-input" type="number" step="0.01" v-model.number="fogTuning.maxOpacity" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>深度係数:</label>
                      <input type="range" min="0" max="0.15" step="0.002" v-model.number="fogTuning.heightFalloff" />
                      <input class="value-input" type="number" step="0.002" v-model.number="fogTuning.heightFalloff" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>直接光減衰 RGB:</label>
                      <input type="color" v-model="fogTuning.directAttenuationColor" />
                      <input class="color-value-input" v-model="fogTuning.directAttenuationColor" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>散乱増加 RGB:</label>
                      <input type="color" v-model="fogTuning.backscatterAttenuationColor" />
                      <input class="color-value-input" v-model="fogTuning.backscatterAttenuationColor" />
                    </div>
                    <div class="fog-vector-group">
                      <span>深度光減衰 RGB</span>
                      <input v-for="axis in fogVectorAxes" :key="`depth-${axis}`" class="vector-value-input" type="number" min="0" step="0.0001" v-model.number="fogTuning.depthLightAttenuation[axis]" :aria-label="`深度光減衰 ${axis.toUpperCase()}`" />
                    </div>
                    <button type="button" @click="resetFogTuning">Fog / Skyを既定値へ戻す</button>
                  </div>
                </details>
                <details class="fog-tuning-section">
                  <summary>Lighting tuning（再読込でリセット）</summary>
                  <div class="fog-tuning-content">
                    <div class="setting-row compact-setting-row">
                      <label>Ambient色:</label>
                      <input type="color" v-model="lightingTuning.ambientColor" />
                      <input class="color-value-input" v-model="lightingTuning.ambientColor" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>Ambient強度:</label>
                      <input type="range" min="0" max="3" step="0.05" v-model.number="lightingTuning.ambientIntensity" />
                      <input class="value-input" type="number" min="0" step="0.05" v-model.number="lightingTuning.ambientIntensity" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>Sun色:</label>
                      <input type="color" v-model="lightingTuning.sunColor" />
                      <input class="color-value-input" v-model="lightingTuning.sunColor" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>Sun強度:</label>
                      <input type="range" min="0" max="20" step="0.1" v-model.number="lightingTuning.sunIntensity" />
                      <input class="value-input" type="number" min="0" step="0.1" v-model.number="lightingTuning.sunIntensity" />
                    </div>
                    <div class="setting-row compact-setting-row">
                      <label>露出:</label>
                      <input type="range" min="0.1" max="2.5" step="0.05" v-model.number="lightingTuning.exposure" />
                      <input class="value-input" type="number" min="0.1" step="0.05" v-model.number="lightingTuning.exposure" />
                    </div>
                    <button type="button" @click="resetLightingTuning">Lightingを既定値へ戻す</button>
                  </div>
                </details>
                <label class="debug-checkbox" :title="debugHelp.enableEnhancedPostEffects">
                  <input
                    type="checkbox"
                    v-model="debugControls.enableEnhancedPostEffects"
                    :title="debugHelp.enableEnhancedPostEffects"
                  />
                  高品質エフェクト（SSAO/ブルーム）
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
                <label class="debug-checkbox" :title="debugHelp.enableStats">
                  <input type="checkbox" v-model="debugControls.enableStats" :title="debugHelp.enableStats" />
                  stats-gl を有効化
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
import {
  BrowserBenchmark,
  readBrowserBenchmarkConfig,
} from "./benchmark/BrowserBenchmark.js";

// WASM 側の初期配置レンジ（posRange）。描画側の位置量子化レンジ決定にも使う。
const DEFAULT_SIMULATION_POS_RANGE = 4;
const browserBenchmarkConfig = readBrowserBenchmarkConfig(
  typeof window !== "undefined" ? window.location.search : "",
);
const browserBenchmark = browserBenchmarkConfig
  ? new BrowserBenchmark(browserBenchmarkConfig)
  : null;

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
const defaultBoidCount = useLowSpecPreset ? 10000 : 10000;

function isConservativeRendererMode() {
  return webglContextLost || webglRecoveryAttempt > 0;
}

function shouldUseAntialias() {
  return !isConservativeRendererMode();
}

function shouldUseUnderwaterEnvMap() {
  return webglRecoveryAttempt === 0;
}

function getEffectivePixelRatioCap() {
  if (webglContextLost || webglRecoveryAttempt > 0) {
    return Math.min(MAX_RENDER_PIXEL_RATIO, 1.0);
  }
  return MAX_RENDER_PIXEL_RATIO;
}

const DEFAULT_SETTINGS = [
  {
    species: "Boids", // 種族名
    count: defaultBoidCount, // 群れの数（低スペックでは軽量化）
    // 画面ガワの初期値（画像の値）
    cohesion: 3.66, // 凝集
    cohesionRange: 5, // 凝集範囲
    separation: 1.0, // 分離
    separationRange: 0.4, // 分離範囲
    alignment: 8.0, // 整列
    alignmentRange: 1, // 整列範囲
    maxSpeed: 0.35, // 最大速度
    maxTurnAngle: 0.75, // 最大曲がり（曲率）
    maxNeighbors: 4, // 最大近傍数
    horizontalTorque: 0.03, // 水平化トルク
    torqueStrength: 1.5, // 回転トルク強度
    lambda: 0.102, // 速度調整係数（減衰係数）
    tau: 0.5, // 記憶時間
    predatorAlertRadius: 2.5, // 捕食者を早めに察知して空隙を作る距離
    densityReturnStrength: 0.0, // 密度復帰強度
    schoolPullEnabled: true, // 大クラスタ引力係数を反映するか
    isPredator: false,
  },
  {
    species: "Predator",
    count: 2,
    cohesion: 0.0, // 捕食者は群集力学を使わない
    cohesionRange: 5.0,
    separation: 0.0,
    separationRange: 0.1,
    alignment: 0.0,
    alignmentRange: 1.0,
    predatorAlertRadius: 0.0,
    densityReturnStrength: 0.0,
    maxSpeed: 1.5,
    minSpeed: 1.2,
    maxTurnAngle: 0.4,
    maxNeighbors: 0,
    lambda: 0.05,
    tau: 1.0, // 捕食者は常に追いかける
    horizontalTorque: 0.01,
    torqueStrength: 2.5,
    schoolPullEnabled: false,
    isPredator: true, // 捕食者フラグ
  },
];
const DEFAULT_TUNING_SETTINGS = {
  threatDecay: 0.75, // 脅威減衰速度（1/sec）。少し長めに残して空隙を維持
  threatPropagationRate: 4.0, // 近傍の脅威へ追従する速度（1/sec）
  threatTransmission: 0.82, // 1 hopで伝わる脅威の割合
  maxEscapeWeight: 0.6, // 逃避方向の最大割合（0〜1）
  baseEscapeStrength: 4.0, // 逃避舵取り強度（目標速度へ寄せる強さ）
  fastAttractStrength: 1.0, // 近傍不足時の補助凝集強度（0で無効）
  schoolPullCoefficient: 0.0004, // 大クラスタ引力係数
};
  
// 調整スライダーの説明（ユーザ目線）。ホバー時に title として表示する。
// NOTE: 実装の内部用語ではなく「何がどう変わるか」を短く書く。
const tuningHelp = {
  threatDecay: '脅威（捕食者などの危険度）が時間でどれだけ早く消えるか。大きいほど早く落ち着きます。',
  threatPropagationRate: '直接反応した魚の逃避反応が、近くの魚へ伝わる速さ。大きいほど波及が速くなります。',
  threatTransmission: '逃避反応が隣の魚へ伝わるたびに残る割合。小さいほど遠くまで伝わりにくくなります。',
  maxEscapeWeight: '逃避行動をどれだけ優先するか（0〜1）。1 に近いほど、危険時はほぼ逃げが優先されます。',
  baseEscapeStrength: '逃避の舵取り強度（目標速度へ寄せる強さ）。大きいほど素早く逃げ方向へ乗ります。',
  fastAttractStrength: '近くの仲間が少ないときに、群れへ戻す補助の凝集強度（0で無効）。',
  schoolPullCoefficient: '大きな群れ（大クラスタ）へ引き寄せる強さ。大きいほど大群にまとまりやすいです。',
};

// デバッグ表示/負荷設定の説明（ユーザ目線）。
const debugHelp = {
  enableFogPipeline: '軽量な水中の色吸収と散乱を有効にします。',
  enableEnhancedPostEffects: 'SSAOとブルームをまとめて有効にします。スマホや重い環境ではOFFを推奨。',
  enableShadows: '影描画を有効にします。見た目は良くなりますが負荷が上がりやすいです。',
  enableTailAnimation: '尾びれのアニメーションを有効にします。負荷が気になるなら OFF。',
  enableStats: 'stats-gl の計測 HUD を有効にします。通常は OFF で十分です。',
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

if (browserBenchmarkConfig) {
  const benchmarkSettings = DEFAULT_SETTINGS.map((entry) => ({ ...entry }));
  const fixedSecondaryCount = benchmarkSettings
    .slice(1)
    .reduce((sum, entry) => sum + entry.count, 0);
  benchmarkSettings[0].count = Math.max(
    1,
    browserBenchmarkConfig.boids - fixedSecondaryCount,
  );
  replaceSettings(benchmarkSettings);
  syncSystemSettings(DEFAULT_TUNING_SETTINGS);
}

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
let ambientLight = null; // 環境光。非永続の照明調整UIから更新する
let groundMesh = null; // 地面メッシュの参照
let oceanSphere = null; // 背景天球。Fog調整時の比較用に表示を切り替える
let underwaterEnvMap = null; // 海中スペキュラ用 PMREM 環境マップ

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
  lastUserInteractionAtMs: 0,
  resumeAfterIdleMs: 60000,
  fadeInDurationMs: 10000,
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
  enableFogPipeline: true,
  enableEnhancedPostEffects: !useLowSpecPreset,
  enableShadows: !useLowSpecPreset,
  enableTailAnimation: true,
  enableStats: false,
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
let statsInitPromise = null;
let glContext = null;
let frameCounter = 0;
let flockReinitTimer = null; // 群れ再初期化の遅延タイマー

// モバイルはWebGL context喪失が起きやすいので、復旧のための状態を持つ。
let rendererCanvas = null;
let webglContextLost = false;
let webglRecoveryTimer = null;
let webglRecoveryAttempt = 0;
let boidAssetsReady = false;
let resizeHooked = false;
// WebGL2 が不安定な端末では、いったん WebGL1 に固定して再生成ループを避ける。
let forceWebgl1 = false;

// WebGL 復旧で renderer/camera を作り直すと、視点が初期値へ戻りやすい。
// 直前の視点をスナップショットして復元し、UXを維持する。
let cameraSnapshotForRecovery = null;

function snapshotCameraStateForRecovery() {
  if (!camera || !controls) {
    return;
  }
  cameraSnapshotForRecovery = {
    position: { x: camera.position.x, y: camera.position.y, z: camera.position.z },
    target: { x: controls.target.x, y: controls.target.y, z: controls.target.z },
  };
}

function restoreCameraStateAfterRecovery() {
  if (!camera || !controls || !cameraSnapshotForRecovery) {
    return;
  }

  camera.position.set(
    cameraSnapshotForRecovery.position.x,
    cameraSnapshotForRecovery.position.y,
    cameraSnapshotForRecovery.position.z
  );
  controls.target.set(
    cameraSnapshotForRecovery.target.x,
    cameraSnapshotForRecovery.target.y,
    cameraSnapshotForRecovery.target.z
  );
  controls.update();

  // 復旧後は自動注視を無効化（ユーザーが見ていた視点を優先）。
  startupCameraLookAt.active = false;
  startupCameraLookAt.userInteracted = true;
  startupCameraLookAt.lastUserInteractionAtMs = performance.now();
}

function markCameraInteraction() {
  startupCameraLookAt.active = false;
  startupCameraLookAt.userInteracted = true;
  startupCameraLookAt.lastUserInteractionAtMs = performance.now();
}

function resumeAutoLookAtIfIdle() {
  if (startupCameraLookAt.active || !startupCameraLookAt.userInteracted) {
    return;
  }
  const idleMs = performance.now() - startupCameraLookAt.lastUserInteractionAtMs;
  if (idleMs < startupCameraLookAt.resumeAfterIdleMs) {
    return;
  }
  startupCameraLookAt.active = true;
  startupCameraLookAt.userInteracted = false;
  startupCameraLookAt.startedAtMs = performance.now();
}

function shouldAttemptWebglRecovery() {
  if (webglContextLost) {
    return true;
  }
  if (!renderer || !scene || !camera) {
    return true;
  }
  const ctx = glContext || renderer?.getContext?.();
  if (ctx && typeof ctx.isContextLost === "function" && ctx.isContextLost()) {
    return true;
  }
  return false;
}

let animationTimer = null;
// requestAnimationFrame で vsync に同期して更新する。
// setTimeout の高頻度ループは余計なスケジューリング負荷を生みやすい。
const COUNT_REINIT_DELAY_MS = 400; // 個体数変更後の再初期化待機時間

function applyRendererPixelRatio() {
  if (!renderer || typeof window === "undefined") {
    return;
  }
  const dpr = window.devicePixelRatio || 1;
  renderer.setPixelRatio(Math.min(dpr, getEffectivePixelRatioCap()));
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

function getStatsElement() {
  return (
    (typeof stats?.domElement !== "undefined" ? stats.domElement : null) ||
    (typeof stats?.getDom === "function" ? stats.getDom() : null) ||
    stats?.dom ||
    stats?.container ||
    stats?.wrapper ||
    null
  );
}

function setStatsOverlayVisibility(visible) {
  const statsElement = getStatsElement();
  if (!statsElement) {
    return;
  }
  if (visible) {
    if (!statsElement.parentElement) {
      document.body.appendChild(statsElement);
    }
    positionStatsOverlay(statsElement);
    statsElement.style.display = "";
    return;
  }
  if (statsElement.parentElement) {
    statsElement.parentElement.removeChild(statsElement);
  }
}

function ensureStatsInitialized() {
  if (stats) {
    setStatsOverlayVisibility(true);
    return Promise.resolve(stats);
  }
  if (statsInitPromise) {
    return statsInitPromise;
  }

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
  const initPromise =
    stats && typeof stats.init === "function"
      ? Promise.resolve(stats.init(statsInitTarget))
      : Promise.resolve();

  statsInitPromise = initPromise
    .then(() => {
      if (renderer && typeof stats?.patchThreeRenderer === "function" && !stats?.threeRendererPatched) {
        stats.patchThreeRenderer(renderer);
      }
      setStatsOverlayVisibility(debugControls.enableStats);
      return stats;
    })
    .catch((error) => {
      console.error("Failed to initialize stats-gl:", error);
      setStatsOverlayVisibility(debugControls.enableStats);
      return stats;
    })
    .finally(() => {
      statsInitPromise = null;
    });

  return statsInitPromise;
}

function applyStatsDebugState() {
  if (debugControls.enableStats) {
    void ensureStatsInitialized();
    return;
  }
  setStatsOverlayVisibility(false);
  if (!stats) {
    return;
  }
  stats = null;
  statsInitPromise = null;

  if (!renderer) {
    return;
  }

  snapshotCameraStateForRecovery();
  const ok = initThreeJS();
  if (!ok) {
    scheduleWebglRecovery("stats-toggle");
    return;
  }
  restoreCameraStateAfterRecovery();
  if (boidAssetsReady) {
    initInstancedBoids(cachedTotalBoidCount || totalBoids.value || 0);
  }
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
      fogPipeline = new FogPipeline(heightFogConfig, {
        enableEnhancedEffects: debugControls.enableEnhancedPostEffects,
        internalScale: deviceProfile.isMobile ? 0.75 : 1.0,
      });
    }
    fogPipeline.enableEnhancedEffects = debugControls.enableEnhancedPostEffects;
    fogPipeline.internalScale = deviceProfile.isMobile ? 0.75 : 1.0;
    const size = new THREE.Vector2();
    renderer.getSize(size);
    fogPipeline.init(renderer, scene, camera, size.x, size.y);
  } else if (fogPipeline) {
    fogPipeline.dispose();
    fogPipeline = null;
  }
  applyFogTuning();
}

function clearWebglRecoveryTimer() {
  if (webglRecoveryTimer) {
    clearTimeout(webglRecoveryTimer);
    webglRecoveryTimer = null;
  }
}

function detachRendererCanvas() {
  const canvas = renderer?.domElement ?? rendererCanvas;
  if (!canvas) {
    return;
  }
  if (canvas.parentElement) {
    canvas.parentElement.removeChild(canvas);
  }
}

function disposeRendererAndPipeline() {
  if (fogPipeline) {
    fogPipeline.dispose();
    fogPipeline = null;
  }

  if (underwaterEnvMap) {
    underwaterEnvMap.dispose();
    underwaterEnvMap = null;
  }

  if (renderer) {
    rendererCanvas = renderer.domElement;
    renderer.dispose?.();
  }
  renderer = null;
  glContext = null;
}

function createWebglContext(canvas, preferWebgl2) {
  const attrs = {
    alpha: false,
    antialias: shouldUseAntialias(),
    depth: true,
    stencil: false,
    preserveDrawingBuffer: false,
    powerPreference: deviceProfile.isMobile ? 'default' : 'high-performance',
    failIfMajorPerformanceCaveat: false,
  };

  if (preferWebgl2) {
    const gl2 = canvas.getContext('webgl2', attrs);
    if (gl2) {
      return gl2;
    }
  }

  // WebGL2 が取れない端末向けのフォールバック。
  return canvas.getContext('webgl', attrs) || canvas.getContext('experimental-webgl', attrs);
}

function onWebglContextLost(event) {
  // 既定動作（自動リロード等）を止め、こちらで復旧を試みる。
  event?.preventDefault?.();
  webglContextLost = true;
  console.warn('WebGL context lost. Scheduling recovery...');

  // 復旧後に視点を戻す。
  snapshotCameraStateForRecovery();

  // ループを止めて、再生成後に再開する。
  if (animationTimer && typeof cancelAnimationFrame === 'function') {
    cancelAnimationFrame(animationTimer);
    animationTimer = null;
  }

  // GPU側はステートレスになるので、レンダラー/パイプラインだけ作り直す。
  // NOTE: WebGL2 が不安定な端末では喪失→再生成でブラウザにブロックされやすい。
  // 一度でも WebGL2 で喪失したら WebGL1 を優先し、安定性を取る。
  if (renderer?.capabilities?.isWebGL2) {
    forceWebgl1 = true;
    // WebGL2 コンテキストを持つ canvas は WebGL1 に切替できないため、次回は作り直す。
    rendererCanvas = null;
  }
  disposeRendererAndPipeline();
  scheduleWebglRecovery('contextlost');
}

function onWebglContextRestored() {
  webglContextLost = false;
  console.warn('WebGL context restored. Reinitializing renderer...');
  snapshotCameraStateForRecovery();
  scheduleWebglRecovery('contextrestored');
}

function scheduleWebglRecovery(reason) {
  if (webglRecoveryTimer) {
    return;
  }

  // focus/visibility などから呼ばれても、正常時は再初期化しない。
  if (!shouldAttemptWebglRecovery()) {
    return;
  }

  // 連続で失敗するとブラウザがブロックするため、指数バックオフで再試行する。
  const delayMs = Math.min(8000, 500 * Math.pow(2, webglRecoveryAttempt));
  webglRecoveryAttempt = Math.min(webglRecoveryAttempt + 1, 6);

  if (deviceProfile.isMobile) {
    debugControls.enableShadows = false;
    debugControls.enableEnhancedPostEffects = false;
    debugControls.enableFogPipeline = false;
  }

  webglRecoveryTimer = setTimeout(() => {
    webglRecoveryTimer = null;
    try {
      // 待っている間に復旧できていれば何もしない。
      if (!shouldAttemptWebglRecovery()) {
        webglRecoveryAttempt = 0;
        clearWebglRecoveryTimer();
        return;
      }

      const ok = initThreeJS();
      if (!ok) {
        scheduleWebglRecovery('retry:' + reason);
        return;
      }

      // レンダラ作り直しでリセットされたカメラを復元。
      restoreCameraStateAfterRecovery();

      // stats-gl は renderer を差し替えると参照が古くなるため、可能なら再パッチする。
      if (stats && typeof stats.patchThreeRenderer === 'function') {
        stats.patchThreeRenderer(renderer);
      }

      // GPUは失われるので、boidsは次フレームの update() で再送される。
      // モデルロード済みならインスタンシングを作り直して描画を復帰する。
      if (boidAssetsReady) {
        initInstancedBoids(cachedTotalBoidCount || totalBoids.value || 0);
      }

      // ループが止まっていれば再開。
      if (!animationTimer) {
        scheduleNextFrame();
      }

      // 復旧に成功したらカウンタをリセット。
      webglRecoveryAttempt = 0;
      clearWebglRecoveryTimer();
    } catch (error) {
      console.warn('WebGL recovery failed:', error);
      scheduleWebglRecovery('error:' + reason);
    }
  }, delayMs);
}

function initThreeJS() {
  const width = window.innerWidth;
  const height = window.innerHeight;

  // renderer/camera を作り直す前に、直前の視点を保持する。
  snapshotCameraStateForRecovery();

  // 既存のレンダラを捨てて作り直す（context lost 復旧用）。
  detachRendererCanvas();
  disposeRendererAndPipeline();

  // 以前の scene を保持したまま再生成するとメモリが積み上がるので、都度作り直す。
  scene = null;
  camera = null;
  controls = null;

  scene = new THREE.Scene();
  scene.background = new THREE.Color('#062040'); // フォグ色・背景球底色と揃えて遠景を統一
  oceanSphere = createOceanSphere();

  camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
  camera.position.set(3, -5, 3);
  camera.lookAt(0, 0, 0);

  // まず WebGL2 を試し、ダメなら WebGL1 にフォールバックする。
  // NOTE: 端末によっては WebGL2 が常に失敗する（省電力/制限/一時ブロック等）。
  const canvas = rendererCanvas || document.createElement('canvas');
  const preferWebgl2 = !forceWebgl1;
  const context = createWebglContext(canvas, preferWebgl2);
  if (!context) {
    console.warn('Failed to create WebGL context (webgl2/webgl).');
    rendererCanvas = canvas;
    // 復旧ループへ回す（ここでは例外にしない）。
    scheduleWebglRecovery('init');
    return false;
  }

  // WebGL2 を期待していたのに WebGL1 にフォールバックした場合は、以降も WebGL1 を優先する。
  // （WebGL2 を毎回試して失敗→ブロック、を避ける）
  const isWebgl2 = (typeof WebGL2RenderingContext !== 'undefined') && (context instanceof WebGL2RenderingContext);
  if (preferWebgl2 && !isWebgl2) {
    forceWebgl1 = true;
  }

  renderer = new THREE.WebGLRenderer({
    canvas,
    context,
    antialias: shouldUseAntialias(),
    depth: true,
  });
  // テクスチャは sRGB 前提で運用しているため、出力色空間も明示して「くすみ」を避ける。
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  // ACESFilmic は映画的なコントラストと彩度を維持しつつダイナミクスを再現する。
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;
  // 海中環境マップを生成し、シーンのスペキュラ反射（metalness）に使用する。
  // PMREMGenerator で水面〜深海グラデーションをフィルタリング済みキューブマップに変換。
  if (shouldUseUnderwaterEnvMap()) {
    underwaterEnvMap = createUnderWaterEnvMap(renderer);
    scene.environment = underwaterEnvMap;
  } else {
    underwaterEnvMap = null;
    scene.environment = null;
  }
  applyRendererPixelRatio();
  renderer.setSize(width, height);
  renderer.shadowMap.enabled = debugControls.enableShadows;
  renderer.shadowMap.type = THREE.PCFShadowMap; // 影を柔らかく

  glContext = renderer.getContext();

  // context lost/restore を拾って自動復旧する。
  // 同一canvasを使い回すことで「作りすぎブロック」を避ける。
  rendererCanvas = renderer.domElement;
  rendererCanvas.addEventListener('webglcontextlost', onWebglContextLost, false);
  rendererCanvas.addEventListener('webglcontextrestored', onWebglContextRestored, false);

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
  startupCameraLookAt.lastUserInteractionAtMs = startupCameraLookAt.startedAtMs;
  // controls は context 復旧で作り直されるため、毎回フックし直す。
  startupCameraLookAt.controlsHooked = true;
  controls.addEventListener("start", () => {
    markCameraInteraction();
  });
  controls.addEventListener("end", () => {
    markCameraInteraction();
  });

  // 地面メッシュ追加
  const groundGeo = new THREE.PlaneGeometry(300, 300);
  const groundMat = createFadeOutGroundMaterial();
  groundMat.depthTest = true;
  groundMesh = new THREE.Mesh(groundGeo, groundMat);
  groundMesh.rotation.x = -Math.PI / 2;
  groundMesh.position.y = -9;
  groundMesh.receiveShadow = true; // 影を受ける
  scene.add(groundMesh);

  // ライト
  ambientLight = new THREE.AmbientLight(
    toHex(OCEAN_COLORS.AMBIENT_LIGHT),
    1.1
  );
  scene.add(ambientLight);

  // 水面を透過した青白いカスティック光（水中の荘厳な輝きを演出）
  dirLight = new THREE.DirectionalLight(toHex(OCEAN_COLORS.SUN_LIGHT), 9); // 水中カスティック光（穏やか）
  dirLight.position.set(300, 500, 200); // 高い位置から照らす
  dirLight.castShadow = true;

  // 影カメラの範囲を広げる
  dirLight.shadow.camera.left = -100;
  dirLight.shadow.camera.right = 100;
  dirLight.shadow.camera.top = 100;
  dirLight.shadow.camera.bottom = -100;
  dirLight.shadow.camera.near = 1;
  dirLight.shadow.camera.far = 1200;
  dirLight.shadow.camera.updateProjectionMatrix();

  dirLight.shadow.mapSize.width = 768;
  dirLight.shadow.mapSize.height = 768;
  dirLight.shadow.bias = -0.01;
  dirLight.shadow.normalBias = 0.01;

  scene.add(dirLight);
  applyLightingTuning();
  initParticleSystem();
  rebuildFogPipeline();
  applyShadowDebugState();
  applyTailAnimationDebugState();
  applyWorldAxisGridState();
  // ウィンドウリサイズ対応
  if (!resizeHooked) {
    resizeHooked = true;
    window.addEventListener("resize", onWindowResize);
  }

  webglContextLost = false;
  return true;
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
  resumeAutoLookAtIfIdle();
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
  const elapsedMs = Math.max(0, performance.now() - startupCameraLookAt.startedAtMs);
  const fadeT = startupCameraLookAt.fadeInDurationMs > 0
    ? Math.min(elapsedMs / startupCameraLookAt.fadeInDurationMs, 1)
    : 1;
  const fadeIn = fadeT < 0.5
    ? 4 * fadeT * fadeT * fadeT
    : 1 - Math.pow(-2 * fadeT + 2, 3) / 2;
  const effectiveSmoothingSpeed = startupCameraLookAt.smoothingSpeed * (0.15 + 0.85 * fadeIn);
  const smoothing = 1.0 - Math.exp(-effectiveSmoothingSpeed * safeDeltaTime);

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
  forceLowLod: deviceProfile.isMobile,
});

let instancedMeshHigh = null;
let instancedMeshLow = null;

// 海中シーンの色味をまとめて管理する定数群
const OCEAN_COLORS = {
  SKY_HIGHLIGHT: "#6ac8e0",   // 水面側（少しくぐもった青白）
  SKY_BLUE: "#0574a8",        // 中層の濃い海の青
  DEEP_BLUE: "#031c4d",       // 深い濃紺
  SEAFLOOR: "#3d5a4a",        // 海底：海藻や泥を帯びた緑灰色
  AMBIENT_LIGHT: "#3a4e5e",   // 低彩度の青灰色（影に青みが出すぎない）
  SUN_LIGHT: "#b8dff4",       // 水中を透過した青白いカスティック光
  SIDE_LIGHT1: "#2a7ba8",     // 水中サイドライト
  SIDE_LIGHT2: "#0c3a52",     // 深部サイドライト
  BOTTOM_LIGHT: "#040e1a",    // 最深部の光
};

// '#rrggbb' 形式の色を three.js の整数表現に変換
const toHex = (colorStr) => parseInt(colorStr.replace("#", "0x"), 16);

// 距離と深度で濃さが変わる海中フォグ設定
const heightFogConfig = {
  color: new THREE.Color('#1d2e35'), // 距離とともに加わる低彩度のveiling light
  distanceStart: 0.4,
  distanceEnd: 15.0,
  distanceExponent: 0.3, // 距離カーブを少し勾配に
  distanceControlPoint1: new THREE.Vector2(0.1, 0.25), // 開始側：早めにフォグを踏む
  distanceControlPoint2: new THREE.Vector2(0.85, 0.92), // 終端側
  surfaceLevel: 100.0, // 水面の高さ
  heightFalloff: 0.024,
  heightExponent: 1.2, // 深度カーブを少し勾配に
  maxOpacity: 0.56,
  directAttenuation: new THREE.Vector3(0x15 / 255, 0x0b / 255, 0x05 / 255), // UI #150b05。赤ほど早く失われる
  backscatterAttenuation: new THREE.Vector3(0x21 / 255, 0x1a / 255, 0x12 / 255), // UI #211a12
  depthLightAttenuation: new THREE.Vector3(0.004, 0.0018, 0.0007),
};

const fogVectorAxes = ['x', 'y', 'z'];

function attenuationVectorToHex(vector) {
  const component = (value) => Math.round(
    THREE.MathUtils.clamp(Number(value) || 0, 0, 1) * 255,
  ).toString(16).padStart(2, '0');
  return `#${component(vector.x)}${component(vector.y)}${component(vector.z)}`;
}

function attenuationHexToVector(value, fallback) {
  const match = /^#([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})$/i.exec(String(value));
  if (!match) {
    return fallback.clone();
  }
  return new THREE.Vector3(
    parseInt(match[1], 16) / 255,
    parseInt(match[2], 16) / 255,
    parseInt(match[3], 16) / 255,
  );
}

const initialFogTuning = Object.freeze({
  color: `#${heightFogConfig.color.getHexString()}`,
  skyHighlight: OCEAN_COLORS.SKY_HIGHLIGHT,
  skyBlue: OCEAN_COLORS.SKY_BLUE,
  deepBlue: OCEAN_COLORS.DEEP_BLUE,
  distanceStart: heightFogConfig.distanceStart,
  distanceEnd: heightFogConfig.distanceEnd,
  distanceExponent: heightFogConfig.distanceExponent,
  maxOpacity: heightFogConfig.maxOpacity,
  heightFalloff: heightFogConfig.heightFalloff,
  directAttenuationColor: attenuationVectorToHex(heightFogConfig.directAttenuation),
  backscatterAttenuationColor: attenuationVectorToHex(heightFogConfig.backscatterAttenuation),
  depthLightAttenuation: Object.freeze({
    x: heightFogConfig.depthLightAttenuation.x,
    y: heightFogConfig.depthLightAttenuation.y,
    z: heightFogConfig.depthLightAttenuation.z,
  }),
});

function copyInitialFogTuning() {
  return {
    ...initialFogTuning,
    showOceanSphere: true,
    depthLightAttenuation: { ...initialFogTuning.depthLightAttenuation },
  };
}

// デフォルト値を決めるためだけの非永続UI。localStorageへは保存しない。
const fogTuning = reactive(copyInitialFogTuning());

function tuningVector3(value) {
  return new THREE.Vector3(
    Math.max(0, Number(value?.x) || 0),
    Math.max(0, Number(value?.y) || 0),
    Math.max(0, Number(value?.z) || 0),
  );
}

function currentFogTuningConfig() {
  return {
    ...heightFogConfig,
    color: new THREE.Color(fogTuning.color),
    distanceStart: Number(fogTuning.distanceStart),
    distanceEnd: Number(fogTuning.distanceEnd),
    distanceExponent: Number(fogTuning.distanceExponent),
    maxOpacity: Number(fogTuning.maxOpacity),
    heightFalloff: Number(fogTuning.heightFalloff),
    directAttenuation: attenuationHexToVector(
      fogTuning.directAttenuationColor,
      heightFogConfig.directAttenuation,
    ),
    backscatterAttenuation: attenuationHexToVector(
      fogTuning.backscatterAttenuationColor,
      heightFogConfig.backscatterAttenuation,
    ),
    depthLightAttenuation: tuningVector3(fogTuning.depthLightAttenuation),
  };
}

function applyFogTuning() {
  const config = currentFogTuningConfig();
  fogPipeline?.updateConfig(config);
  updateGroundUnderwaterMedium(
    groundMesh?.material,
    debugControls.enableFogPipeline ? config : { ...config, maxOpacity: 0 },
  );

  if (oceanSphere) {
    oceanSphere.visible = fogTuning.showOceanSphere;
    updateOceanSphereGradient(oceanSphere.material?.map, fogTuning);
  }
}

function resetFogTuning() {
  Object.assign(fogTuning, copyInitialFogTuning());
}

const initialLightingTuning = Object.freeze({
  ambientColor: '#466177',
  ambientIntensity: 1.55,
  sunColor: OCEAN_COLORS.SUN_LIGHT,
  sunIntensity: 7.7,
  exposure: 1.1,
});
const lightingTuning = reactive({ ...initialLightingTuning });

function applyLightingTuning() {
  if (ambientLight) {
    ambientLight.color.set(lightingTuning.ambientColor);
    ambientLight.intensity = Math.max(0, Number(lightingTuning.ambientIntensity) || 0);
  }
  if (dirLight) {
    dirLight.color.set(lightingTuning.sunColor);
    dirLight.intensity = Math.max(0, Number(lightingTuning.sunIntensity) || 0);
  }
  if (renderer) {
    renderer.toneMappingExposure = Math.max(0.01, Number(lightingTuning.exposure) || 1);
  }
}

function resetLightingTuning() {
  Object.assign(lightingTuning, initialLightingTuning);
}

/**
 * 海中環境マップを生成する。
 * 水面（上）→海の青（中）→深海（下）のグラデーションを equirectangular テクスチャとして作成し
 * PMREMGenerator でフィルタリングすることで MeshStandardMaterial の envMap に使用できる形式にする。
 * metalness/roughness が正しく機能するためには scene.environment の設定が必須。
 */
function createUnderWaterEnvMap(rendererRef) {
  const width = 64;
  const height = 32;
  const data = new Uint8Array(width * height * 4);

  // 上: 明るいシアン寄りの水面反射  →  中: 海中の青  →  下: 深海寄りの青黒
  const topR = 0x72, topG = 0xd8, topB = 0xff;
  const midR = 0x1e, midG = 0x56, midB = 0x78;
  const botR = 0x06, botG = 0x16, botB = 0x2c;

  for (let y = 0; y < height; y++) {
    // equirectangular では y=0 が上方向（天頂/水面）
    const t = y / (height - 1);
    let r, g, b;
    if (t < 0.5) {
      const s = t * 2.0;
      r = Math.round(topR + (midR - topR) * s);
      g = Math.round(topG + (midG - topG) * s);
      b = Math.round(topB + (midB - topB) * s);
    } else {
      const s = (t - 0.5) * 2.0;
      r = Math.round(midR + (botR - midR) * s);
      g = Math.round(midG + (botG - midG) * s);
      b = Math.round(midB + (botB - midB) * s);
    }
    for (let x = 0; x < width; x++) {
      const base = (y * width + x) * 4;
      data[base + 0] = r;
      data[base + 1] = g;
      data[base + 2] = b;
      data[base + 3] = 255;
    }
  }

  const envTexture = new THREE.DataTexture(data, width, height, THREE.RGBAFormat);
  envTexture.colorSpace = THREE.SRGBColorSpace;
  envTexture.mapping = THREE.EquirectangularReflectionMapping;
  envTexture.needsUpdate = true;

  const pmremGenerator = new THREE.PMREMGenerator(rendererRef);
  pmremGenerator.compileEquirectangularShader();
  const envRT = pmremGenerator.fromEquirectangular(envTexture);
  pmremGenerator.dispose();
  envTexture.dispose();

  return envRT.texture;
}

function createOceanSphere() {
  if (!scene) return null;

  // 上層→深層のグラデーションで海中の空気感を演出
  const canvas = document.createElement("canvas");
  const context = canvas.getContext("2d");
  canvas.width = 512;
  canvas.height = 512;

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.generateMipmaps = false;
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;
  updateOceanSphereGradient(texture, fogTuning);

  const sphereGeo = new THREE.SphereGeometry(300, 32, 32);
  const sphereMat = new THREE.MeshBasicMaterial({
    map: texture,
    side: THREE.BackSide,
    fog: false,
    // 8bit CanvasTexture の緩い色変化で見える帯を微小ディザで崩す。
    dithering: true,
    // 天球は背景色だけを描く。深度を書かせると水中パスが天球上の
    // surfaceLevelを実在する海面として拾い、水平な境界線を作ってしまう。
    depthWrite: false,
  });

  const oceanSphere = new THREE.Mesh(sphereGeo, sphereMat);
  scene.add(oceanSphere);
  return oceanSphere;
}

function updateOceanSphereGradient(texture, colors) {
  const canvas = texture?.image;
  const context = canvas?.getContext?.('2d');
  if (!canvas || !context) {
    return;
  }

  const signature = [colors.skyHighlight, colors.skyBlue, colors.deepBlue].join('|');
  if (texture.userData.oceanGradientSignature === signature) {
    return;
  }

  const gradient = context.createLinearGradient(0, 0, 0, canvas.height);
  gradient.addColorStop(0, colors.skyHighlight);
  gradient.addColorStop(0.15, colors.skyBlue);
  gradient.addColorStop(0.5, colors.deepBlue);
  gradient.addColorStop(1, '#051535');
  context.fillStyle = gradient;
  context.fillRect(0, 0, canvas.width, canvas.height);
  texture.userData.oceanGradientSignature = signature;
  texture.needsUpdate = true;
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
    emissive: new THREE.Color('#1a2e22'), // フォグ内でも潰れないよう微弱な自発光
    emissiveIntensity: 0.4,
    transparent: true,
    alphaMap,
    // 半透明面を単一値のdepth bufferへ渡すと、alphaに関係なくPlane全体が
    // 四角い深度面になる。色は通常どおりdepthTestし、媒質はmaterial内で処理する。
    depthWrite: false,
  });

  material.roughness = 0.92;
  material.metalness = 0.0;
  applyUnderwaterMediumToGroundMaterial(material, heightFogConfig);
  return material;
}

/**
 * 半透明の海底は画面全体のdepth-based passでは正しく扱えないため、
 * 不透明物と同じ吸収/backscatterをマテリアル側で適用する。
 */
function applyUnderwaterMediumToGroundMaterial(material, config) {
  const uniforms = {
    uGroundFogColor: { value: config.color.clone() },
    uGroundDirectAttenuation: { value: config.directAttenuation.clone() },
    uGroundBackscatterAttenuation: { value: config.backscatterAttenuation.clone() },
    uGroundDepthLightAttenuation: { value: config.depthLightAttenuation.clone() },
    uGroundDistanceStart: { value: config.distanceStart },
    uGroundDistanceEnd: { value: config.distanceEnd },
    uGroundDistanceExponent: { value: config.distanceExponent },
    uGroundSurfaceLevel: { value: config.surfaceLevel },
    uGroundHeightFalloff: { value: config.heightFalloff },
    uGroundHeightExponent: { value: config.heightExponent },
    uGroundMaxOpacity: { value: config.maxOpacity },
  };
  material.userData.underwaterMediumUniforms = uniforms;
  material.onBeforeCompile = (shader) => {
    Object.assign(shader.uniforms, uniforms);

    shader.vertexShader = shader.vertexShader
      .replace(
        '#include <common>',
        '#include <common>\nvarying float vGroundWorldY;'
      )
      .replace(
        '#include <project_vertex>',
        '#include <project_vertex>\nvGroundWorldY = (modelMatrix * vec4(transformed, 1.0)).y;'
      );

    shader.fragmentShader = shader.fragmentShader
      .replace(
        '#include <common>',
        `#include <common>
varying float vGroundWorldY;
uniform vec3 uGroundFogColor;
uniform vec3 uGroundDirectAttenuation;
uniform vec3 uGroundBackscatterAttenuation;
uniform vec3 uGroundDepthLightAttenuation;
uniform float uGroundDistanceStart;
uniform float uGroundDistanceEnd;
uniform float uGroundDistanceExponent;
uniform float uGroundSurfaceLevel;
uniform float uGroundHeightFalloff;
uniform float uGroundHeightExponent;
uniform float uGroundMaxOpacity;`
      )
      .replace(
        '#include <opaque_fragment>',
        `#include <opaque_fragment>
float groundViewDistance = length(vViewPosition);
float groundDistanceNorm = clamp(
  (groundViewDistance - uGroundDistanceStart) /
    max(uGroundDistanceEnd - uGroundDistanceStart, 1e-5),
  0.0,
  1.0
);
float groundDistanceFog = pow(
  smoothstep(0.0, 1.0, groundDistanceNorm),
  uGroundDistanceExponent
);
float groundDepthBelowSurface = max(uGroundSurfaceLevel - vGroundWorldY, 0.0);
float groundHeightFactor = 1.0 - exp(
  -groundDepthBelowSurface * uGroundHeightFalloff
);
groundHeightFactor = clamp(
  pow(groundHeightFactor, uGroundHeightExponent),
  0.0,
  1.0
);
float groundHeightDistanceFade = smoothstep(
  uGroundDistanceStart,
  uGroundDistanceStart + 10.0,
  groundViewDistance
);
groundHeightFactor *= groundHeightDistanceFade;
float groundMediumAmount = clamp(
  groundDistanceFog * groundHeightFactor * uGroundMaxOpacity,
  0.0,
  1.0
);
float groundOpticalDistance = max(
  groundViewDistance - uGroundDistanceStart,
  0.0
) * groundMediumAmount;
vec3 groundTransmission = exp(
  -uGroundDirectAttenuation * groundOpticalDistance
);
vec3 groundDepthIllumination = exp(
  -uGroundDepthLightAttenuation *
    groundDepthBelowSurface * groundHeightDistanceFade
);
vec3 groundScatterBuildUp = 1.0 - exp(
  -uGroundBackscatterAttenuation * groundOpticalDistance
);
gl_FragColor.rgb =
  gl_FragColor.rgb * groundTransmission * groundDepthIllumination +
  uGroundFogColor * groundScatterBuildUp * groundMediumAmount;`
      );
  };
  material.customProgramCacheKey = () => 'ground-underwater-medium-v1';
  material.needsUpdate = true;
}

function updateGroundUnderwaterMedium(material, config) {
  const uniforms = material?.userData?.underwaterMediumUniforms;
  if (!uniforms) {
    return;
  }
  uniforms.uGroundFogColor.value.copy(config.color);
  uniforms.uGroundDirectAttenuation.value.copy(config.directAttenuation);
  uniforms.uGroundBackscatterAttenuation.value.copy(config.backscatterAttenuation);
  uniforms.uGroundDepthLightAttenuation.value.copy(config.depthLightAttenuation);
  uniforms.uGroundDistanceStart.value = config.distanceStart;
  uniforms.uGroundDistanceEnd.value = config.distanceEnd;
  uniforms.uGroundDistanceExponent.value = config.distanceExponent;
  uniforms.uGroundSurfaceLevel.value = config.surfaceLevel;
  uniforms.uGroundHeightFalloff.value = config.heightFalloff;
  uniforms.uGroundHeightExponent.value = config.heightExponent;
  uniforms.uGroundMaxOpacity.value = config.maxOpacity;
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
      posRange: DEFAULT_SIMULATION_POS_RANGE,
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
    color: 0xa8c4d8, // 海中の青銀色（イワシ・アジ系小型魚の色）
    roughness: 0.25,      // 適度な光沢（ぎらつかない範囲）
    metalness: 0.35,      // 魚体の濡れた質感
    envMapIntensity: 1.0, // 環境マップ反射（控えめ）
    transparent: false,
    alphaTest: 0.5,
    map: texture,
    vertexColors: false,
    vertexColor: 0xffffff,
  });

  let boidLodMaterial = new THREE.MeshStandardMaterial({
    color: 0xa8c4d8, // 海中の青銀色
    roughness: 0.40,      // LOD は少し落ち着いた質感
    metalness: 0.25,      // 遠方は控えめな光沢
    envMapIntensity: 0.7, // 環境マップ反射（LOD は控えめ）
    transparent: false,
    alphaTest: 0.5,
    map: textureLod,
    vertexColors: false,
    vertexColor: 0xffffff,
  });

  originalMaterial = boidMaterial;
  originalMaterialLod = boidLodMaterial;
  predatorMaterial = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    roughness: 0.4,       // 捕食者も落ち着いた光沢
    metalness: 0.3,       // 大型魚の質感
    envMapIntensity: 0.8, // 環境マップ反射
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
const benchmarkMessageChannel = browserBenchmarkConfig ? new MessageChannel() : null;
if (benchmarkMessageChannel) {
  benchmarkMessageChannel.port1.onmessage = () => animate(performance.now());
}

function scheduleNextFrame() {
  if (webglContextLost || !renderer || !scene || !camera) {
    // 描画不能な場合は復旧を優先する。
    if (!renderer) {
      scheduleWebglRecovery('raf');
    }
    return;
  }
  if (browserBenchmarkConfig) {
    benchmarkMessageChannel.port2.postMessage(0);
  } else if (typeof requestAnimationFrame === "function") {
    animationTimer = requestAnimationFrame(animate);
  }
}

function animate(frameTimeMs) {
  if (webglContextLost || !renderer || !scene || !camera) {
    scheduleWebglRecovery('animate');
    return;
  }
  stats?.begin();
  const benchmarkFrameIndex = browserBenchmark?.beginFrame();
  const currentTime =
    typeof frameTimeMs === "number" ? frameTimeMs : performance.now();
  const deltaTime = (currentTime - lastTime) / 1000;
  lastTime = currentTime;

  if (!paused.value) {
    shaderTime += deltaTime;
  }
  updateInstancingMaterialUniforms(shaderTime);

  if (browserBenchmark?.shouldResetPhaseTimings()) {
    wasmBridge?.resetPhaseTimings();
  }
  const sampleLocality =
    browserBenchmarkConfig &&
    browserBenchmarkConfig.warmup > 0 &&
    benchmarkFrameIndex === browserBenchmarkConfig.warmup - 1;
  if (sampleLocality) {
    wasmBridge?.beginLocalitySample();
  }
  const simulationDelta = browserBenchmarkConfig ? 1 / 60 : deltaTime;
  const count = browserBenchmark
    ? browserBenchmark.measure('wasm_simulation', () =>
        stepSimulationAndUpdateState(simulationDelta),
      )
    : stepSimulationAndUpdateState(paused.value ? 0 : deltaTime);
  if (sampleLocality) {
    wasmBridge?.endLocalitySample();
  }

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

  const { positions, orientations, velocities } = browserBenchmark
    ? browserBenchmark.measure('wasm_to_js_views', () => getWasmViews(count))
    : getWasmViews(count);
  if ((frameCounter++ & 63) === 0) {
    wasmBridge?.getDiagnostics?.({ firstBoidX: true });
  }

  const predatorCount = getPredatorCount();
  const updateInstancing = () => boidInstancing.update({
      count,
      positions,
      orientations,
      velocities,
      cameraPosition: camera.position,
      originPosition: controls?.target,
      predatorCount,
      posRange: DEFAULT_SIMULATION_POS_RANGE,
      preserveLod: paused.value,
    });
  const updateInfo = browserBenchmark
    ? browserBenchmark.measure('js_instance_packing', updateInstancing)
    : updateInstancing();

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
  const submitRender = () => {
    if (pipelineReady) {
      fogPipeline.updateCameraUniforms(camera);
      fogPipeline.render(deltaTime);
    } else {
      renderer.render(scene, camera);
    }
  };
  if (browserBenchmark) {
    browserBenchmark.beginGpu(renderer.getContext());
    browserBenchmark.measure('render_submission_cpu', submitRender);
    browserBenchmark.endGpu();
  } else {
    submitRender();
  }

  stats?.end();
  stats?.update();

  if (browserBenchmark?.endFrame()) {
    void browserBenchmark.finish({ bridge: wasmBridge, positions, renderer });
    return;
  }

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
  // WebGL が初期化できていない端末では、復旧後にループが再開される。
  scheduleNextFrame();
}

onMounted(() => {
  // 初回マウント時にシステム調整値を WASM に反映
  if (!tuningInitialized.value) {
    updateSystemSettings(toRaw(systemSettings));
    tuningInitialized.value = true;
    applySystemSettingsToWasm();
    if (!browserBenchmarkConfig) {
      saveToStorage();
    }
  }

  initThreeJS();
  loadBoidModel(() => {
    console.log("Boid model loaded successfully.");

    boidAssetsReady = true;
    if (browserBenchmarkConfig) {
      wasmBridge?.configureBenchmarkDiagnostics({
        seed: browserBenchmarkConfig.seed,
        taskLimit: browserBenchmarkConfig.taskLimit,
        parallelTiming: true,
      });
    } else {
      applyStatsDebugState();
    }

    startSimulation();
    if (!browserBenchmarkConfig) {
      initBackgroundAudioPlayback();
    }
  });

  window.addEventListener("keydown", handleKeydown);

  // タブ/ウィンドウのアクティブ状態に応じて背景音を自動ミュートする。
  // visibilitychange: タブ切替/最小化など
  // blur/focus: アプリをアクティブにしているか
  document.addEventListener("visibilitychange", applyBackgroundAudioAutoMute);
  window.addEventListener("blur", applyBackgroundAudioAutoMute);
  window.addEventListener("focus", applyBackgroundAudioAutoMute);

  // タブ復帰時に context が失われていたら復旧を試みる。
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden) {
      if (shouldAttemptWebglRecovery()) {
        scheduleWebglRecovery('visibility');
      }
    }
  });
  window.addEventListener('focus', () => {
    if (shouldAttemptWebglRecovery()) {
      scheduleWebglRecovery('focus');
    }
  });
});

onUnmounted(() => {
  benchmarkMessageChannel?.port1.close();
  benchmarkMessageChannel?.port2.close();
  window.removeEventListener("keydown", handleKeydown);
  document.removeEventListener("visibilitychange", applyBackgroundAudioAutoMute);
  window.removeEventListener("blur", applyBackgroundAudioAutoMute);
  window.removeEventListener("focus", applyBackgroundAudioAutoMute);

  if (animationTimer && typeof cancelAnimationFrame === "function") {
    cancelAnimationFrame(animationTimer);
    animationTimer = null;
  }

  clearWebglRecoveryTimer();
  setStatsOverlayVisibility(false);
  stats = null;
  statsInitPromise = null;
  disposeRendererAndPipeline();
});

watch(
  () => debugControls.enableFogPipeline,
  () => {
    rebuildFogPipeline();
  }
);

watch(
  () => debugControls.enableEnhancedPostEffects,
  () => {
    rebuildFogPipeline();
  }
);

watch(fogTuning, applyFogTuning, { deep: true });
watch(lightingTuning, applyLightingTuning, { deep: true });

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

watch(
  () => debugControls.enableStats,
  () => {
    applyStatsDebugState();
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
    if (!browserBenchmarkConfig) {
      saveToStorage();
    }
  },
  { deep: true }
);

// 種族設定の変更を監視し、WASM 側へ反映 + 必要なら再初期化
watch(
  settings,
  () => {
    wasmBridge?.applySpeciesParams(toRaw(settings), { spatialScale: 1 });

    if (!browserBenchmarkConfig) {
      saveToStorage();
    }

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

.fog-tuning-section {
  margin: 8px 0;
  border: 1px solid rgba(255, 255, 255, 0.16);
  border-radius: 4px;
}

.debug-settings .species-section {
  width: min(430px, calc(100vw - 40px));
  min-width: 0;
}

.debug-content > .debug-checkbox {
  display: block;
  margin-bottom: 6px;
}

.fog-tuning-section > summary {
  padding: 7px;
  cursor: pointer;
}

.fog-tuning-content {
  padding: 4px 8px 10px;
}

.compact-setting-row label {
  width: 120px;
}

.compact-setting-row input[type="color"] {
  width: 38px;
  height: 26px;
  padding: 0;
  border: 0;
  background: transparent;
}

.color-value-input {
  width: 72px;
  margin-left: 8px;
}

.fog-vector-group {
  display: grid;
  grid-template-columns: 120px repeat(3, 62px);
  gap: 6px;
  align-items: center;
  margin-bottom: 8px;
}

.vector-value-input {
  min-width: 0;
  width: 100%;
  box-sizing: border-box;
}
</style>
