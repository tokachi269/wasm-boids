<template>
  <div class="settings">
    <details class="species-section" :open="false">
      <summary class="species-header">
        <span class="species-title">{{ settings.species }} ({{ settings.count }}匹)</span>
        <button
          v-if="canRemove"
          class="species-remove"
          type="button"
          @click.stop="emitRemove"
        >削除</button>
      </summary>

      <div class="species-content">
        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.species">
          <label :title="settingHelp.species">種族名<br />(Species):</label>
          <span :title="settingHelp.species">{{ settings.species }}</span>
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.count">
          <label :title="settingHelp.count">群れの数(要更新)<br />(Count):</label>
          <input
            type="range"
            v-model.number="countDraft"
            min="0"
            max="50000"
            step="1"
            @change="commitCountFromSlider"
            :title="settingHelp.count"
          />
          <span
            v-if="!editingCount"
            class="editable-value"
            @click="startEditCount"
            :title="settingHelp.count + '（クリックして編集）'"
          >{{ settings.count }}</span>
          <input
            v-else
            ref="countInput"
            class="value-input"
            type="number"
            v-model.number="countDraft"
            min="0"
            max="20000"
            step="1"
            @blur="cancelCountEdit"
            @keyup.enter="commitCountFromInput"
            :title="settingHelp.count"
          />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.cohesion">
          <label :title="settingHelp.cohesion">凝集<br />(Cohesion):</label>
          <input type="range" v-model.number="settings.cohesion" min="0" max="40" step="0.01" :title="settingHelp.cohesion" />
          <span v-if="!editingCohesion" class="editable-value" @click="startEditCohesion" :title="settingHelp.cohesion + '（クリックして編集）'">{{ settings.cohesion }}</span>
          <input v-else ref="cohesionInput" class="value-input" type="number" v-model.number="settings.cohesion" min="0" max="40" step="0.01" @blur="stopEditCohesion" @keyup.enter="stopEditCohesion" :title="settingHelp.cohesion" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.cohesionRange">
          <label :title="settingHelp.cohesionRange">凝集範囲<br />(Cohesion Range):</label>
          <input type="range" v-model.number="settings.cohesionRange" min="1" max="300" step="1" :title="settingHelp.cohesionRange" />
          <span v-if="!editingCohesionRange" class="editable-value" @click="startEditCohesionRange" :title="settingHelp.cohesionRange + '（クリックして編集）'">{{ settings.cohesionRange }}</span>
          <input v-else ref="cohesionRangeInput" class="value-input" type="number" v-model.number="settings.cohesionRange" min="1" max="300" step="1" @blur="stopEditCohesionRange" @keyup.enter="stopEditCohesionRange" :title="settingHelp.cohesionRange" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.separation">
          <label :title="settingHelp.separation">分離<br />(Separation):</label>
          <input type="range" v-model.number="settings.separation" min="0" max="10" step="0.01" :title="settingHelp.separation" />
          <span v-if="!editingSeparation" class="editable-value" @click="startEditSeparation" :title="settingHelp.separation + '（クリックして編集）'">{{ settings.separation }}</span>
          <input v-else ref="separationInput" class="value-input" type="number" v-model.number="settings.separation" min="0" max="10" step="0.01" @blur="stopEditSeparation" @keyup.enter="stopEditSeparation" :title="settingHelp.separation" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.separationRange">
          <label :title="settingHelp.separationRange">分離範囲<br />(Separation Range):</label>
          <input type="range" v-model.number="settings.separationRange" min="0.1" max="10" step="0.1" :title="settingHelp.separationRange" />
          <span v-if="!editingSeparationRange" class="editable-value" @click="startEditSeparationRange" :title="settingHelp.separationRange + '（クリックして編集）'">{{ settings.separationRange }}</span>
          <input v-else ref="separationRangeInput" class="value-input" type="number" v-model.number="settings.separationRange" min="0.1" max="10" step="0.1" @blur="stopEditSeparationRange" @keyup.enter="stopEditSeparationRange" :title="settingHelp.separationRange" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.alignment">
          <label :title="settingHelp.alignment">整列<br />(Alignment):</label>
          <input type="range" v-model.number="settings.alignment" min="0" max="20" step="0.01" :title="settingHelp.alignment" />
          <span v-if="!editingAlignment" class="editable-value" @click="startEditAlignment" :title="settingHelp.alignment + '（クリックして編集）'">{{ settings.alignment }}</span>
          <input v-else ref="alignmentInput" class="value-input" type="number" v-model.number="settings.alignment" min="0" max="20" step="0.01" @blur="stopEditAlignment" @keyup.enter="stopEditAlignment" :title="settingHelp.alignment" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.alignmentRange">
          <label :title="settingHelp.alignmentRange">整列範囲<br />(Alignment Range):</label>
          <input type="range" v-model.number="settings.alignmentRange" min="1" max="100" step="1" :title="settingHelp.alignmentRange" />
          <span v-if="!editingAlignmentRange" class="editable-value" @click="startEditAlignmentRange" :title="settingHelp.alignmentRange + '（クリックして編集）'">{{ settings.alignmentRange }}</span>
          <input v-else ref="alignmentRangeInput" class="value-input" type="number" v-model.number="settings.alignmentRange" min="1" max="100" step="1" @blur="stopEditAlignmentRange" @keyup.enter="stopEditAlignmentRange" :title="settingHelp.alignmentRange" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.predatorAlertRadius">
          <label :title="settingHelp.predatorAlertRadius">逃避開始距離<br />(Predator Alert):</label>
          <input type="range" v-model.number="settings.predatorAlertRadius" min="0" max="10" step="0.05" :title="settingHelp.predatorAlertRadius" />
          <span v-if="!editingPredatorAlertRadius" class="editable-value" @click="startEditPredatorAlertRadius" :title="settingHelp.predatorAlertRadius + '（クリックして編集）'">{{ settings.predatorAlertRadius }}</span>
          <input v-else ref="predatorAlertRadiusInput" class="value-input" type="number" v-model.number="settings.predatorAlertRadius" min="0" max="20" step="0.05" @blur="stopEditPredatorAlertRadius" @keyup.enter="stopEditPredatorAlertRadius" :title="settingHelp.predatorAlertRadius" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.densityReturnStrength">
          <label :title="settingHelp.densityReturnStrength">密度復帰強度<br />(Density Return):</label>
          <input type="range" v-model.number="settings.densityReturnStrength" min="0" max="80" step="0.5" :title="settingHelp.densityReturnStrength" />
          <span v-if="!editingDensityReturnStrength" class="editable-value" @click="startEditDensityReturnStrength" :title="settingHelp.densityReturnStrength + '（クリックして編集）'">{{ settings.densityReturnStrength }}</span>
          <input v-else ref="densityReturnStrengthInput" class="value-input" type="number" v-model.number="settings.densityReturnStrength" min="0" max="120" step="0.5" @blur="stopEditDensityReturnStrength" @keyup.enter="stopEditDensityReturnStrength" :title="settingHelp.densityReturnStrength" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.maxSpeed">
          <label :title="settingHelp.maxSpeed">最大速度<br />(Max Speed):</label>
          <input type="range" v-model.number="settings.maxSpeed" min="0.1" max="2" step="0.01" :title="settingHelp.maxSpeed" />
          <span v-if="!editingMaxSpeed" class="editable-value" @click="startEditMaxSpeed" :title="settingHelp.maxSpeed + '（クリックして編集）'">{{ settings.maxSpeed }}</span>
          <input v-else ref="maxSpeedInput" class="value-input" type="number" v-model.number="settings.maxSpeed" min="0.1" max="2" step="0.01" @blur="stopEditMaxSpeed" @keyup.enter="stopEditMaxSpeed" :title="settingHelp.maxSpeed" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.maxTurnAngle">
          <label :title="settingHelp.maxTurnAngle">最大旋回速度<br />(Max Turn Rate, rad/s):</label>
          <input type="range" v-model.number="settings.maxTurnAngle" min="0" max="30" step="0.1" :title="settingHelp.maxTurnAngle" />
          <span v-if="!editingMaxTurnAngle" class="editable-value" @click="startEditMaxTurnAngle" :title="settingHelp.maxTurnAngle + '（クリックして編集）'">{{ settings.maxTurnAngle }}</span>
          <input v-else ref="maxTurnAngleInput" class="value-input" type="number" v-model.number="settings.maxTurnAngle" min="0" max="60" step="0.1" @blur="stopEditMaxTurnAngle" @keyup.enter="stopEditMaxTurnAngle" :title="settingHelp.maxTurnAngle" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.maxNeighbors">
          <label :title="settingHelp.maxNeighbors">最大近傍数<br />(Max Neighbors):</label>
          <input type="range" v-model.number="settings.maxNeighbors" min="0" max="32" step="1" :title="settingHelp.maxNeighbors" />
          <span v-if="!editingMaxNeighbors" class="editable-value" @click="startEditMaxNeighbors" :title="settingHelp.maxNeighbors + '（クリックして編集）'">{{ settings.maxNeighbors }}</span>
          <input v-else ref="maxNeighborsInput" class="value-input" type="number" v-model.number="settings.maxNeighbors" min="0" max="32" step="1" @blur="stopEditMaxNeighbors" @keyup.enter="stopEditMaxNeighbors" :title="settingHelp.maxNeighbors" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.horizontalTorque">
          <label :title="settingHelp.horizontalTorque">水平化トルク<br />(Horizontal Torque):</label>
          <input type="range" v-model.number="settings.horizontalTorque" min="0.0" max="0.2" step="0.001" :title="settingHelp.horizontalTorque" />
          <span v-if="!editingHorizontalTorque" class="editable-value" @click="startEditHorizontalTorque" :title="settingHelp.horizontalTorque + '（クリックして編集）'">{{ settings.horizontalTorque }}</span>
          <input v-else ref="horizontalTorqueInput" class="value-input" type="number" v-model.number="settings.horizontalTorque" min="0.0" max="0.2" step="0.001" @blur="stopEditHorizontalTorque" @keyup.enter="stopEditHorizontalTorque" :title="settingHelp.horizontalTorque" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.torqueStrength">
          <label :title="settingHelp.torqueStrength">回転トルク強度<br />(Torque Strength):</label>
          <input type="range" v-model.number="settings.torqueStrength" min="0.0" max="20" step="0.001" :title="settingHelp.torqueStrength" />
          <span v-if="!editingTorqueStrength" class="editable-value" @click="startEditTorqueStrength" :title="settingHelp.torqueStrength + '（クリックして編集）'">{{ settings.torqueStrength }}</span>
          <input v-else ref="torqueStrengthInput" class="value-input" type="number" v-model.number="settings.torqueStrength" min="0.0" max="5" step="0.001" @blur="stopEditTorqueStrength" @keyup.enter="stopEditTorqueStrength" :title="settingHelp.torqueStrength" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.lambda">
          <label :title="settingHelp.lambda">減衰係数<br />(Damping Coefficient):</label>
          <input type="range" v-model.number="settings.lambda" min="0" max="1" step="0.001" :title="settingHelp.lambda" />
          <span v-if="!editingLambda" class="editable-value" @click="startEditLambda" :title="settingHelp.lambda + '（クリックして編集）'">{{ settings.lambda }}</span>
          <input v-else ref="lambdaInput" class="value-input" type="number" v-model.number="settings.lambda" min="0" max="1" step="0.001" @blur="stopEditLambda" @keyup.enter="stopEditLambda" :title="settingHelp.lambda" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.tau">
          <label :title="settingHelp.tau">記憶時間<br />(Memory Time):</label>
          <input type="range" v-model.number="settings.tau" min="0" max="5" step="0.01" :title="settingHelp.tau" />
          <span v-if="!editingTau" class="editable-value" @click="startEditTau" :title="settingHelp.tau + '（クリックして編集）'">{{ settings.tau }}</span>
          <input v-else ref="tauInput" class="value-input" type="number" v-model.number="settings.tau" min="0" max="5" step="0.01" @blur="stopEditTau" @keyup.enter="stopEditTau" :title="settingHelp.tau" />
        </div>

        <div class="setting-row tooltip-target" :data-tooltip="settingHelp.isPredator">
          <label :title="settingHelp.isPredator">捕食者フラグ<br />(Is Predator):</label>
          <input type="checkbox" v-model="settings.isPredator" :title="settingHelp.isPredator" />
          <span :title="settingHelp.isPredator">{{ settings.isPredator }}</span>
        </div>
      </div>
    </details>
  </div>
</template>

<script setup>
import { nextTick, ref, watch } from 'vue';

// 親(App.vue)から渡された種族設定オブジェクト。
// ここでは v-model で直接書き換える（親が配列で保持しているため反映される）。
const props = defineProps({
  settings: {
    type: Object,
    required: true,
  },
  canRemove: {
    type: Boolean,
    default: true,
  },
});

const emit = defineEmits(['remove']);

function emitRemove() {
  emit('remove');
}

// 各設定の説明（ユーザ目線）。ホバー時に title として表示する。
// NOTE: 実装詳細ではなく「何が変わるか/どう触るか」を優先して書く。
const settingHelp = {
  species: 'この種族の表示名です。',
  count: 'この種族の個体数です。増やすほど重くなります。反映にはリセットが必要です（要更新）。',
  cohesion: '仲間の中心へ寄る強さです。大きいほど群れが固まりやすくなります。',
  cohesionRange: '凝集を効かせる距離です。遠くの仲間まで意識すると大きくまとまりやすくなります。',
  separation: '近づきすぎたときに離れる強さです。大きいほど衝突しにくく、散りやすくなります。',
  separationRange: '分離を始める距離です。大きいほど早めに距離を取ります。',
  alignment: '進行方向を揃える強さです。大きいほど同じ向きに泳ぎます。',
  alignmentRange: '整列を効かせる距離です。近い仲間だけを見るか、少し遠くまで見るかが変わります。',
  predatorAlertRadius: '捕食者に反応し始める距離です。大きいほど早めに逃げます。',
  densityReturnStrength: '群れの密度が崩れたときに、元の密度へ戻ろうとする強さです。',
  maxSpeed: '移動速度の上限です。',
  maxTurnAngle: '向きを変える速さの上限です（rad/s）。大きいほどキビキビ曲がります。',
  maxNeighbors: '近傍として見る最大数です。小さいと局所的、大きいと全体的な動きになります。',
  horizontalTorque: '上下方向の傾きを水平へ戻す強さです。',
  torqueStrength: '回転の反応の強さです。',
  lambda: '動きの減衰（抵抗）です。大きいほど勢いが落ちやすくなります。',
  tau: '過去の状態をどれくらい残すか（記憶時間）です。大きいほど反応が遅れて滑らかになります。',
  isPredator: 'この種族を捕食者として扱います（他の種族が避ける対象になります）。',
};

// count は「要更新（リセットで反映）」なので、操作の感触を保ちつつ draft を使う。
const countDraft = ref(props.settings.count);
const editingCount = ref(false);
const countInput = ref(null);

watch(
  () => props.settings.count,
  (newCount) => {
    // 親側から値が変わった場合（リセット等）は、編集していないときだけ同期する。
    if (!editingCount.value) {
      countDraft.value = newCount;
    }
  }
);

function startEditCount() {
  editingCount.value = true;
  countDraft.value = props.settings.count;
  nextTick(() => {
    countInput.value?.focus?.();
  });
}

function commitCountFromSlider() {
  props.settings.count = countDraft.value;
}

function commitCountFromInput() {
  props.settings.count = countDraft.value;
  editingCount.value = false;
}

function cancelCountEdit() {
  editingCount.value = false;
  countDraft.value = props.settings.count;
}

// クリック編集の共通ヘルパ。
function startEditNumber(editingFlag, inputRef) {
  editingFlag.value = true;
  nextTick(() => {
    inputRef.value?.focus?.();
  });
}

function stopEditNumber(editingFlag) {
  editingFlag.value = false;
}

const editingCohesion = ref(false);
const cohesionInput = ref(null);
function startEditCohesion() { startEditNumber(editingCohesion, cohesionInput); }
function stopEditCohesion() { stopEditNumber(editingCohesion); }

const editingCohesionRange = ref(false);
const cohesionRangeInput = ref(null);
function startEditCohesionRange() { startEditNumber(editingCohesionRange, cohesionRangeInput); }
function stopEditCohesionRange() { stopEditNumber(editingCohesionRange); }

const editingSeparation = ref(false);
const separationInput = ref(null);
function startEditSeparation() { startEditNumber(editingSeparation, separationInput); }
function stopEditSeparation() { stopEditNumber(editingSeparation); }

const editingSeparationRange = ref(false);
const separationRangeInput = ref(null);
function startEditSeparationRange() { startEditNumber(editingSeparationRange, separationRangeInput); }
function stopEditSeparationRange() { stopEditNumber(editingSeparationRange); }

const editingAlignment = ref(false);
const alignmentInput = ref(null);
function startEditAlignment() { startEditNumber(editingAlignment, alignmentInput); }
function stopEditAlignment() { stopEditNumber(editingAlignment); }

const editingAlignmentRange = ref(false);
const alignmentRangeInput = ref(null);
function startEditAlignmentRange() { startEditNumber(editingAlignmentRange, alignmentRangeInput); }
function stopEditAlignmentRange() { stopEditNumber(editingAlignmentRange); }

const editingPredatorAlertRadius = ref(false);
const predatorAlertRadiusInput = ref(null);
function startEditPredatorAlertRadius() { startEditNumber(editingPredatorAlertRadius, predatorAlertRadiusInput); }
function stopEditPredatorAlertRadius() { stopEditNumber(editingPredatorAlertRadius); }

const editingDensityReturnStrength = ref(false);
const densityReturnStrengthInput = ref(null);
function startEditDensityReturnStrength() { startEditNumber(editingDensityReturnStrength, densityReturnStrengthInput); }
function stopEditDensityReturnStrength() { stopEditNumber(editingDensityReturnStrength); }

const editingMaxSpeed = ref(false);
const maxSpeedInput = ref(null);
function startEditMaxSpeed() { startEditNumber(editingMaxSpeed, maxSpeedInput); }
function stopEditMaxSpeed() { stopEditNumber(editingMaxSpeed); }

const editingMaxTurnAngle = ref(false);
const maxTurnAngleInput = ref(null);
function startEditMaxTurnAngle() { startEditNumber(editingMaxTurnAngle, maxTurnAngleInput); }
function stopEditMaxTurnAngle() { stopEditNumber(editingMaxTurnAngle); }

const editingMaxNeighbors = ref(false);
const maxNeighborsInput = ref(null);
function startEditMaxNeighbors() { startEditNumber(editingMaxNeighbors, maxNeighborsInput); }
function stopEditMaxNeighbors() { stopEditNumber(editingMaxNeighbors); }

const editingHorizontalTorque = ref(false);
const horizontalTorqueInput = ref(null);
function startEditHorizontalTorque() { startEditNumber(editingHorizontalTorque, horizontalTorqueInput); }
function stopEditHorizontalTorque() { stopEditNumber(editingHorizontalTorque); }

const editingTorqueStrength = ref(false);
const torqueStrengthInput = ref(null);
function startEditTorqueStrength() { startEditNumber(editingTorqueStrength, torqueStrengthInput); }
function stopEditTorqueStrength() { stopEditNumber(editingTorqueStrength); }

const editingLambda = ref(false);
const lambdaInput = ref(null);
function startEditLambda() { startEditNumber(editingLambda, lambdaInput); }
function stopEditLambda() { stopEditNumber(editingLambda); }

const editingTau = ref(false);
const tauInput = ref(null);
function startEditTau() { startEditNumber(editingTau, tauInput); }
function stopEditTau() { stopEditNumber(editingTau); }
</script>

<style scoped>
.settings {
  pointer-events: auto;
}

.species-section {
  border: 1px solid rgba(255, 255, 255, 0.2);
  border-radius: 5px;
  margin-bottom: 10px;
  background-color: rgba(255, 255, 255, 0.05);
  pointer-events: auto;
  position: relative;
  overflow: visible;
  max-width: 100%;
  width: fit-content;
  min-width: 260px;
}

.species-header {
  padding: 10px;
  font-weight: bold;
  cursor: pointer;
  background-color: rgba(255, 255, 255, 0.1);
  border-radius: 5px 5px 0 0;
  user-select: none;
  pointer-events: auto;
  max-width: 100%;
  width: 100%;
  box-sizing: border-box;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 8px;
}

.species-header:hover {
  background-color: rgba(255, 255, 255, 0.15);
}

.species-title {
  flex: 1;
}

.species-remove {
  pointer-events: auto;
  background: #d9534f;
  color: #fff;
  border: none;
  border-radius: 4px;
  padding: 4px 8px;
  font-size: 0.8rem;
  cursor: pointer;
}

.species-remove:hover {
  background: #c9302c;
}

.species-content {
  padding: 10px;
  pointer-events: auto;
  position: relative;
  max-width: 100%;
  width: 100%;
  box-sizing: border-box;
  /*
    CSS疑似要素のツールチップ（.tooltip-target:hover::after）を
    設定パネル内で見切れないようにする。
  */
  overflow: visible;
}

.setting-row {
  display: flex;
  align-items: center;
  margin-bottom: 8px;
  width: 100%;
  /* ツールチップが行の外にはみ出せるようにする */
  overflow: visible;
}

.setting-row label {
  width: 150px;
  text-align: left;
  margin-right: 10px;
  flex-shrink: 0;
  line-height: 1.3;
}

.setting-row input[type="range"] {
  width: 140px;
  min-width: 100px;
  max-width: 220px;
  margin: 0 10px;
}

.value-input {
  width: 70px;
  padding: 2px 4px;
  border: 1px solid #ccc;
  border-radius: 3px;
  background: transparent;
  color: inherit;
  font-size: inherit;
}

.value-input:focus {
  outline: none;
  border-color: #007bff;
  box-shadow: 0 0 3px rgba(0, 123, 255, 0.3);
}

.editable-value {
  width: 70px;
  padding: 2px 4px;
  border: 1px dashed rgba(255, 255, 255, 0.35);
  border-radius: 3px;
  cursor: pointer;
  text-align: right;
  user-select: none;
}
</style>
