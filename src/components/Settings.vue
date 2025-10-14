// filepath: d:\GitHub\wasm-boids\src\components\Settings.vue
<template>
  <div class="settings">
    <details class="species-section" :open="false">
      <summary class="species-header">
        {{ settings.species }} ({{ settings.count }}匹)
      </summary>
      <div class="species-content">
        <div class="setting-row">
          <label>種族名<br>(Species):</label>
          <span>{{ settings.species }}</span>
        </div>
        <div class="setting-row">
          <label>群れの数(要更新)<br>(Count):</label>
          <input type="range" v-model.number="settings.count" min="0" max="20000" step="1" />
          <span 
            v-if="!editingCount" 
            class="editable-value" 
            @click="startEditCount"
            title="クリックして編集"
          >{{ settings.count }}</span>
          <input 
            v-if="editingCount"
            type="number" 
            v-model.number="settings.count" 
            min="0" 
            max="20000"
            class="value-input"
            @blur="stopEditCount"
            @keyup.enter="stopEditCount"
            ref="countInput"
          />
        </div><div class="setting-row">
      <label>凝集<br>(Cohesion):</label>
      <input type="range" v-model.number="settings.cohesion" min="0" max="40" step="0.01" />
      <span 
        v-if="!editingCohesion" 
        class="editable-value" 
        @click="startEditCohesion"
        title="クリックして編集"
      >{{ settings.cohesion }}</span>
      <input 
        v-if="editingCohesion"
        type="number" 
        v-model.number="settings.cohesion" 
        min="0" 
        max="40"
        step="0.01"
        class="value-input"
        @blur="stopEditCohesion"
        @keyup.enter="stopEditCohesion"
        ref="cohesionInput"
      />
    </div>
    <div class="setting-row">
      <label>凝集範囲<br>(Cohesion Range):</label>
      <input type="range" v-model.number="settings.cohesionRange" min="1" max="300" step="1" />
      <span 
        v-if="!editingCohesionRange" 
        class="editable-value" 
        @click="startEditCohesionRange"
        title="クリックして編集"
      >{{ settings.cohesionRange }}</span>
      <input 
        v-if="editingCohesionRange"
        type="number" 
        v-model.number="settings.cohesionRange" 
        min="1" 
        max="300"
        step="1"
        class="value-input"
        @blur="stopEditCohesionRange"
        @keyup.enter="stopEditCohesionRange"
        ref="cohesionRangeInput"
      />
    </div>
    <div class="setting-row">
      <label>分離<br>(Separation):</label>
      <input type="range" v-model.number="settings.separation" min="0" max="10" step="0.01" />
      <span 
        v-if="!editingSeparation" 
        class="editable-value" 
        @click="startEditSeparation"
        title="クリックして編集"
      >{{ settings.separation }}</span>
      <input 
        v-if="editingSeparation"
        type="number" 
        v-model.number="settings.separation" 
        min="0" 
        max="10"
        step="0.01"
        class="value-input"
        @blur="stopEditSeparation"
        @keyup.enter="stopEditSeparation"
        ref="separationInput"
      />
    </div>
    <div class="setting-row">
      <label>分離範囲<br>(Separation Range):</label>
      <input type="range" v-model.number="settings.separationRange" min="0.1" max="10" step="0.1" />
      <span 
        v-if="!editingSeparationRange" 
        class="editable-value" 
        @click="startEditSeparationRange"
        title="クリックして編集"
      >{{ settings.separationRange }}</span>
      <input 
        v-if="editingSeparationRange"
        type="number" 
        v-model.number="settings.separationRange" 
        min="0.1" 
        max="10"
        step="0.1"
        class="value-input"
        @blur="stopEditSeparationRange"
        @keyup.enter="stopEditSeparationRange"
        ref="separationRangeInput"
      />
    </div>    <div class="setting-row">
      <label>整列<br>(Alignment):</label>
      <input type="range" v-model.number="settings.alignment" min="0" max="20" step="0.01" />
      <span 
        v-if="!editingAlignment" 
        class="editable-value" 
        @click="startEditAlignment"
        title="クリックして編集"
      >{{ settings.alignment }}</span>
      <input 
        v-if="editingAlignment"
        type="number" 
        v-model.number="settings.alignment" 
        min="0" 
        max="20"
        step="0.01"
        class="value-input"
        @blur="stopEditAlignment"
        @keyup.enter="stopEditAlignment"
        ref="alignmentInput"
      />
    </div>
    <div class="setting-row">
      <label>整列範囲<br>(Alignment Range):</label>
      <input type="range" v-model.number="settings.alignmentRange" min="1" max="100" step="1" />
      <span 
        v-if="!editingAlignmentRange" 
        class="editable-value" 
        @click="startEditAlignmentRange"
        title="クリックして編集"
      >{{ settings.alignmentRange }}</span>
      <input 
        v-if="editingAlignmentRange"
        type="number" 
        v-model.number="settings.alignmentRange" 
        min="1" 
        max="100"
        step="1"
        class="value-input"
        @blur="stopEditAlignmentRange"
        @keyup.enter="stopEditAlignmentRange"
        ref="alignmentRangeInput"
      />
    </div>
    <div class="setting-row">
      <label>最大速度<br>(Max Speed):</label>
      <input type="range" v-model.number="settings.maxSpeed" min="0.1" max="2" step="0.01" />
      <span 
        v-if="!editingMaxSpeed" 
        class="editable-value" 
        @click="startEditMaxSpeed"
        title="クリックして編集"
      >{{ settings.maxSpeed }}</span>
      <input 
        v-if="editingMaxSpeed"
        type="number" 
        v-model.number="settings.maxSpeed" 
        min="0.1" 
        max="2"
        step="0.01"
        class="value-input"
        @blur="stopEditMaxSpeed"
        @keyup.enter="stopEditMaxSpeed"
        ref="maxSpeedInput"
      />
    </div>    <div class="setting-row">
      <label>最大旋回角<br>(Max Turn Angle):</label>
      <input type="range" v-model.number="settings.maxTurnAngle" min="0.001" max="0.3" step="0.001" />
      <span 
        v-if="!editingMaxTurnAngle" 
        class="editable-value" 
        @click="startEditMaxTurnAngle"
        title="クリックして編集"
      >{{ settings.maxTurnAngle }}</span>
      <input 
        v-if="editingMaxTurnAngle"
        type="number" 
        v-model.number="settings.maxTurnAngle" 
        min="0.001" 
        max="0.3"
        step="0.001"
        class="value-input"
        @blur="stopEditMaxTurnAngle"
        @keyup.enter="stopEditMaxTurnAngle"
        ref="maxTurnAngleInput"
      />
    </div>
    <div class="setting-row">
      <label>最大近傍数<br>(Max Neighbors):</label>
      <input type="range" v-model.number="settings.maxNeighbors" min="0" max="32" step="1" />
      <span 
        v-if="!editingMaxNeighbors" 
        class="editable-value" 
        @click="startEditMaxNeighbors"
        title="クリックして編集"
      >{{ settings.maxNeighbors }}</span>
      <input 
        v-if="editingMaxNeighbors"
        type="number" 
        v-model.number="settings.maxNeighbors" 
        min="0" 
        max="32"
        step="1"
        class="value-input"
        @blur="stopEditMaxNeighbors"
        @keyup.enter="stopEditMaxNeighbors"
        ref="maxNeighborsInput"
      />
    </div>
    <div class="setting-row">
      <label>水平化トルク<br>(Horizontal Torque):</label>
      <input type="range" v-model.number="settings.horizontalTorque" min="0.0" max="0.2" step="0.001" />
      <span 
        v-if="!editingHorizontalTorque" 
        class="editable-value" 
        @click="startEditHorizontalTorque"
        title="クリックして編集"
      >{{ settings.horizontalTorque }}</span>
      <input 
        v-if="editingHorizontalTorque"
        type="number" 
        v-model.number="settings.horizontalTorque" 
        min="0.0" 
        max="0.2"
        step="0.001"
        class="value-input"
        @blur="stopEditHorizontalTorque"
        @keyup.enter="stopEditHorizontalTorque"
        ref="horizontalTorqueInput"
      />
    </div>
    <div class="setting-row">
      <label>回転トルク強度<br>(Torque Strength):</label>
      <input type="range" v-model.number="settings.torqueStrength" min="0.0" max="20" step="0.001" />
      <span 
        v-if="!editingTorqueStrength" 
        class="editable-value" 
        @click="startEditTorqueStrength"
        title="クリックして編集"
      >{{ settings.torqueStrength }}</span>
      <input 
        v-if="editingTorqueStrength"
        type="number" 
        v-model.number="settings.torqueStrength" 
        min="0.0" 
        max="5"
        step="0.001"
        class="value-input"
        @blur="stopEditTorqueStrength"
        @keyup.enter="stopEditTorqueStrength"
        ref="torqueStrengthInput"
      />
    </div>
    <div class="setting-row">
      <label>減衰係数<br>(Damping Coefficient):</label>
      <input type="range" v-model.number="settings.lambda" min="0" max="1" step="0.001" />
      <span 
        v-if="!editingLambda" 
        class="editable-value" 
        @click="startEditLambda"
        title="クリックして編集"
      >{{ settings.lambda }}</span>
      <input 
        v-if="editingLambda"
        type="number" 
        v-model.number="settings.lambda" 
        min="0" 
        max="1"
        step="0.001"
        class="value-input"
        @blur="stopEditLambda"
        @keyup.enter="stopEditLambda"
        ref="lambdaInput"
      />
    </div>
    <div class="setting-row">
      <label>記憶時間<br>(Memory Time):</label>
      <input type="range" v-model.number="settings.tau" min="0" max="5" step="0.01" />
      <span 
        v-if="!editingTau" 
        class="editable-value" 
        @click="startEditTau"
        title="クリックして編集"
      >{{ settings.tau }}</span>
      <input 
        v-if="editingTau"
        type="number" 
        v-model.number="settings.tau" 
        min="0" 
        max="5"
        step="0.01"
        class="value-input"
        @blur="stopEditTau"
        @keyup.enter="stopEditTau"
        ref="tauInput"
      />
    </div>
    <div class="setting-row">
      <label>捕食者フラグ<br>(Is Predator):</label>
      <input type="checkbox" v-model="settings.isPredator" />
      <span>{{ settings.isPredator }}</span>
    </div>
      </div>
    </details>
  </div>
</template>

<script setup>
import { ref, nextTick } from 'vue';

defineProps({
  settings: {
    type: Object,
    required: true
  }
});

// 編集状態のref
const editingCount = ref(false);
const editingCohesion = ref(false);
const editingCohesionRange = ref(false);
const editingSeparation = ref(false);
const editingSeparationRange = ref(false);
const editingAlignment = ref(false);
const editingAlignmentRange = ref(false);
const editingMaxSpeed = ref(false);
const editingMaxTurnAngle = ref(false);
const editingMaxNeighbors = ref(false);
const editingHorizontalTorque = ref(false);
const editingTorqueStrength = ref(false);
const editingLambda = ref(false);
const editingTau = ref(false);

// 入力フィールドのref
const countInput = ref(null);
const cohesionInput = ref(null);
const cohesionRangeInput = ref(null);
const separationInput = ref(null);
const separationRangeInput = ref(null);
const alignmentInput = ref(null);
const alignmentRangeInput = ref(null);
const maxSpeedInput = ref(null);
const maxTurnAngleInput = ref(null);
const maxNeighborsInput = ref(null);
const horizontalTorqueInput = ref(null);
const torqueStrengthInput = ref(null);
const lambdaInput = ref(null);
const tauInput = ref(null);

// 編集開始関数
async function startEditCount() {
  editingCount.value = true;
  await nextTick();
  if (countInput.value) {
    countInput.value.focus();
    countInput.value.select();
  }
}

async function startEditCohesion() {
  editingCohesion.value = true;
  await nextTick();
  if (cohesionInput.value) {
    cohesionInput.value.focus();
    cohesionInput.value.select();
  }
}

async function startEditCohesionRange() {
  editingCohesionRange.value = true;
  await nextTick();
  if (cohesionRangeInput.value) {
    cohesionRangeInput.value.focus();
    cohesionRangeInput.value.select();
  }
}

async function startEditSeparation() {
  editingSeparation.value = true;
  await nextTick();
  if (separationInput.value) {
    separationInput.value.focus();
    separationInput.value.select();
  }
}

async function startEditSeparationRange() {
  editingSeparationRange.value = true;
  await nextTick();
  if (separationRangeInput.value) {
    separationRangeInput.value.focus();
    separationRangeInput.value.select();
  }
}

async function startEditAlignment() {
  editingAlignment.value = true;
  await nextTick();
  if (alignmentInput.value) {
    alignmentInput.value.focus();
    alignmentInput.value.select();
  }
}

async function startEditAlignmentRange() {
  editingAlignmentRange.value = true;
  await nextTick();
  if (alignmentRangeInput.value) {
    alignmentRangeInput.value.focus();
    alignmentRangeInput.value.select();
  }
}

async function startEditMaxSpeed() {
  editingMaxSpeed.value = true;
  await nextTick();
  if (maxSpeedInput.value) {
    maxSpeedInput.value.focus();
    maxSpeedInput.value.select();
  }
}

async function startEditMaxTurnAngle() {
  editingMaxTurnAngle.value = true;
  await nextTick();
  if (maxTurnAngleInput.value) {
    maxTurnAngleInput.value.focus();
    maxTurnAngleInput.value.select();
  }
}

async function startEditMaxNeighbors() {
  editingMaxNeighbors.value = true;
  await nextTick();
  if (maxNeighborsInput.value) {
    maxNeighborsInput.value.focus();
    maxNeighborsInput.value.select();
  }
}

async function startEditHorizontalTorque() {
  editingHorizontalTorque.value = true;
  await nextTick();
  if (horizontalTorqueInput.value) {
    horizontalTorqueInput.value.focus();
    horizontalTorqueInput.value.select();
  }
}

async function startEditTorqueStrength() {
  editingTorqueStrength.value = true;
  await nextTick();
  if (torqueStrengthInput.value) {
    torqueStrengthInput.value.focus();
    torqueStrengthInput.value.select();
  }
}

async function startEditLambda() {
  editingLambda.value = true;
  await nextTick();
  if (lambdaInput.value) {
    lambdaInput.value.focus();
    lambdaInput.value.select();
  }
}

async function startEditTau() {
  editingTau.value = true;
  await nextTick();
  if (tauInput.value) {
    tauInput.value.focus();
    tauInput.value.select();
  }
}

// 編集終了関数
function stopEditCount() {
  editingCount.value = false;
}

function stopEditCohesion() {
  editingCohesion.value = false;
}

function stopEditCohesionRange() {
  editingCohesionRange.value = false;
}

function stopEditSeparation() {
  editingSeparation.value = false;
}

function stopEditSeparationRange() {
  editingSeparationRange.value = false;
}

function stopEditAlignment() {
  editingAlignment.value = false;
}

function stopEditAlignmentRange() {
  editingAlignmentRange.value = false;
}

function stopEditMaxSpeed() {
  editingMaxSpeed.value = false;
}

function stopEditMaxTurnAngle() {
  editingMaxTurnAngle.value = false;
}

function stopEditMaxNeighbors() {
  editingMaxNeighbors.value = false;
}

function stopEditHorizontalTorque() {
  editingHorizontalTorque.value = false;
}

function stopEditTorqueStrength() {
  editingTorqueStrength.value = false;
}

function stopEditLambda() {
  editingLambda.value = false;
}

function stopEditTau() {
  editingTau.value = false;
}
</script>

<style scoped>
.settings {
  margin-bottom: 20px;
  width: 100%;
  min-width: 260px;
  max-width: 520px;
  box-sizing: border-box;
  pointer-events: auto;
  position: relative;
  z-index: 1;
}
.setting-row {
  display: flex;
  align-items: center;
  margin-bottom: 8px;
  width: 100%;
}
.setting-row label {
  width: 130px;
  text-align: left;
  margin-right: 10px;
  flex-shrink: 0;
  display: inline-block;
}
.setting-row input[type="range"] {
  width: 120px;
  min-width: 80px;
  max-width: 200px;
  margin: 0 10px;
  flex: none;
  display: inline-block;
}
.setting-row span {
  width: 60px;
  text-align: left;
  display: inline-block;
  flex-shrink: 0;
}

.editable-value {
  cursor: pointer;
  padding: 2px 4px;
  border-radius: 3px;
  transition: background-color 0.2s;
}

.editable-value:hover {
  background-color: rgba(255, 255, 255, 0.1);
  text-decoration: underline;
}

.value-input {
  width: 60px;
  padding: 2px 4px;
  border: 1px solid #ccc;
  border-radius: 3px;
  background: transparent;
  color: inherit;
  font-size: inherit;
  text-align: left;
}

.value-input:focus {
  outline: none;
  border-color: #007bff;
  box-shadow: 0 0 3px rgba(0, 123, 255, 0.3);
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
}

.species-header:hover {
  background-color: rgba(255, 255, 255, 0.15);
}

.species-content {
  padding: 10px;
  pointer-events: auto;
  position: relative;
  max-width: 100%;
  width: 100%;
  box-sizing: border-box;
  overflow: hidden;
}
</style>