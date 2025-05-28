// filepath: d:\GitHub\wasm-boids\src\components\Settings.vue
<template>
  <div class="settings">
    <div class="setting-row">
      <label>群れの数<br> (Flock Size):</label>
      <input type="range" v-model.number="settings.flockSize" min="10" max="10000" step="1" />
      <span>{{ settings.flockSize }}</span>
    </div>
    <div class="setting-row">
      <label>凝集<br> (Cohesion):</label>
      <input type="range" v-model.number="settings.cohesion" min="0" max="10" step="0.01" />
      <span>{{ settings.cohesion }}</span>
    </div>
    <div class="setting-row">
      <label>凝集範囲<br> (Cohesion Range):</label>
      <input type="range" v-model.number="settings.cohesionRange" min="1" max="300" step="1" />
      <span>{{ settings.cohesionRange }}</span>
    </div>
    <div class="setting-row">
      <label>分離<br> (Separation):</label>
      <input type="range" v-model.number="settings.separation" min="0" max="10" step="0.01" />
      <span>{{ settings.separation }}</span>
    </div>
    <div class="setting-row">
      <label>分離範囲<br> (Separation Range):</label>
      <input type="range" v-model.number="settings.separationRange" min="1" max="100" step="1" />
      <span>{{ settings.separationRange }}</span>
    </div>
    <div class="setting-row">
      <label>整列<br> (Alignment):</label>
      <input type="range" v-model.number="settings.alignment" min="0" max="10" step="0.01" />
      <span>{{ settings.alignment }}</span>
    </div>
    <div class="setting-row">
      <label>整列範囲<br> (Alignment Range):</label>
      <input type="range" v-model.number="settings.alignmentRange" min="1" max="100" step="1" />
      <span>{{ settings.alignmentRange }}</span>
    </div>
    <div class="setting-row">
      <label>最大速度:<br> (Max Speed):</label>
      <input type="range" v-model.number="settings.maxSpeed" min="0.1" max="2" step="0.01" />
      <span>{{ settings.maxSpeed }}</span>
    </div>
    <div class="setting-row">
      <label>最大旋回角:<br> (Max Turn Angle):</label>
      <input type="range" v-model.number="settings.maxTurnAngle" min="0.001" max="1" step="0.001" />
      <span>{{ settings.maxTurnAngle }}</span>
    </div>
    <div class="setting-row">
      <label>最大近傍数<br>(Max Neighbors):</label>
      <input type="range" v-model.number="settings.maxNeighbors" min="1" max="32" step="1" />
      <span>{{ settings.maxNeighbors }}</span>
    </div>
    <div class="setting-row">
      <label>吸引減衰λ<br>(Lambda):</label>
      <input type="range" v-model.number="settings.lambda" min="0.001" max="0.2" step="0.001" />
      <span>{{ settings.lambda }}</span>
    </div>
    <div class="setting-row">
      <label>水平化トルク<br>(Horizontal Torque):</label>
      <input type="range" v-model.number="settings.horizontalTorque" min="0.001" max="0.2" step="0.001" />
      <span>{{ settings.horizontalTorque }}</span>
    </div>
    <div class="setting-row">
      <label>速度閾値ε<br>(Velocity Epsilon):</label>
      <input type="range" v-model.number="settings.velocityEpsilon" min="0.00001" max="0.01" step="0.00001" />
      <span>{{ settings.velocityEpsilon }}</span>
    </div>
    <div class="setting-row">
      <label>回転トルク強度<br>(Torque Strength):</label>
      <input type="range" v-model.number="settings.torqueStrength" min="0.001" max="5" step="0.001" />
      <span>{{ settings.torqueStrength }}</span>
    </div>
    
  </div>
</template>

<script>
export default {
  props: {
    settings: {
      type: Object,
      required: true,
    },
  },
};
</script>

<style scoped>
.settings {
  margin-bottom: 20px;
  width: 100%;
  min-width: 260px;
  max-width: 520px;
  box-sizing: border-box;
}
.setting-row {
  display: flex;
  align-items: center;
  margin-bottom: 8px;
  width: 100%;
}
.setting-row label {
  width: 130px;           /* ← 幅を固定して左端を揃える */
  text-align: left;
  margin-right: 10px;
  flex-shrink: 0;
  display: inline-block;
}
.setting-row input[type="range"] {
  width: 120px;           /* 全スライダーで統一 */
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
</style>