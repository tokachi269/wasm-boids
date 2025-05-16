// filepath: d:\GitHub\wasm-boids\src\components\Settings.vue
<template>
  <div class="settings">
    <div class="setting-row">
      <label>群れの数:</label>
      <input type="range" v-model.number="settings.flockSize" min="10" max="10000" step="1" />
      <span>{{ settings.flockSize }}</span>
    </div>
    <div class="setting-row">
      <label>スピード:</label>
      <input type="range" v-model.number="settings.speed" min="0" max="10" step="0.01" />
      <span>{{ settings.speed }}</span>
    </div>
    <div class="setting-row">
      <label>凝集 (Cohesion):</label>
      <input type="range" v-model.number="settings.cohesion" min="0" max="10" step="0.01" />
      <span>{{ settings.cohesion }}</span>
    </div>
    <div class="setting-row">
      <label>凝集範囲 (Cohesion Range):</label>
      <input type="range" v-model.number="settings.cohesionRange" min="1" max="300" step="1" />
      <span>{{ settings.cohesionRange }}</span>
    </div>
    <div class="setting-row">
      <label>分離 (Separation):</label>
      <input type="range" v-model.number="settings.separation" min="0" max="10" step="0.01" />
      <span>{{ settings.separation }}</span>
    </div>
    <div class="setting-row">
      <label>分離範囲 (Separation Range):</label>
      <input type="range" v-model.number="settings.separationRange" min="1" max="100" step="1" />
      <span>{{ settings.separationRange }}</span>
    </div>
    <div class="setting-row">
      <label>整列 (Alignment):</label>
      <input type="range" v-model.number="settings.alignment" min="0" max="10" step="0.01" />
      <span>{{ settings.alignment }}</span>
    </div>
    <div class="setting-row">
      <label>整列範囲 (Alignment Range):</label>
      <input type="range" v-model.number="settings.alignmentRange" min="1" max="200" step="1" />
      <span>{{ settings.alignmentRange }}</span>
    </div>
    <div class="setting-row">
      <label>最大速度:</label>
      <input type="range" v-model.number="settings.maxSpeed" min="0.1" max="10" step="0.01" />
      <span>{{ settings.maxSpeed }}</span>
    </div>
    <div class="setting-row">
      <label>最大旋回角:</label>
      <input type="range" v-model.number="settings.maxTurnAngle" min="0.001" max="0.2" step="0.001" />
      <span>{{ settings.maxTurnAngle }}</span>
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
  width: 520px;
}
.setting-row {
  display: flex;
  align-items: center;
  margin-bottom: 8px;
}
.setting-row label {
  width: 230px;
  text-align: left;
  margin-right: 10px;
}
.setting-row input[type="range"] {
  flex: 1;
  margin: 0 10px;
}
.setting-row span {
  width: 60px;
  text-align: left;
  display: inline-block;
}
</style>