// filepath: d:\GitHub\wasm-boids\src\components\Settings.vue
<template>
  <div class="settings">
    <label>Speed:</label>
    <input type="range" v-model="settings.speed" min="0" max="10" step="0.01" />
    <p>Current Speed: {{ settings.speed }}</p>

    <label>Flock Size:</label>
    <input type="range" v-model="settings.flockSize" min="10" max="10000" step="1" />
    <p>Current Flock Size: {{ settings.flockSize }}</p>

    <label>Cohesion:</label>
    <input type="range" v-model="settings.cohesion" min="0" max="10" step="0.01" />
    <p>Current Cohesion: {{ settings.cohesion }}</p>

    <label>Separation:</label>
    <input type="range" v-model="settings.separation" min="0" max="10" step="0.01" />
    <p>Current Separation: {{ settings.separation }}</p>

    <label>Alignment:</label>
    <input type="range" v-model="settings.alignment" min="0" max="10" step="0.01" />
    <p>Current Alignment: {{ settings.alignment }}</p>
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
}
</style>