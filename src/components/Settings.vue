// filepath: d:\GitHub\wasm-boids\src\components\Settings.vue
<template>
  <div class="settings">
    <label>Speed:</label>
    <input type="range" v-model="settings.speed" min="0" max="10" />
    <p>Current Speed: {{ settings.speed }}</p>

    <label>Flock Size:</label>
    <input type="number" v-model="settings.flockSize" min="10" max="1000" />
    <p>Current Flock Size: {{ settings.flockSize }}</p>
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