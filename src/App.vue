<template>
  <div id="app">
    <h1>Boids Simulation</h1>
    <Settings :settings="settings" />
    <div class="info">
      <p>Boids Count: {{ boidCount }}</p>
      <p>Simulation Time: {{ simulationTime }}s</p>
    </div>
    <canvas id="boidsCanvas" width="800" height="600"></canvas>
  </div>
</template>

<script>
import Settings from './components/Settings.vue';
import Module from './wasm/build/wasm_boids.js'; // WebAssemblyモジュールをインポート

export default {
  components: {
    Settings,
  },
  data() {
    return {
      settings: {
        speed: 5,
        flockSize: 100,
      },
      boidCount: 0,
      simulationTime: 0,
      wasmModule: null,
      boidTree: null,
    };
  },
  methods: {
    async initializeWasm() {
      if (!this.wasmModule) {
        this.wasmModule = await Module();
        this.boidTree = new this.wasmModule.BoidTree();
      }
    },
    async startSimulation() {
      await this.initializeWasm();

      const boids = new this.wasmModule.VectorBoid();
      for (let i = 0; i < this.settings.flockSize; i++) {
        const boid = new this.wasmModule.Boid();
        boid.position = { x: Math.random() * 800, y: Math.random() * 600, z: 0 };
        boid.id = i;
        boids.push_back(boid); // VectorBoid に追加
      }

      this.boidTree.build(boids); // VectorBoid を渡す
      this.drawBoids(boids);
    },
    drawBoids(boids) {
      const canvas = document.getElementById('boidsCanvas');
      const ctx = canvas.getContext('2d');
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      boids.forEach((boid) => {
        ctx.beginPath();
        ctx.arc(boid.position.x, boid.position.y, 5, 0, 2 * Math.PI);
        ctx.fillStyle = 'blue';
        ctx.fill();
        ctx.stroke();
      });
    },
  },
  mounted() {
    this.startSimulation(); // コンポーネントがマウントされたらシミュレーションを開始
  },
};
</script>

<style>
#app {
  font-family: Arial, sans-serif;
  padding: 20px;
}

.settings {
  margin-bottom: 20px;
}

.info {
  margin-top: 20px;
}

canvas {
  display: block;
  width: 100vw;
  height: 100vh;
  border: 1px solid black;
}
</style>
