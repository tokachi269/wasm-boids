<template>
  <div id="app">
    <h1>Boids Simulation</h1>
    <Settings :settings="settings" />
    <div class="info">
      <p>Boids Count: {{ settings.flockSize }}</p>
    </div>
    <div ref="threeContainer" class="three-container" />
  </div>
</template>

<script setup>
import { inject, onMounted, reactive, ref, watch } from 'vue';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import Settings from './components/Settings.vue';

const wasmModule = inject('wasmModule');
if (!wasmModule) {
  console.error('wasmModule not provided');
}

const settings = reactive({
  speed: 5,
  flockSize: 2000,
  cohesion: 0.1,
  separation: 0.1,
  alignment: 0.05,
});

const threeContainer = ref(null);
let scene, camera, renderer, controls;
let boidMeshes = [];
let boidTree = null;
let boids = null; // グローバル参照用

const paused = ref(false);

function handleKeydown(e) {
  if (e.code === 'Space') {
    paused.value = !paused.value;
  }
}

function initThreeJS() {
    const width = 1600;
  const height = 1200;
  // const width = window.innerWidth;
  // const height = window.innerHeight;

  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
  camera.position.set(0, 0, 500);
  camera.lookAt(0, 0, 0);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(width, height);
  threeContainer.value.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);

  const ambientLight = new THREE.AmbientLight(0xffffff, 1.0);
  scene.add(ambientLight);

  const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
  directionalLight.position.set(0, 0, 1).normalize();
  scene.add(directionalLight);
}

function drawBoids(boids) {
  const count = boids.size();
  while (boidMeshes.length < count) {
    const geometry = new THREE.SphereGeometry(1, 4, 4);
    const material = new THREE.MeshStandardMaterial({ color: 0x00aaff });
    const mesh = new THREE.Mesh(geometry, material);
    scene.add(mesh);
    boidMeshes.push(mesh);
  }

  for (let i = 0; i < count; i++) {
    const boid = boids.get(i);
    boidMeshes[i].position.set(boid.position.x, boid.position.y, boid.position.z);
  }
}

function animate() {
  requestAnimationFrame(animate);
  if (!paused.value && boidTree) {
    boidTree.update(1.0);
    const boids = boidTree.getBoids();
    drawBoids(boids);
  }
  controls.update();
  renderer.render(scene, camera);
}

function startSimulation() {
  const BoidTree = wasmModule.BoidTree;
  const VectorBoid = wasmModule.VectorBoid;
  const Boid = wasmModule.Boid;

  boids = new VectorBoid();
  for (let i = 0; i < settings.flockSize; i++) {
    const boid = new Boid();
    boid.position = {
      x: Math.random() * 200 - 100,
      y: Math.random() * 200 - 100,
      z: Math.random() * 200 - 100,
    };
    boid.velocity = {
      x: (Math.random() - 0.5) * 0.5,
      y: (Math.random() - 0.5) * 0.5,
      z: (Math.random() - 0.5) * 0.5,
    };
    boid.acceleration = { x: 0, y: 0, z: 0 };
    boid.id = i;
    boid.stress = 0.0;

    // ここを参照にする
    boid.params = settings; // 参照を渡す

    boids.push_back(boid);
  }

  boidTree = new BoidTree();
  boidTree.build(boids, 32, 0);
  animate();
}

onMounted(() => {
  initThreeJS();

  if (
    wasmModule &&
    wasmModule.globalSpeciesParams
  ) {
    wasmModule.globalSpeciesParams.cohesion = settings.cohesion;
    wasmModule.globalSpeciesParams.separation = settings.separation;
    wasmModule.globalSpeciesParams.alignment = settings.alignment;
  }

  window.addEventListener('keydown', handleKeydown);

  startSimulation();
});

// settingsの変更をwasmModuleに反映
watch(
  () => [settings.cohesion, settings.separation, settings.alignment],
  ([cohesion, separation, alignment]) => {
    if (
      wasmModule &&
      wasmModule.globalSpeciesParams
    ) {
      wasmModule.globalSpeciesParams.cohesion = cohesion;
      wasmModule.globalSpeciesParams.separation = separation;
      wasmModule.globalSpeciesParams.alignment = alignment;
    }
  }
);
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

.three-container {
  width: 100vw;
  height: 100vh;
  display: block;
  border: 1px solid black;
  overflow: hidden;
}
</style>
