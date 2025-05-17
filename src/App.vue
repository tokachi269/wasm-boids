<template>
  <div id="app">
    <h1>Boids Simulation</h1>
    <Settings :settings="settings" />
    <div class="info">
      <p>Boids Count: {{ settings.flockSize }}</p>
    </div>
    <div>
      <label>
        <input type="checkbox" v-model="showUnits" />
        Unit可視化
      </label>
      <label style="margin-left:1em;">
        <input type="checkbox" v-model="showUnitSpheres" />
        スフィアのみ表示
      </label>
      <label style="margin-left:1em;">
        <input type="checkbox" v-model="showUnitLines" />
        線のみ表示
      </label>
      <label style="margin-left:1em;">
        表示レイヤ下限: <input type="range" min="1" max="20" v-model="unitLayer" />
        {{ unitLayer }}
      </label>
    </div>
    <div ref="threeContainer" class="three-container" />
  </div>
</template>

<script setup>
import { inject, onMounted, reactive, ref, watch, toRaw } from 'vue';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import Settings from './components/Settings.vue';

const wasmModule = inject('wasmModule');
if (!wasmModule) {
  console.error('wasmModule not provided');
}

const settings = reactive({
  cohesion: 6.8,
  separation: 2,
  alignment: 1.0,
  maxSpeed: 1.0,
  maxTurnAngle: 0.281,
  separationRange: 12.0,
  alignmentRange: 30.0,
  cohesionRange: 171.0,
  speed: 5,
  flockSize: 3000,
});

const threeContainer = ref(null);
let scene, camera, renderer, controls;
let boidMeshes = [];
let boidTree = null;
let boids = null; // グローバル参照用

const paused = ref(false);

// --- 追加: unit可視化制御 ---
const showUnits = ref(true);
const showUnitSpheres = ref(false); // 追加: デフォルトoff
const showUnitLines = ref(false);   // 追加: デフォルトoff
const unitLayer = ref(1); // デフォルト1に（最下層のみ表示）

let unitSpheres = [];
let unitLines = [];

let maxDepth = 1;

// ツリーの最大深さを計算
function calcMaxDepth(unit, depth = 0) {
  if (!unit || !unit.children || typeof unit.children.size !== 'function' || unit.children.size() === 0) {
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
  if (e.code === 'Space') {
    paused.value = !paused.value;
  }
}

function initThreeJS() {
  const width = 1600;
  const height = 1200;

  scene = new THREE.Scene();
  // 背景を青色に
  scene.background = new THREE.Color(0x0a1e3a); // 魚の色に近い深い青

  // 深くなるほど暗くする（zが大きいほど暗くなるイメージ）
  scene.fog = new THREE.Fog(0x0a1e3a, 10, 300); 

  camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
  camera.position.set(0, 0, 500);
  camera.lookAt(0, 0, 0);
  scene.fog = new THREE.Fog(0x0a1e3a, 300, 900); // 近くは青、遠くはさらに暗い青
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
    // 球を細長い楕円体に
    const geometry = new THREE.SphereGeometry(1, 8, 8);
    const material = new THREE.MeshStandardMaterial({ color: 0x00aaff });
    const mesh = new THREE.Mesh(geometry, material);
    // Z方向に2倍、X/Y方向は0.5倍で細長く
    mesh.scale.set(0.5, 0.5, 2.0);
    scene.add(mesh);
    boidMeshes.push(mesh);
  }

  for (let i = 0; i < count; i++) {
    const boid = boids.get(i);
    boidMeshes[i].position.set(boid.position.x, boid.position.y, boid.position.z);

    // 進行方向（velocity）に向けて回転
    const v = boid.velocity;
    const dir = new THREE.Vector3(v.x, v.y, v.z);
    if (dir.lengthSq() > 0.0001) {
      // Z軸正方向を進行方向に合わせる
      boidMeshes[i].quaternion.setFromUnitVectors(
        new THREE.Vector3(0, 0, 1),
        dir.clone().normalize()
      );
    }
  }
}

function clearUnitVisuals() {
  for (const mesh of unitSpheres) scene.remove(mesh);
  for (const line of unitLines) scene.remove(line);
  unitSpheres = [];
  unitLines = [];
}

// レイヤ制限付き再帰描画
function drawUnitTree(unit, layer = 0) {
  // スフィア: スライダで制御（最下層も含めて正しく描画）
  if (
    showUnitSpheres.value &&
    layer >= (maxDepth - unitLayer.value + 1) &&
    (unit.children == null || unit.children.size() === 0 || layer === maxDepth)
  ) {
    // 最下層 or 指定レイヤ以上のノード
    const color = new THREE.Color().setHSL(0.1, 1, 0.7 - 0.4 * (layer / maxDepth)); // オレンジ系グラデ
    const geometry = new THREE.SphereGeometry(unit.radius, 16, 16);
    const material = new THREE.MeshBasicMaterial({
      color: color,
      wireframe: true,
      opacity: 0.3,
      transparent: true,
    });
    const sphere = new THREE.Mesh(geometry, material);
    sphere.position.set(unit.center.x, unit.center.y, unit.center.z);
    scene.add(sphere);
    unitSpheres.push(sphere);
  }

  // 線: チェックボックスで全て表示、色は深さでグラデ
  if (showUnitLines.value && unit.children && typeof unit.children.size === 'function' && unit.children.size() > 0) {
    for (let i = 0; i < unit.children.size(); i++) {
      const child = unit.children.get(i);
      const points = [
        new THREE.Vector3(unit.center.x, unit.center.y, unit.center.z),
        new THREE.Vector3(child.center.x, child.center.y, child.center.z)
      ];
      // 線の色も深さでグラデーション
      const color = new THREE.Color().setHSL(0.35, 1, 0.7 - 0.4 * (layer / maxDepth)); // 緑系グラデ
      const lineGeometry = new THREE.BufferGeometry().setFromPoints(points);
      const line = new THREE.Line(lineGeometry, new THREE.LineBasicMaterial({ color: color }));
      scene.add(line);
      unitLines.push(line);
    }
  }

  // 再帰
  if (unit.children && typeof unit.children.size === 'function' && unit.children.size() > 0) {
    for (let i = 0; i < unit.children.size(); i++) {
      const child = unit.children.get(i);
      drawUnitTree(child, layer + 1);
    }
  }
}

function animate() {
  requestAnimationFrame(animate);
  if (!paused.value && boidTree) {
    boidTree.update(1.0);
    const boids = boidTree.getBoids();
    drawBoids(boids);
    clearUnitVisuals();
    if (showUnits.value && boidTree.root) {
      maxDepth = calcMaxDepth(boidTree.root, 0); // 毎フレーム計算でもOK
      drawUnitTree(boidTree.root, 0);
    }
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

function updateParamsFromUI(settings) {
  const params = new Module.SpeciesParams();
  params.cohesion = settings.cohesion;
  params.separation = settings.separation;
  params.alignment = settings.alignment;
  params.maxSpeed = settings.maxSpeed;
  params.maxTurnAngle = settings.maxTurnAngle;
  params.separationRange = settings.separationRange;
  params.alignmentRange = settings.alignmentRange;
  params.cohesionRange = settings.cohesionRange;
  Module.setGlobalSpeciesParams(params);
}

onMounted(() => {
  initThreeJS();

  // 初期値をwasmに反映
  if (
    wasmModule &&
    wasmModule.setGlobalSpeciesParamsFromJS
  ) {
    const raw = toRaw(settings);
    wasmModule.setGlobalSpeciesParamsFromJS({
      cohesion: Number(raw.cohesion),
      separation: Number(raw.separation),
      alignment: Number(raw.alignment),
      maxSpeed: Number(raw.maxSpeed),
      maxTurnAngle: Number(raw.maxTurnAngle),
      separationRange: Number(raw.separationRange),
      alignmentRange: Number(raw.alignmentRange),
      cohesionRange: Number(raw.cohesionRange),
    });
  }

  window.addEventListener('keydown', handleKeydown);

  startSimulation();
});

// settingsの変更をwasmModuleに反映
watch(
  () => [
    settings.cohesion,
    settings.separation,
    settings.alignment,
    settings.maxSpeed,
    settings.maxTurnAngle,
    settings.separationRange,
    settings.alignmentRange,
    settings.cohesionRange,
  ],
  () => {
    if (
      wasmModule &&
      wasmModule.setGlobalSpeciesParamsFromJS
    ) {
      // プレーンなオブジェクトを作る
      const raw = toRaw(settings);
      console.log('setGlobalSpeciesParamsFromJSに渡す値', raw);

      // 値がundefinedでないかチェック
      if (
        [
          raw.cohesion, raw.separation, raw.alignment, raw.maxSpeed,
          raw.maxTurnAngle, raw.separationRange, raw.alignmentRange, raw.cohesionRange
        ].some(v => typeof v === 'undefined')
      ) {
        console.error('パラメータがundefinedです', raw);
        return;
      }
      wasmModule.setGlobalSpeciesParamsFromJS({
        cohesion: Number(raw.cohesion),
        separation: Number(raw.separation),
        alignment: Number(raw.alignment),
        maxSpeed: Number(raw.maxSpeed),
        maxTurnAngle: Number(raw.maxTurnAngle),
        separationRange: Number(raw.separationRange),
        alignmentRange: Number(raw.alignmentRange),
        cohesionRange: Number(raw.cohesionRange),
      });
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
