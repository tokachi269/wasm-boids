<template>
  <div id="app">
    <div class="ui-overlay">
      <h1>Boids Simulation</h1>
      <details>
        <summary>Settings</summary>
        <Settings :settings="settings" />
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
      </details>
      <div class="info">
        <p>Boids Count: {{ settings.flockSize }}</p>
      </div>
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
  flockSize: 2000,
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
  const width = window.innerWidth;
  const height = window.innerHeight;


  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a1e3a);
  scene.fog = new THREE.Fog(0x0a1e3a, 300, 900);

  camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
  camera.position.set(0, 0, 250);
  camera.lookAt(0, 0, 0);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(width, height);
  renderer.shadowMap.enabled = true; // ★影を有効化
  threeContainer.value.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);

  // 地面メッシュ追加
  const groundGeo = new THREE.PlaneGeometry(1000, 1000);
  const groundMat = new THREE.MeshStandardMaterial({ color: 0x183050, roughness: 0.8 });
  const ground = new THREE.Mesh(groundGeo, groundMat);
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -200; // 水面より下に
  ground.receiveShadow = true; // ★影を受ける
  scene.add(ground);

  // ライト
  const ambientLight = new THREE.AmbientLight(0xffffff, 1.5);
  scene.add(ambientLight);

  // 太陽光（やや暖色のDirectionalLight）
  const dirLight = new THREE.DirectionalLight(0xfff2cc, 1.3); // 暖色＆強め
  dirLight.position.set(300, 500, 200); // 高い位置から照らす
  dirLight.castShadow = true;

  // 影カメラの範囲を広げる
  dirLight.shadow.camera.left = -500;
  dirLight.shadow.camera.right = 500;
  dirLight.shadow.camera.top = 500;
  dirLight.shadow.camera.bottom = -500;
  dirLight.shadow.camera.near = 1;
  dirLight.shadow.camera.far = 2000;

  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  dirLight.shadow.bias = -0.001;
  dirLight.shadow.normalBias = 0.01;

  scene.add(dirLight);

  // ウィンドウリサイズ対応
  window.addEventListener('resize', onWindowResize);
}

function onWindowResize() {
  const width = window.innerWidth;
  const height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}

function drawBoids(boids) {
  const count = boids.size();

  // メッシュが多い場合は余分なものを削除
  while (boidMeshes.length > count) {
    const mesh = boidMeshes.pop();
    scene.remove(mesh);
  }

  // メッシュが足りない場合は追加
  while (boidMeshes.length < count) {
    const geometry = new THREE.SphereGeometry(1, 8, 8);
    const material = new THREE.MeshStandardMaterial({ color: 0x1fb5ff });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.scale.set(0.5, 0.5, 2.0);
    mesh.castShadow = true; // ★影を落とす
    scene.add(mesh);
    boidMeshes.push(mesh);
  }

  // 位置・向きを更新
  for (let i = 0; i < count; i++) {
    const boid = boids.get(i);
    boidMeshes[i].position.set(boid.position.x, boid.position.y, boid.position.z);

    const v = boid.velocity;
    const dir = new THREE.Vector3(v.x, v.y, v.z);
    if (dir.lengthSq() > 0.0001) {
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

let animationTimer = null;
const FRAME_INTERVAL = 1000 / 60; // 60FPS

function animate() {
  if (!paused.value && boidTree) {
    boidTree.update(1.0);
    const boids = boidTree.getBoids();
    drawBoids(boids);
    clearUnitVisuals();
    if (showUnits.value && boidTree.root) {
      maxDepth = calcMaxDepth(boidTree.root, 0);
      drawUnitTree(boidTree.root, 0);
    }
  }
  controls.update();
  renderer.render(scene, camera);

  // 60FPSで次のフレームを呼ぶ
  animationTimer = setTimeout(animate, FRAME_INTERVAL);
}

// 停止時はclearTimeout(animationTimer)を呼ぶと良いです

function startSimulation() {
  const BoidTree = wasmModule.BoidTree;
  const VectorBoid = wasmModule.VectorBoid;
  const Boid = wasmModule.Boid;
  // 初期化時
  boids = wasmModule.BoidTree.generateRandomBoids(settings.flockSize, 100, 0.25);
  boidTree = new BoidTree();
  boidTree.build(boids, 32, 0);
  animate();
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

// flockSizeの変更を監視
watch(
  () => settings.flockSize,
  (newSize) => {
    if (boidTree && boidTree.setFlockSize) {
      // flockSize変更時
      boidTree.setFlockSize(newSize, 100, 0.25);
    }
  }
);
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
}

.info {
  margin-top: 20px;
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
}
</style>