<template>
  <div id="app">
    <div class="ui-overlay">
      <h1>Boids Simulation</h1>
      <details>
        <summary>Settings</summary>
        <Settings :settings="settings" />
        <button @click="resetSettings" style="margin-bottom:1em;">リセット</button>
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
import Stats from 'three/examples/jsm/libs/stats.module'

const wasmModule = inject('wasmModule');
if (!wasmModule) {
  console.error('wasmModule not provided');
}
const DEFAULT_SETTINGS = {
  cohesion: 3.07,
  separation: 5.27,
  alignment: 6.55,
  maxSpeed: 0.48,
  maxTurnAngle: 0.08,
  separationRange: 6,
  alignmentRange: 25,
  cohesionRange: 128,
  speed: 5,
  flockSize: 3000,
  maxNeighbors: 4,
  lambda: 0.068,
  horizontalTorque: 0.047,
  velocityEpsilon: 0.00448,
  torqueStrength: 2.477,
};

function loadSettings() {
  const saved = localStorage.getItem('boids_settings');
  if (saved) {
    try {
      return { ...DEFAULT_SETTINGS, ...JSON.parse(saved) };
    } catch {
      return { ...DEFAULT_SETTINGS };
    }
  }
  return { ...DEFAULT_SETTINGS };
}

const settings = reactive(loadSettings());

const threeContainer = ref(null);
let scene, camera, renderer, controls;
let boidTree = null;
let boids = null;

const paused = ref(false);

const showUnits = ref(true);
const showUnitSpheres = ref(false);
const showUnitLines = ref(false);
const unitLayer = ref(1);

let unitSpheres = [];
let unitLines = [];

let maxDepth = 1;
let stats = null;

let animationTimer = null;
const FRAME_INTERVAL = 1000 / 60;//1000 / 60; // 60FPS

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
  scene.fog = new THREE.Fog(0x0a1e3a, 10, 500);

  camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
  camera.position.set(20, 40, 40);
  camera.lookAt(0, 0, 0);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(width, height);
  renderer.shadowMap.enabled = true;
  threeContainer.value.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);

  // 地面メッシュ追加
  const groundGeo = new THREE.PlaneGeometry(1000, 1000);
  const groundMat = new THREE.MeshStandardMaterial({ color: 0x183050, roughness: 0.8 });
  const ground = new THREE.Mesh(groundGeo, groundMat);
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -80;
  ground.receiveShadow = true; // 影を受ける
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

let instancedMesh = null; // 使わないので削除してもOK
let boidLODs = []; // LOD用配列

// LOD用ジオメトリ・マテリアルを使い回す
const boidGeometryHigh = new THREE.SphereGeometry(1, 8, 8);
const boidGeometryLow = new THREE.SphereGeometry(1, 3, 2);
const boidMaterial = new THREE.MeshStandardMaterial({ color: 0x1fb5ff });

function initBoidLODs(count) {
  // 既存LODをクリア
  for (const lod of boidLODs) scene.remove(lod);
  boidLODs = [];

  for (let i = 0; i < count; i++) {
    const lod = new THREE.LOD();

    // 近距離: 高ポリ
    const meshHigh = new THREE.Mesh(boidGeometryHigh, boidMaterial);
    meshHigh.castShadow = true; // 影を落とす
    meshHigh.receiveShadow = true; // 影を受ける

    meshHigh.scale.set(0.5, 0.5, 2.0);
    lod.addLevel(meshHigh, 0);

    // 遠距離: 低ポリ
    const meshLow = new THREE.Mesh(boidGeometryLow, boidMaterial);
    meshLow.castShadow = true; // 影を落とす
    meshLow.receiveShadow = true; // 影を受ける
    meshLow.scale.set(0.5, 0.5, 2.0);
    lod.addLevel(meshLow, 100);

    scene.add(lod);
    boidLODs.push(lod);
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
  // スフィア: スライダで制御
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
  if (stats) stats.begin();

  if (!paused.value && boidTree) {
    boidTree.update(1.0);
    const ptr = boidTree.getPositionsPtr();
    const vptr = boidTree.getVelocitiesPtr();
    const count = boidTree.getBoidCount();
    const positions = new Float32Array(wasmModule.HEAPF32.buffer, ptr, count * 3);
    const velocities = new Float32Array(wasmModule.HEAPF32.buffer, vptr, count * 3);

    if (boidLODs.length !== count) {
      initBoidLODs(count);
    }
    for (let i = 0; i < count; i++) {
      const lod = boidLODs[i];
      lod.position.set(
        positions[i * 3 + 0],
        positions[i * 3 + 1],
        positions[i * 3 + 2]
      );
      // 進行方向に向ける
      const dir = new THREE.Vector3(
        velocities[i * 3 + 0],
        velocities[i * 3 + 1],
        velocities[i * 3 + 2]
      );
      if (dir.lengthSq() > 0.0001) {
        lod.quaternion.setFromUnitVectors(
          new THREE.Vector3(0, 0, 1),
          dir.clone().normalize()
        );
      } else {
        lod.quaternion.identity();
      }
    }

    clearUnitVisuals();
    if (showUnits.value && boidTree.root) {
      maxDepth = calcMaxDepth(boidTree.root, 0);
      drawUnitTree(boidTree.root, 0);
    }
  }
  controls.update();
  renderer.render(scene, camera);

  if (stats) stats.end();

  animationTimer = setTimeout(animate, FRAME_INTERVAL);
}

function startSimulation() {
  const BoidTree = wasmModule.BoidTree;
  // 初期化時
  boidTree = new BoidTree();
  boidTree.initializeBoids(settings.flockSize, 30, 0.25);
  boidTree.build(8, 0);
  animate();
}

onMounted(() => {
  initThreeJS();

  // stats.jsの初期化とDOM追加
  stats = new Stats();
  stats.showPanel(0);
  document.body.appendChild(stats.dom);
  // 右上に移動させる
  stats.dom.style.position = 'fixed';
  stats.dom.style.right = '0px';
  stats.dom.style.top = '0px';
  stats.dom.style.left = 'auto';
  stats.dom.style.zIndex = 1000;

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
      horizontalTorque: Number(raw.horizontalTorque),
      torqueStrength: Number(raw.torqueStrength),
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
    settings.maxNeighbors, // 追加
    settings.lambda,       // 追加
    settings.horizontalTorque,
    settings.velocityEpsilon,
    settings.torqueStrength,
  ],
  () => {
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
        maxNeighbors: Number(raw.maxNeighbors),
        lambda: Number(raw.lambda),
        horizontalTorque: Number(raw.horizontalTorque),
        velocityEpsilon: Number(raw.velocityEpsilon),
        torqueStrength: Number(raw.torqueStrength),
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
      boidTree.setFlockSize(newSize, 40, 0.25);
    }
  }
);

watch(
  settings,
  (val) => {
    localStorage.setItem('boids_settings', JSON.stringify(toRaw(val)));
  },
  { deep: true }
);

function resetSettings() {
  Object.assign(settings, DEFAULT_SETTINGS);
}
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
  pointer-events: none;
}

.ui-overlay * {
  pointer-events: auto;
}
</style>