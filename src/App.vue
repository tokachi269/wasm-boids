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
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { SSAOPass } from 'three/examples/jsm/postprocessing/SSAOPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";

const wasmModule = inject('wasmModule');
if (!wasmModule) {
  console.error('wasmModule not provided');
}

const posPtr = wasmModule.cwrap('posPtr', 'number', [])
const velPtr = wasmModule.cwrap('velPtr', 'number', [])
const oriPtr = wasmModule.cwrap('oriPtr', 'number', [])
const boidCount = wasmModule.cwrap('boidCount', 'number', [])
const initBoids = wasmModule.cwrap('initBoids', 'void', ['number', 'number', 'number'])
const build = wasmModule.cwrap('build', 'void', ['number', 'number'])
const update = wasmModule.cwrap('update', 'void', ['number'])
const setFlockSize = wasmModule.cwrap('setFlockSize', 'void', ['number', 'number', 'number'])
const exportTreeStructure = wasmModule.cwrap('exportTreeStructure', 'object', []);
// const getUnitCount = wasmModule.cwrap('getUnitCount', 'number', []);
// const getUnitCentersPtr = wasmModule.cwrap('getUnitCentersPtr', 'number', []);
// const getUnitParentIndicesPtr = wasmModule.cwrap('getUnitParentIndicesPtr', 'number', []);

function fetchTreeStructure() {
  const treeData = exportTreeStructure();
  return treeData;
}

const DEFAULT_SETTINGS = {
  flockSize: 5118,         // 群れの数
  cohesion: 9.04,          // 凝集
  cohesionRange: 140,      // 凝集範囲
  separation: 10,          // 分離
  separationRange: 6,      // 分離範囲
  alignment: 8.5,          // 整列
  alignmentRange: 35,      // 整列範囲
  maxSpeed: 0.27,          // 最大速度
  maxTurnAngle: 0.047,     // 最大旋回角
  maxNeighbors: 4,         // 最大近傍数
  lambda: 0.109,           // 吸引減衰 λ
  horizontalTorque: 0.004, // 水平化トルク
  velocityEpsilon: 0.004,  // 速度閾値 ε
  torqueStrength: 3.398    // 回転トルク強度
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
let scene, camera, renderer, controls, composer;

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
  scene.background = new THREE.Color(0x193255);
  scene.fog = new THREE.Fog(0x193255, 0, 28);

  camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
  camera.position.set(4, 7, 7);
  camera.lookAt(0, 0, 0);

  renderer = new THREE.WebGLRenderer({
    antialias: true,
      depth: true, // 深度バッファを有効化

  });
  renderer.setPixelRatio(window.devicePixelRatio); // 高DPI対応
  renderer.setSize(width, height);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap; // 影を柔らかく

  threeContainer.value.appendChild(renderer.domElement);

  camera.aspect = width / height;
  camera.updateProjectionMatrix();

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; // なめらかな操作
  controls.dampingFactor = 0.1;

  // 地面メッシュ追加
  const groundGeo = new THREE.PlaneGeometry(1000, 1000);
  const groundMat = new THREE.MeshStandardMaterial({ color: 0x183050, roughness: 0.8,depthTest: true });
  const ground = new THREE.Mesh(groundGeo, groundMat);
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -10;
  ground.receiveShadow = true; // 影を受ける
  scene.add(ground);

  // ライト
  const ambientLight = new THREE.AmbientLight(0xa2b7d4, 1);
  scene.add(ambientLight);

  // 太陽光（やや暖色のDirectionalLight）
  const dirLight = new THREE.DirectionalLight(0x88a5cf, 15); // 暖色＆強め
  dirLight.position.set(300, 500, 200); // 高い位置から照らす
  dirLight.castShadow = true;

  // 影カメラの範囲を広げる
  dirLight.shadow.camera.left = -100;
  dirLight.shadow.camera.right = 100;
  dirLight.shadow.camera.top = 100;
  dirLight.shadow.camera.bottom = -100;
  dirLight.shadow.camera.near = 1;
  dirLight.shadow.camera.far = 1000;

  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  dirLight.shadow.bias = -0.001;
  dirLight.shadow.normalBias = 0.01;

  scene.add(dirLight);
  // EffectComposer の初期化（スマホ以外の場合のみ）
  if (!isMobileDevice()) {
    // EffectComposer の初期化
    composer = new EffectComposer(renderer);

    // RenderPass を追加
    const renderPass = new RenderPass(scene, camera);
    composer.addPass(renderPass);

    const ssaoPass = new SSAOPass(scene, camera, width, height);
    ssaoPass.kernelRadius = 8; // サンプル半径（大きくすると効果が広がる）
    ssaoPass.minDistance = 0.001; // 最小距離（小さくすると近距離の効果が強調される）
    ssaoPass.maxDistance = 0.01; // 最大距離（大きくすると遠距離の効果が強調される）
    composer.addPass(ssaoPass);

    // UnrealBloomPass を追加（任意）
    const bloomPass = new UnrealBloomPass(new THREE.Vector2(width, height), 1.5, 0.4, 0.85);
    composer.addPass(bloomPass);
  }
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

let instancedMeshHigh = null; // 高ポリゴン用
let instancedMeshLow = null;  // 低ポリゴン用

// LOD用ジオメトリ・マテリアルを使い回す
const boidGeometryHigh = new THREE.SphereGeometry(1, 8, 8);
const boidGeometryLow = new THREE.SphereGeometry(1, 3, 2);
const boidMaterial = new THREE.MeshStandardMaterial({ color: 0x1fb5ff });
boidGeometryHigh.scale(0.5, 0.5, 2.0); // 少し小さくする
boidGeometryLow.scale(0.5, 0.5, 2.0); // 少し小さくする
let boidModel = null; // 読み込んだモデルを保持
let boidModelLod = null; // 読み込んだモデルを保持

function initInstancedBoids(count) {
  if (!boidModel) {
    console.error('Boid model not loaded yet.');
    return;
  }

  if (!boidModel.children || !boidModel.children[0]) {
    console.error('Boid model does not have valid children.');
    return;
  }

  if (instancedMeshHigh) {
    scene.remove(instancedMeshHigh);
  }
  if (instancedMeshLow) {
    scene.remove(instancedMeshLow);
  }

  const dummy = new THREE.Object3D();

  // 高ポリゴンメッシュ
  instancedMeshHigh = new THREE.InstancedMesh(
    boidModel.children[0].geometry,
    boidModel.children[0].material,
    count
  );
  instancedMeshHigh.castShadow = true;
  instancedMeshHigh.receiveShadow = true;

  // 低ポリゴンメッシュ
  instancedMeshLow = new THREE.InstancedMesh(
    boidModelLod.children[0].geometry,
    boidModelLod.children[0].material,
    count
  );
  instancedMeshLow.castShadow = true;
  instancedMeshLow.receiveShadow = true;

  scene.add(instancedMeshHigh);
  scene.add(instancedMeshLow);
}

function loadBoidModel(callback) {
  const loader = new GLTFLoader();
  const basePath = process.env.BASE_URL || '/'; // publicPath を取得
  loader.load(
    `./models/boidModel.glb`, // モデルのパス
    (gltf) => {
      boidModel = gltf.scene;
      callback();
    },
    undefined,
    (error) => {
      console.error('An error occurred while loading the model:', error);
    }

    
  );
    loader.load(
    `./models/boidModel_lod.glb`, // モデルのパス
    (gltf) => {
      boidModelLod = gltf.scene;
      callback();
    },
    undefined,
    (error) => {
      console.error('An error occurred while loading the model:', error);
    }

  );
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
    layer >= (maxDepth - unitLayer.value + 1) &&
    (unit.children == null || unit.children.size() === 0 || layer === maxDepth)
  ) {
    let sphere;
    if (unitSpheres.length > 0) {
      sphere = unitSpheres.pop(); // 再利用
      sphere.visible = true;
    } else {
      const geometry = new THREE.SphereGeometry(unit.radius, 16, 16);
      const material = new THREE.MeshBasicMaterial({
        color: new THREE.Color().setHSL(0.1, 1, 0.7 - 0.4 * (layer / maxDepth)),
        wireframe: true,
        opacity: 0.3,
        transparent: true,
      });
      sphere = new THREE.Mesh(geometry, material);
      scene.add(sphere);
    }
    sphere.position.set(unit.center.x, unit.center.y, unit.center.z);
    unitSpheres.push(sphere);
  }

  // 線: チェックボックスで全て表示、色は深さでグラデ
  if (showUnitLines.value && unit.children && typeof unit.children.size === 'function' && unit.children.size() > 0) {
    for (let i = 0; i < unit.children.size(); i++) {
      const child = unit.children.get(i);
      let line;
      if (unitLines.length > 0) {
        line = unitLines.pop(); // 再利用
        line.visible = true;
      } else {
        const lineGeometry = new THREE.BufferGeometry();
        const lineMaterial = new THREE.LineBasicMaterial({
          color: new THREE.Color().setHSL(0.35, 1, 0.7 - 0.4 * (layer / maxDepth)),
        });
        line = new THREE.Line(lineGeometry, lineMaterial);
        scene.add(line);
      }
      const points = [
        new THREE.Vector3(unit.center.x, unit.center.y, unit.center.z),
        new THREE.Vector3(child.center.x, child.center.y, child.center.z),
      ];
      line.geometry.setFromPoints(points);
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
let positions, velocities, orientations;

function animate() {
  if (stats) stats.begin();

  if (!paused.value) {
    update(1.0);
  }
  const count = boidCount();

  positions = new Float32Array(wasmModule.HEAPF32.buffer, posPtr(), count * 3);
  orientations = new Float32Array(wasmModule.HEAPF32.buffer, oriPtr(), count * 4);

  const dummy = new THREE.Object3D();
  const cameraPosition = camera.position;

  for (let i = 0; i < count; i++) {
    // 位置を設定
    dummy.position.set(
      positions[i * 3 + 0],
      positions[i * 3 + 1],
      positions[i * 3 + 2]
    );

    // クォータニオンを設定
    dummy.quaternion.set(
      orientations[i * 4 + 0], // x
      orientations[i * 4 + 1], // y
      orientations[i * 4 + 2], // z
      orientations[i * 4 + 3]  // w
    );

    dummy.updateMatrix();

    // 距離判定
    const distanceSq = cameraPosition.distanceToSquared(dummy.position);
    if (distanceSq < 25) { // 近距離: 高ポリゴン
      instancedMeshHigh.setMatrixAt(i, dummy.matrix);
      instancedMeshLow.setMatrixAt(i, new THREE.Matrix4().identity()); // 非表示
    } else { // 遠距離: 低ポリゴン
      instancedMeshLow.setMatrixAt(i, dummy.matrix);
      instancedMeshHigh.setMatrixAt(i, new THREE.Matrix4().identity()); // 非表示
    }
  }

  instancedMeshHigh.instanceMatrix.needsUpdate = true;
  instancedMeshLow.instanceMatrix.needsUpdate = true;

  controls.update();
  // スマホの場合は renderer を使用、それ以外は composer を使用
  if (isMobileDevice()) {
    renderer.render(scene, camera);
  } else {
    composer.render();
  }
  if (stats) stats.end();

  animationTimer = setTimeout(animate, FRAME_INTERVAL);
}

function drawTreeStructure(treeData) {
  const drawNode = (node, parentPosition = null) => {
    const position = new THREE.Vector3(
      node.center[0],
      node.center[1],
      node.center[2]
    );

    if (parentPosition) {
      const geometry = new THREE.BufferGeometry().setFromPoints([
        parentPosition,
        position,
      ]);
      const material = new THREE.LineBasicMaterial({ color: 0xffffff });
      const line = new THREE.Line(geometry, material);
      scene.add(line);
    }

    if (node.children) {
      node.children.forEach((child) => drawNode(child, position));
    }
  };

  treeData.forEach((rootNode) => drawNode(rootNode));
}
function startSimulation() {
  initBoids(settings.flockSize, 6, 0.25);
  build(16, 0);
  initInstancedBoids(settings.flockSize);
  animate();
}
// `setSpeciesParams` を呼び出す関数
function updateSpeciesParams() {
  if (wasmModule && wasmModule.setSpeciesParams) {
    const raw = toRaw(settings);
    setSpeciesParams({
      cohesion: Number(raw.cohesion ?? -1),
      separation: Number(raw.separation ?? -1),
      alignment: Number(raw.alignment ?? -1),
      maxSpeed: Number(raw.maxSpeed ?? -1),
      minSpeed: Number(raw.minSpeed ?? -1),
      maxTurnAngle: Number(raw.maxTurnAngle ?? -1),
      separationRange: Number(raw.separationRange ?? -1),
      alignmentRange: Number(raw.alignmentRange ?? -1),
      cohesionRange: Number(raw.cohesionRange ?? -1),
      maxNeighbors: Number(raw.maxNeighbors ?? -1),
      lambda: Number(raw.lambda ?? -1),
      horizontalTorque: Number(raw.horizontalTorque ?? -1),
      velocityEpsilon: Number(raw.velocityEpsilon ?? -1),
      torqueStrength: Number(raw.torqueStrength ?? -1),
    }, 0.1);
  }
}
onMounted(() => {
  initThreeJS();
  loadBoidModel(() => {
    console.log('Boid model loaded successfully.');
    startSimulation();
  });
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
      maxNeighbors: Number(raw.maxNeighbors),
      lambda: Number(raw.lambda),
    }, 0.1);
  }

  window.addEventListener('keydown', handleKeydown);

});

function isMobileDevice() {
  return /Mobi|Android/i.test(navigator.userAgent);
}
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
    settings.maxNeighbors,
    settings.lambda,
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
        horizontalTorque: Number(raw.horizontalTorque),
        torqueStrength: Number(raw.torqueStrength),
        maxNeighbors: Number(raw.maxNeighbors),
        lambda: Number(raw.lambda),
      }, 0.1);
    }
  }
);

// flockSizeの変更を監視
watch(
  () => settings.flockSize,
  (newSize) => {
    if (setFlockSize) {
      // flockSize変更時
      setFlockSize(newSize, 40, 0.25);
      // Three.js 側の初期化
      initInstancedBoids(newSize);
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