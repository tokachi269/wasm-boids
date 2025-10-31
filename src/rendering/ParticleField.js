import * as THREE from 'three';

// 粒子帯の初期広がり（XYZ）: モバイルは控えめにして描画負荷を抑える
const PARTICLE_BASE_SPREAD_DESKTOP = new THREE.Vector3(24, 12, 26);
const PARTICLE_BASE_SPREAD_MOBILE = new THREE.Vector3(16, 9, 18);
// 粒子が描画対象となる最大距離: 広がりから導出
const PARTICLE_BASE_MAX_DISTANCE_DESKTOP = PARTICLE_BASE_SPREAD_DESKTOP.z * 1.2;
const PARTICLE_BASE_MAX_DISTANCE_MOBILE = PARTICLE_BASE_SPREAD_MOBILE.z * 1.2;
// ワールド基底（流れ方向および左右上下の初期値）
const FLOW_TILT_DEGREES = 30; // 粒子の流れを水平から約30度下向きに傾ける
const DEFAULT_FLOW_DIR = new THREE.Vector3(
  Math.cos(THREE.MathUtils.degToRad(FLOW_TILT_DEGREES)),
  -Math.sin(THREE.MathUtils.degToRad(FLOW_TILT_DEGREES)),
  0
);
const DEFAULT_LAT1 = new THREE.Vector3(0, 0, 1);
const DEFAULT_LAT2 = new THREE.Vector3(0, 1, 0);

// 頻度の高い計算で毎回 new しないようにモジュールスコープで共有
const TEMP_DIR = new THREE.Vector3();
const TEMP_AXIS = new THREE.Vector3();
const TEMP_LAT1 = new THREE.Vector3();
const TEMP_LAT2 = new THREE.Vector3();

/**
 * 軽量パーティクルシステムを司るヘルパークラス。
 * App.vue から init/update/dispose を呼び出し、内部で Three.js の Points を管理します。
 */
export class ParticleField {
  constructor(isMobileDevice) {
    this.isMobileDevice = Boolean(isMobileDevice);
    this.scene = null;
    this.renderer = null;
    this.camera = null;
    this.controls = null;

    this.particlePoints = null;
    this.particleMaterial = null;
    this.elapsedTime = 0;
  }

  /**
   * Three.js のシーンとカメラ、コントロール類を受け取りパーティクルを初期化します。
   */
  init(scene, renderer, camera, controls) {
    if (!scene || !renderer?.capabilities?.isWebGL2) {
      console.warn('Skipping particle system: WebGL2 required.');
      return false;
    }

    this.dispose(scene);

    this.scene = scene;
    this.renderer = renderer;
    this.camera = camera;
    this.controls = controls;
    this.elapsedTime = 0;

    const count = this.isMobileDevice ? 800 : 2000; // 端末の性能差に合わせて頂点数を抑制
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(count * 3), 3));
    geometry.setDrawRange(0, count);

    const baseSpread = (this.isMobileDevice ? PARTICLE_BASE_SPREAD_MOBILE : PARTICLE_BASE_SPREAD_DESKTOP).clone();
    const baseMaxDistance = this.isMobileDevice ? PARTICLE_BASE_MAX_DISTANCE_MOBILE : PARTICLE_BASE_MAX_DISTANCE_DESKTOP;

    const material = new THREE.ShaderMaterial({
      glslVersion: THREE.GLSL3,
      transparent: true,
      depthWrite: false,
      depthTest: true,
      blending: THREE.AdditiveBlending,
      uniforms: {
        uTime: { value: 0 }, // 経過時間（CPUから制御）
        uOrigin: { value: new THREE.Vector3() }, // 粒子帯の基準座標（カメラ位置）
        uFlowDir: { value: DEFAULT_FLOW_DIR.clone() }, // 流れ方向の基準ベクトル
        uLat1: { value: DEFAULT_LAT1.clone() }, // 流れに直交するラテラル軸1
        uLat2: { value: DEFAULT_LAT2.clone() }, // 同ラテラル軸2
        uSpread: { value: baseSpread.clone() }, // 粒子の広がり（XYZ スケール）
        uMaxDistance: { value: baseMaxDistance }, // 描画対象とする最大距離
        uBaseSpeed: { value: 0.6 }, // 流れ方向への基本速度倍率
        uJitterAmp: { value: 0.22 }, // ランダム揺らぎの振幅
        uSizePx: { value: this.isMobileDevice ? 5.0 : 12.0 }, // スクリーン上の粒子サイズ（px）
        uFadeNear: { value: 1.5 }, // 手前でフェードアウトを開始する距離
        uFadeFar: { value: 14.0 }, // フェードアウトが完了する距離
        uColorNear: { value: new THREE.Color(0x72a1c4) }, // カメラ近傍の粒子色
        uColorFar: { value: new THREE.Color(0x0a5270) }, // 遠方の粒子色
      },
      vertexShader: PARTICLE_VERTEX_SHADER,
      fragmentShader: PARTICLE_FRAGMENT_SHADER,
    });

    const baseTargetDistance = controls
      ? camera.position.distanceTo(controls.target)
      : camera.position.length();
    material.userData = {
      baseSpread: baseSpread.clone(),
      baseMaxDistance,
      baseTargetDistance: Math.max(baseTargetDistance, 0.1), // OrbitControls の距離を初期参照距離として記録
    };

    this.particlePoints = new THREE.Points(geometry, material);
    this.particlePoints.frustumCulled = false;
    this.particlePoints.renderOrder = 2;
    scene.add(this.particlePoints);

    this.particleMaterial = material;
    this.setWorldBasis(material.uniforms.uFlowDir.value);
    this.update(camera, controls);
    return true;
  }

  /**
   * uTime を直接設定します。
   */
  setTime(timeSeconds) {
    this.elapsedTime = timeSeconds ?? this.elapsedTime;
    if (this.particleMaterial?.uniforms?.uTime) {
      this.particleMaterial.uniforms.uTime.value = this.elapsedTime;
    }
  }

  /**
   * 経過時間を加算します。
   */
  advanceTime(deltaSeconds) {
    if (!Number.isFinite(deltaSeconds) || deltaSeconds === 0) {
      return;
    }
    this.elapsedTime += deltaSeconds;
    if (this.particleMaterial?.uniforms?.uTime) {
      this.particleMaterial.uniforms.uTime.value = this.elapsedTime;
    }
  }

  /**
   * フロー方向から直交基底を生成し、uniform に反映します。
   */
  setWorldBasis(flowDir) {
    if (!this.particleMaterial || !flowDir) {
      return;
    }

    const dir = TEMP_DIR.copy(flowDir).normalize();
    // 極付近で cross がゼロに近づくのを避けるため適当な補助軸を選ぶ
    const axis = Math.abs(dir.y) < 0.99
      ? TEMP_AXIS.set(0, 1, 0)
      : TEMP_AXIS.set(1, 0, 0);

    const lat1 = TEMP_LAT1.copy(dir).cross(axis);
    if (lat1.lengthSq() < 1e-6) {
      lat1.set(0, 0, 1);
      lat1.cross(dir);
    }
    lat1.normalize();

    const lat2 = TEMP_LAT2.copy(dir).cross(lat1).normalize();

    const uniforms = this.particleMaterial.uniforms;
    uniforms.uFlowDir.value.copy(dir);
    uniforms.uLat1.value.copy(lat1);
    uniforms.uLat2.value.copy(lat2);
  }

  /**
   * カメラに合わせて uniform を更新します。
   */
  update(camera, controls) {
    const targetCamera = camera || this.camera;
    if (!this.particleMaterial || !targetCamera) {
      return;
    }

    const uniforms = this.particleMaterial.uniforms;
    uniforms.uOrigin.value.copy(targetCamera.position);

    const { baseSpread, baseMaxDistance, baseTargetDistance } = this.particleMaterial.userData || {};
    if (!baseSpread || !baseMaxDistance) {
      return;
    }

    // ズーム時に密度が変化しないよう、常に初期値を適用する
    uniforms.uSpread.value.copy(baseSpread);
    uniforms.uMaxDistance.value = baseMaxDistance;
  }

  /**
   * シーンからパーティクルオブジェクトを取り除き、GPUリソースを解放します。
   */
  dispose(scene) {
    const targetScene = scene || this.scene;
    if (this.particlePoints && targetScene) {
      targetScene.remove(this.particlePoints);
    }
    this.particlePoints?.geometry?.dispose();
    this.particlePoints?.material?.dispose();

    this.particlePoints = null;
    this.particleMaterial = null;
    this.scene = null;
    this.renderer = null;
    this.camera = null;
    this.controls = null;
  }

  /** 現在のマテリアル参照を返します。 */
  getMaterial() {
    return this.particleMaterial;
  }
}

const PARTICLE_VERTEX_SHADER = /* glsl */`
precision highp float;
precision highp int;

uniform float uTime;
uniform vec3 uOrigin;
uniform vec3 uFlowDir;
uniform vec3 uLat1;
uniform vec3 uLat2;
uniform vec3 uSpread;
uniform float uMaxDistance;
uniform float uBaseSpeed;
uniform float uJitterAmp;
uniform float uSizePx;
uniform float uFadeNear;
uniform float uFadeFar;

out float vFade;
out float vColorMix;

uint hashUint(uint n) {
  n ^= (n << 13);
  n ^= (n >> 17);
  n ^= (n << 5);
  return n * 0x27d4eb2du;
}

float hashFloat(uint n) {
  return float(hashUint(n)) / 4294967296.0;
}

vec3 hashVec3(uint n) {
  return vec3(
    hashFloat(n),
    hashFloat(n * 1664525u + 1013904223u),
    hashFloat(n * 22695477u + 1u)
  );
}

void main() {
  uint id = uint(gl_VertexID);
  vec3 randSeed = hashVec3(id) - 0.5;

  vec3 flowDir = normalize(uFlowDir);
  vec3 lateral = uLat1 * (randSeed.x * uSpread.x)
    + uLat2 * (randSeed.y * uSpread.y);

  float speedSeed = hashFloat(id * 747796405u);
  float speed = uBaseSpeed * (0.7 + 0.6 * speedSeed);
  float flowCycle = max(uSpread.z, 1e-3);
  float flowParam = randSeed.z + (uTime * speed) / flowCycle;
  float wrappedZ = fract(flowParam) - 0.5;
  vec3 pos = uOrigin + lateral + flowDir * (wrappedZ * flowCycle);

  float phase = hashFloat(id * 1664525u) * 6.28318530718;
  float driftSeed = hashFloat(id * 22695477u);
  float triangle = 1.0 - abs(fract((phase + uTime * (0.8 + 0.4 * driftSeed)) / 3.14159265) * 2.0 - 1.0);

  vec3 jitter1 = normalize(uLat1);
  vec3 jitter2 = normalize(uLat2);
  pos += jitter1 * ((triangle - 0.5) * uJitterAmp);
  pos += jitter2 * ((hashFloat(id * 1103515245u) - 0.5) * uJitterAmp * 0.6);

  vec4 mvPosition = modelViewMatrix * vec4(pos, 1.0);
  float dist = -mvPosition.z;
  float forwardMask = step(0.0, dist);

  vec4 clipPosition = projectionMatrix * mvPosition;
  float invW = 1.0 / max(abs(clipPosition.w), 1e-3);
  vec2 ndc = clipPosition.xy * invW;

  float screenMask = 1.0 - smoothstep(0.96, 1.16, max(abs(ndc.x), abs(ndc.y)));
  float rangeMask = 1.0 - smoothstep(uMaxDistance * 0.7, uMaxDistance, dist);

  float fadeT = clamp((dist - uFadeNear) / max(uFadeFar - uFadeNear, 1e-3), 0.0, 1.0);
  float fade = (1.0 - fadeT) * screenMask * rangeMask * forwardMask;
  vFade = fade;
  vColorMix = fade;

  float sizeBase = uSizePx * projectionMatrix[1][1] / max(dist, 1e-3);
  float attenuatedSize = sizeBase * fade;
  gl_PointSize = max(attenuatedSize, 0.75) * step(0.001, fade);
  gl_Position = clipPosition;
}
`;

const PARTICLE_FRAGMENT_SHADER = /* glsl */`
precision highp float;

in float vFade;
in float vColorMix;

uniform vec3 uColorNear;
uniform vec3 uColorFar;

out vec4 fragColor;

void main() {
  vec2 coord = gl_PointCoord - vec2(0.5);
  float falloff = exp(-8.0 * dot(coord, coord));
  float alpha = vFade * falloff * 0.6;
  if (alpha < 0.004) {
    discard;
  }
  vec3 color = mix(uColorFar, uColorNear, vColorMix);
  fragColor = vec4(color, alpha);
}
`;
