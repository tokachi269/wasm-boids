import * as THREE from 'three';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { SSAOPass } from 'three/examples/jsm/postprocessing/SSAOPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';
import { ShaderPass } from 'three/examples/jsm/postprocessing/ShaderPass.js';

/**
 * フォグやポストプロセスのセットアップを担当。
 * Three.js の EffectComposer をまとめて初期化し、フォグ用シェーダーへ
 * 深度テクスチャを受け渡す責務を持ちます。
 */
export class FogPipeline {
  constructor(fogConfig) {
    this.config = fogConfig;
    this.composer = null;
    this.heightFogPass = null;
    this.heightFogRenderTarget = null;
    this.renderer = null;
    this.scene = null;
    this.camera = null;

    // ポストプロセスはフル解像度だと GPU のフラグメント負荷が支配的になりやすい。
    // 見た目（Fog/SSAO/Bloom）を維持しつつ負荷を落とすため、内部レンダーターゲットを縮小して回す。
    // NOTE: UI は増やさない方針のため、ここは固定スケール。
    this.internalScale = 0.75;
  }

  /** HeightFog 用の ShaderPass を返します。 */
  get heightFog() {
    return this.heightFogPass;
  }

  /**
   * レンダラ・シーン・カメラからポストプロセスチェーンを初期化。
   * レンダリング結果 + 深度テクスチャを HeightFogShader に渡します。
   */
  init(renderer, scene, camera, width, height) {
    if (!renderer || !scene || !camera) {
      console.warn('FogPipeline.init: renderer / scene / camera が未設定です');
      return;
    }

    this.dispose();

    this.renderer = renderer;
    this.scene = scene;
    this.camera = camera;

    const scale = Math.max(0.25, Math.min(Number(this.internalScale) || 1.0, 1.0));
    const scaledWidth = Math.max(1, Math.floor(width * scale));
    const scaledHeight = Math.max(1, Math.floor(height * scale));

    // 深度テクスチャ付きのターゲットを確保し、fog パスへ共有する
    const rtOptions = { depthBuffer: true, stencilBuffer: false };
    this.heightFogRenderTarget = new THREE.WebGLRenderTarget(scaledWidth, scaledHeight, rtOptions);
    // Fog は深度の「線形復元」に必要な精度があれば良い。
    // Float 深度は帯域/互換性の面で重くなりやすいので、整数深度を優先する。
    this.heightFogRenderTarget.depthTexture = new THREE.DepthTexture(scaledWidth, scaledHeight, THREE.UnsignedIntType);
    this.heightFogRenderTarget.depthTexture.format = THREE.DepthFormat;
    this.heightFogRenderTarget.depthTexture.type = THREE.UnsignedIntType;
    this.heightFogRenderTarget.depthTexture.needsUpdate = true;

    this.composer = new EffectComposer(renderer, this.heightFogRenderTarget);

    // ベースとなるレンダリング結果を取得
    const renderPass = new RenderPass(scene, camera);
    this.composer.addPass(renderPass);

    // 環境の陰影を補強する SSAO
    const ssaoPass = new SSAOPass(scene, camera, scaledWidth, scaledHeight);
    ssaoPass.kernelRadius = 5;     // サンプル半径（大きいほど広範囲な AO）
    ssaoPass.minDistance = 0.01;   // オクルージョン開始距離
    ssaoPass.maxDistance = 0.3;    // 影響を与える最大距離
    this.composer.addPass(ssaoPass);

    // ハイライトを強調するブルーム
    const bloomStrength = 1.0;   // ブルームの強さ
    const bloomRadius = 0.25;     // 発光の広がり
    const bloomThreshold = 0.85; // ブルームを適用する輝度
    const bloomPass = new UnrealBloomPass(new THREE.Vector2(scaledWidth, scaledHeight), bloomStrength, bloomRadius, bloomThreshold);
    this.composer.addPass(bloomPass);

    const shader = createHeightFogShader(this.config);
    this.heightFogPass = new ShaderPass(shader);
    // 深度テクスチャを毎フレーム差し替え、霧の密度を距離で制御
    const originalRender = this.heightFogPass.render.bind(this.heightFogPass);
    this.heightFogPass.render = (passRenderer, writeBuffer, readBuffer, deltaTime, maskActive) => {
      if (this.heightFogPass.uniforms.tDepth) {
        const depthTexture = readBuffer?.depthTexture ?? this.heightFogRenderTarget?.depthTexture ?? null;
        if (depthTexture) {
          this.heightFogPass.uniforms.tDepth.value = depthTexture;
        }
      }
      originalRender(passRenderer, writeBuffer, readBuffer, deltaTime, maskActive);
    };
    this.heightFogPass.needsSwap = false;
    this.heightFogPass.uniforms.tDepth.value = this.heightFogRenderTarget.depthTexture;

    applyFogConfigToUniforms(this.heightFogPass.uniforms, this.config);

    const material = this.heightFogPass.material;
    if (material) {
      material.depthTest = false;
      material.depthWrite = false;
      material.transparent = false;
      material.blending = THREE.NoBlending;
      material.toneMapped = true;
      material.needsUpdate = true;
    }

    this.heightFogPass.renderToScreen = true;
    this.composer.addPass(this.heightFogPass);
  }

  /** ウィンドウリサイズ時に呼び出してバッファサイズを調整。 */
  resize(width, height) {
    if (!this.composer) {
      return;
    }
    const scale = Math.max(0.25, Math.min(Number(this.internalScale) || 1.0, 1.0));
    const scaledWidth = Math.max(1, Math.floor(width * scale));
    const scaledHeight = Math.max(1, Math.floor(height * scale));
    this.composer.setSize(scaledWidth, scaledHeight);
    this.heightFogRenderTarget?.setSize(scaledWidth, scaledHeight);
  }

  /** composer を使ってレンダリングするヘルパー。 */
  render(delta) {
    if (!this.composer) {
      return;
    }
    this.composer.render(delta);
  }

  /** composer が準備できているか確認。 */
  isReady() {
    return !!this.composer;
  }

  /** カメラ行列やパラメータをフォグシェーダーへ転送。 */
  updateCameraUniforms(camera) {
    const pass = this.heightFogPass;
    const targetCamera = camera || this.camera;
    if (!pass || !targetCamera) {
      return;
    }
    const { uniforms } = pass;
    if (uniforms.cameraNear) {
      uniforms.cameraNear.value = targetCamera.near;
    }
    if (uniforms.cameraFar) {
      uniforms.cameraFar.value = targetCamera.far;
    }
    if (uniforms.projectionMatrixInverse?.value) {
      uniforms.projectionMatrixInverse.value.copy(targetCamera.projectionMatrixInverse);
    }
    if (uniforms.cameraMatrixWorld?.value) {
      uniforms.cameraMatrixWorld.value.copy(targetCamera.matrixWorld);
    }
  }

  /** フォグ設定を更新し、シェーダーのユニフォームへ反映。 */
  updateConfig(nextConfig = {}) {
    this.config = { ...this.config, ...nextConfig };
    if (!this.heightFogPass) {
      return;
    }
    applyFogConfigToUniforms(this.heightFogPass.uniforms, this.config);
  }

  /** リソース解放。 */
  dispose() {
    // HeightFog の距離カーブLUTは小さいが、ホットリロード等で積むとリークし得る。
    // composer/material の破棄とは独立に明示的に解放する。
    const lut = this.heightFogPass?.uniforms?.tDistanceCurveLut?.value ?? null;
    if (lut && typeof lut.dispose === 'function') {
      lut.dispose();
    }
    this.heightFogPass = null;
    this.composer?.dispose?.();
    this.composer = null;
    this.heightFogRenderTarget?.dispose?.();
    this.heightFogRenderTarget = null;
    this.renderer = null;
    this.scene = null;
    this.camera = null;
  }
}

/**
 * HeightFogShader 定義をそのまま移植するためのプレースホルダ。
 */
export function createHeightFogShader(config = {}) {
  const defaults = buildFogDefaults();
  const finalConfig = { ...defaults, ...config };

  // 距離フォグのカーブはピクセルごとに逆変換すると高コスト。
  // LUTにするとテクスチャ1回で済み、GPU負荷を安定して下げられる。
  const distanceCurveLut = createOrUpdateDistanceCurveLutTexture(
    null,
    cloneVector2(finalConfig.distanceControlPoint1),
    cloneVector2(finalConfig.distanceControlPoint2)
  );

  return {
    uniforms: {
      tDiffuse: { value: null },
      tDepth: { value: null },
      tDistanceCurveLut: { value: distanceCurveLut },
      cameraNear: { value: 0.1 },
      cameraFar: { value: 1000 },
      projectionMatrixInverse: { value: new THREE.Matrix4() },
      cameraMatrixWorld: { value: new THREE.Matrix4() },
      fogColor: { value: cloneColor(finalConfig.color) },                     // 霧の色
      distanceStart: { value: finalConfig.distanceStart },                   // 距離フォグ開始位置
      distanceEnd: { value: finalConfig.distanceEnd },                       // 距離フォグ最大濃度距離
      distanceExponent: { value: finalConfig.distanceExponent },             // 距離カーブの鋭さ
      // NOTE: 制御点はJS側LUT生成にのみ使う。シェーダには渡さない（ピクセルコスト削減）。
      surfaceLevel: { value: finalConfig.surfaceLevel },                     // 水面高さ（y）
      heightFalloff: { value: finalConfig.heightFalloff },                   // 深度減衰率
      heightExponent: { value: finalConfig.heightExponent },                 // 深度カーブの鋭さ
      maxOpacity: { value: finalConfig.maxOpacity },                         // 霧の最大不透明度
    },
    vertexShader: /* glsl */`
    varying vec2 vUv;
    void main() {
      vUv = uv;
      gl_Position = vec4(position.xy, 0.0, 1.0);
    }
  `,
    fragmentShader: /* glsl */`
    precision highp float;
    varying vec2 vUv;

    uniform sampler2D tDiffuse;
    uniform sampler2D tDepth;
    uniform sampler2D tDistanceCurveLut;
    uniform float cameraNear;
    uniform float cameraFar;
    uniform vec3 fogColor;
    uniform float distanceStart;
    uniform float distanceEnd;
    uniform float distanceExponent;
    uniform float surfaceLevel;
    uniform float heightFalloff;
    uniform float heightExponent;
    uniform float maxOpacity;
    uniform mat4 projectionMatrixInverse;
    uniform mat4 cameraMatrixWorld;

    float sampleDistanceCurve(float x) {
      // 256x1 LUT。LinearFilterで補間されるためバンディングを抑えられる。
      float u = clamp(x, 0.0, 1.0);
      return texture2D(tDistanceCurveLut, vec2(u, 0.5)).r;
    }

    void main() {
      vec4 baseColor = texture2D(tDiffuse, vUv);
      float depth = texture2D(tDepth, vUv).x;

      if (depth >= 1.0) {
        gl_FragColor = baseColor;
        return;
      }

      vec2 ndc = vUv * 2.0 - 1.0;
      float ndcZ = depth * 2.0 - 1.0;
      vec4 clipPos = vec4(ndc, ndcZ, 1.0);
      vec4 viewPos = projectionMatrixInverse * clipPos;
      viewPos /= max(viewPos.w, 1e-5);
      // height fog は world-space の Y（=水面からの深さ）だけ使うため、
      // vec4 全体の変換は避け、Y 成分だけを計算して ALU を減らす。
      float worldY =
        cameraMatrixWorld[0][1] * viewPos.x +
        cameraMatrixWorld[1][1] * viewPos.y +
        cameraMatrixWorld[2][1] * viewPos.z +
        cameraMatrixWorld[3][1] * viewPos.w;

      // Fog の距離は厳密なユークリッド距離でなくても見た目が崩れにくい。
      // sqrt(length) はフラグメントコストになりやすいので、視線方向距離で近似する。
      float viewDistance = abs(viewPos.z);
      float distanceFogNorm = clamp(
        (viewDistance - distanceStart) / max(distanceEnd - distanceStart, 1e-5),
        0.0,
        1.0
      );
      float distanceFog = sampleDistanceCurve(distanceFogNorm);
      distanceFog = pow(distanceFog, distanceExponent);

      float depthBelowSurface = max(surfaceLevel - worldY, 0.0);
      float heightFactor = 1.0 - exp(-depthBelowSurface * heightFalloff);
      heightFactor = clamp(pow(heightFactor, heightExponent), 0.0, 1.0);

      float fogFactor = clamp(distanceFog * heightFactor, 0.0, 1.0);
      fogFactor = mix(0.0, maxOpacity, fogFactor);

      vec3 fogged = mix(baseColor.rgb, fogColor, fogFactor);
      gl_FragColor = vec4(fogged, baseColor.a);
      #include <tonemapping_fragment>
      #include <colorspace_fragment>
    }
  `,
  };
}

export const HeightFogShader = createHeightFogShader();

function buildFogDefaults() {
  return {
    color: new THREE.Color('#153a6c'),                // 霧の基本色
    distanceStart: 4,                                 // カメラからこの距離でフォグ開始
    distanceEnd: 60,                                  // この距離でフォグが最大
    distanceExponent: 0.4,                            // 距離カーブの緩急
    distanceControlPoint1: new THREE.Vector2(0.4, 0.75), // ベジェ制御点（開始側）
    distanceControlPoint2: new THREE.Vector2(0.75, 0.95), // ベジェ制御点（終端側）
    surfaceLevel: 100,                                // 海面の World Y 値
    heightFalloff: 0.01,                              // 深度方向の減衰係数
    heightExponent: 1,                                // 深度カーブの鋭さ
    maxOpacity: 1,                                    // 霧の上限不透明度
  };
}

function cloneColor(value) {
  if (value instanceof THREE.Color) {
    return value.clone();
  }
  if (typeof value === 'string' || typeof value === 'number') {
    return new THREE.Color(value);
  }
  return new THREE.Color('#000000');
}

function cloneVector2(value) {
  if (value instanceof THREE.Vector2) {
    return value.clone();
  }
  if (value && typeof value.x === 'number' && typeof value.y === 'number') {
    return new THREE.Vector2(value.x, value.y);
  }
  return new THREE.Vector2();
}

function applyFogConfigToUniforms(uniforms, config = {}) {
  const defaults = buildFogDefaults();
  const finalConfig = { ...defaults, ...config };

  if (uniforms.fogColor?.value) {
    uniforms.fogColor.value.copy(cloneColor(finalConfig.color));
  }
  if (uniforms.distanceStart) {
    uniforms.distanceStart.value = finalConfig.distanceStart;
  }
  if (uniforms.distanceEnd) {
    uniforms.distanceEnd.value = finalConfig.distanceEnd;
  }
  if (uniforms.distanceExponent) {
    uniforms.distanceExponent.value = finalConfig.distanceExponent;
  }
  if (uniforms.tDistanceCurveLut) {
    uniforms.tDistanceCurveLut.value = createOrUpdateDistanceCurveLutTexture(
      uniforms.tDistanceCurveLut.value,
      cloneVector2(finalConfig.distanceControlPoint1),
      cloneVector2(finalConfig.distanceControlPoint2)
    );
  }
  if (uniforms.surfaceLevel) {
    uniforms.surfaceLevel.value = finalConfig.surfaceLevel;
  }
  if (uniforms.heightFalloff) {
    uniforms.heightFalloff.value = finalConfig.heightFalloff;
  }
  if (uniforms.heightExponent) {
    uniforms.heightExponent.value = finalConfig.heightExponent;
  }
  if (uniforms.maxOpacity) {
    uniforms.maxOpacity.value = finalConfig.maxOpacity;
  }
}

// ---- Distance curve LUT ----
// ベジェ曲線の「x→y」対応は、シェーダで逆変換（Newton）すると高コスト。
// JS側で256サンプルのLUTを生成し、シェーダは参照だけにする。
function createOrUpdateDistanceCurveLutTexture(existingTexture, controlPoint1, controlPoint2) {
  const size = 256;
  const needsNew = !existingTexture ||
    !(existingTexture instanceof THREE.DataTexture) ||
    existingTexture.image?.width !== size ||
    existingTexture.image?.height !== 1;

  const c1 = controlPoint1 instanceof THREE.Vector2 ? controlPoint1 : new THREE.Vector2();
  const c2 = controlPoint2 instanceof THREE.Vector2 ? controlPoint2 : new THREE.Vector2();

  // 更新不要判定（同一値ならGPU転送を省く）
  const last = existingTexture?.userData?.distanceCurveLut ?? null;
  const same = last &&
    last.c1x === c1.x && last.c1y === c1.y &&
    last.c2x === c2.x && last.c2y === c2.y;
  if (!needsNew && same) {
    return existingTexture;
  }

  const data = needsNew ? new Uint8Array(size * 4) : existingTexture.image.data;

  // cubic-bezier(0,0) -> (c1) -> (c2) -> (1,1)
  const bezier1D = (t, p0, p1, p2, p3) => {
    const u = 1 - t;
    const uu = u * u;
    const tt = t * t;
    return u * uu * p0 + 3 * uu * t * p1 + 3 * u * tt * p2 + tt * t * p3;
  };
  const bezierDerivative1D = (t, p0, p1, p2, p3) => {
    const u = 1 - t;
    return 3 * u * u * (p1 - p0) + 6 * u * t * (p2 - p1) + 3 * t * t * (p3 - p2);
  };

  // xに対応するtをNewtonで求め、yを得る。
  for (let i = 0; i < size; i += 1) {
    const x = i / (size - 1);
    let t = x;
    for (let iter = 0; iter < 5; iter += 1) {
      const current = bezier1D(t, 0, c1.x, c2.x, 1) - x;
      const slope = bezierDerivative1D(t, 0, c1.x, c2.x, 1);
      if (Math.abs(slope) < 1e-5) {
        break;
      }
      t = Math.min(1, Math.max(0, t - current / slope));
    }
    const y = bezier1D(t, 0, c1.y, c2.y, 1);
    const v = Math.min(255, Math.max(0, Math.round(y * 255)));
    const base = i * 4;
    data[base + 0] = v;
    data[base + 1] = v;
    data[base + 2] = v;
    data[base + 3] = 255;
  }

  const texture = needsNew
    ? new THREE.DataTexture(data, size, 1, THREE.RGBAFormat, THREE.UnsignedByteType)
    : existingTexture;

  texture.wrapS = THREE.ClampToEdgeWrapping;
  texture.wrapT = THREE.ClampToEdgeWrapping;
  texture.minFilter = THREE.LinearFilter;
  texture.magFilter = THREE.LinearFilter;
  texture.generateMipmaps = false;
  texture.needsUpdate = true;
  texture.userData.distanceCurveLut = { c1x: c1.x, c1y: c1.y, c2x: c2.x, c2y: c2.y };

  return texture;
}
