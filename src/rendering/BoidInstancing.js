import * as THREE from 'three';

/**
 * Boid のインスタンシング描画管理クラス。
 * ハイ/ロー LOD インスタンスを三重バッファで更新し、尾びれアニメ用の uniform を注入します。
 */
export class BoidInstancing {
  constructor({
    tailAnimation,
    tripleBufferSize = 3,        // インスタンス更新に用いるバッファ数（遅延許容量）
    hiddenPosition = 1e6,        // 非表示時に退避させる座標
    identityQuaternion = [0, 0, 0, 1], // 非表示時に適用する無回転クォータニオン
    lodNearDistanceSq = 4,       // ハイポリ表示に切り替える距離²（約2m）
    lodMidDistanceSq = 25,       // LOD メッシュ＋アニメを行う距離²（約5m）
  } = {}) {
    this.tailAnimation = tailAnimation;
    this.tripleBufferSize = tripleBufferSize;
    this.hiddenPosition = hiddenPosition;
    this.identityQuaternion = identityQuaternion;
    this.lodNearDistanceSq = lodNearDistanceSq;
    this.lodMidDistanceSq = lodMidDistanceSq;
    this.streamUsage = THREE.StreamDrawUsage ?? THREE.DynamicDrawUsage;

    this.scene = null;
    this.count = 0;

    this.instancedMeshHigh = null;
    this.instancedMeshLow = null;
    this.instancingMaterials = new Set();

    this.bufferSetHigh = null;
    this.bufferSetLow = null;
    this.tailPhaseSeeds = null;
    this.bufferCursor = 0;

    this.previousVelocities = null;
    this.previousLodFlags = null;

    this.predatorModel = null;
    this.predatorMeshes = [];
    this.predatorMeshCountCache = -1;
  }

  init(scene, {
    count,
    boidModel,
    boidModelLod,
    highMaterial,
    lowMaterial,
    predatorModel = null,
  }) {
    if (!scene || !boidModel || !boidModelLod || !highMaterial || !lowMaterial) {
      console.error('BoidInstancing.init: missing required arguments.');
      return false;
    }
    if (!boidModel.children?.length || !boidModel.children[0].geometry) {
      console.error('BoidInstancing.init: invalid high model.');
      return false;
    }
    if (!boidModelLod.children?.length || !boidModelLod.children[0].geometry) {
      console.error('BoidInstancing.init: invalid LOD model.');
      return false;
    }

    this.dispose(scene);

    this.scene = scene;
    this.count = count;
    if (predatorModel) {
      this.predatorModel = predatorModel;
    }

    const highMaterialClone = highMaterial.clone();
    const lowMaterialClone = lowMaterial.clone();
    this.patchMaterial(highMaterialClone);
    this.patchMaterial(lowMaterialClone);

    // モデルごとの頂点属性を複製してから必要な属性を追加する
    const highGeometry = boidModel.children[0].geometry.clone();
    const lowGeometry = boidModelLod.children[0].geometry.clone();
    this.ensureBodyCoordAttribute(highGeometry);
    this.ensureBodyCoordAttribute(lowGeometry);

    this.instancedMeshHigh = new THREE.InstancedMesh(highGeometry, highMaterialClone, count);
    this.configureInstancedMesh(this.instancedMeshHigh, highMaterialClone);

    this.instancedMeshLow = new THREE.InstancedMesh(lowGeometry, lowMaterialClone, count);
    this.configureInstancedMesh(this.instancedMeshLow, lowMaterialClone);

    scene.add(this.instancedMeshHigh);
    scene.add(this.instancedMeshLow);

    this.bufferSetHigh = this.createBufferSet(count);
    this.bufferSetLow = this.createBufferSet(count);
    this.resetBufferSetToHidden(this.bufferSetHigh);
    this.resetBufferSetToHidden(this.bufferSetLow);

    this.tailPhaseSeeds = this.createTailPhaseArray(count);
    this.applyTailPhaseSeeds(this.bufferSetHigh, this.tailPhaseSeeds);
    this.applyTailPhaseSeeds(this.bufferSetLow, this.tailPhaseSeeds);

    this.bufferCursor = 0;
    this.applyBufferSet(this.instancedMeshHigh, this.bufferSetHigh, this.bufferCursor);
    this.applyBufferSet(this.instancedMeshLow, this.bufferSetLow, this.bufferCursor);

    this.instancedMeshHigh.instanceMatrix.setUsage(THREE.StaticDrawUsage);
    this.instancedMeshLow.instanceMatrix.setUsage(THREE.StaticDrawUsage);
    this.instancedMeshHigh.count = count;
    this.instancedMeshLow.count = count;

    this.previousVelocities = null;
    this.previousLodFlags = null;

    if (this.scene && this.predatorModel) {
      this.ensurePredatorMeshes(0);
    }

    return true;
  }

  /**
   * シミュレーション出力を三重バッファへ書き込み、LOD ごとにインスタンスを切り替える。
   */
  update({
    count,
    positions,
    orientations,
    velocities,
    cameraPosition,
    predatorCount = 0,
  }) {
    if (!this.instancedMeshHigh || !this.instancedMeshLow || !this.bufferSetHigh || !this.bufferSetLow) {
      return { visibleCount: 0 };
    }
    if (!positions || !orientations) {
      return { visibleCount: 0 };
    }

    this.ensureTailRuntimeBuffers(count);
    const lodFlags = this.ensureLodFlagBuffer(count);

    const currentIndex = this.bufferCursor;
    const nextIndex = (currentIndex + 1) % this.tripleBufferSize;

    const highPosAttr = this.bufferSetHigh.pos[currentIndex];
    const highQuatAttr = this.bufferSetHigh.quat[currentIndex];
    const highTailParamsAttr = this.bufferSetHigh.tailParams[currentIndex];
    const lowPosAttr = this.bufferSetLow.pos[currentIndex];
    const lowQuatAttr = this.bufferSetLow.quat[currentIndex];
    const lowTailParamsAttr = this.bufferSetLow.tailParams[currentIndex];

    const highPosArray = highPosAttr.array;
    const highQuatArray = highQuatAttr.array;
    const highTailParamsArray = highTailParamsAttr.array;
    const lowPosArray = lowPosAttr.array;
    const lowQuatArray = lowQuatAttr.array;
    const lowTailParamsArray = lowTailParamsAttr.array;

    let highTransformTouched = false;
    let lowTransformTouched = false;
    let highTailTouched = false;
    let lowTailTouched = false;

    const camX = cameraPosition?.x ?? 0;
    const camY = cameraPosition?.y ?? 0;
    const camZ = cameraPosition?.z ?? 0;

    const predatorStartIndex = predatorCount > 0 ? Math.max(0, count - predatorCount) : count;
    if (this.scene && this.predatorModel) {
      if (this.ensurePredatorMeshes(predatorCount)) {
        this.predatorMeshCountCache = predatorCount;
      }
      for (const mesh of this.predatorMeshes) {
        mesh.visible = false;
      }
    }

    const hiddenPos = this.hiddenPosition;
    const idQuat = this.identityQuaternion;
    const prevVel = this.previousVelocities;

    for (let i = 0; i < count; i++) {
      const basePos = i * 3;
      const baseQuat = i * 4;
      const tailIndex = i * 3;

      const px = positions[basePos];
      const py = positions[basePos + 1];
      const pz = positions[basePos + 2];

      const dx = px - camX;
      const dy = py - camY;
      const dz = pz - camZ;
      // 高負荷を避けるため距離で LOD を切り替える
      const distSq = dx * dx + dy * dy + dz * dz;
      const isNear = distSq < this.lodNearDistanceSq;
      const isMid = !isNear && distSq < this.lodMidDistanceSq;
      const animateTail = isNear || isMid;

      const qx = orientations[baseQuat];
      const qy = orientations[baseQuat + 1];
      const qz = orientations[baseQuat + 2];
      const qw = orientations[baseQuat + 3];

      const isPredator = i >= predatorStartIndex && predatorCount > 0;
      if (isPredator) {
        const meshIndex = i - predatorStartIndex;
        const predatorMesh = this.predatorMeshes[meshIndex];
        if (predatorMesh) {
          predatorMesh.visible = true;
          predatorMesh.position.set(px, py, pz);
          predatorMesh.quaternion.set(qx, qy, qz, qw);
        }

        // 捕食者は別メッシュで描画するためインスタンス側は非表示にする
        if (
          highPosArray[basePos] !== hiddenPos ||
          highPosArray[basePos + 1] !== hiddenPos ||
          highPosArray[basePos + 2] !== hiddenPos ||
          highQuatArray[baseQuat] !== idQuat[0] ||
          highQuatArray[baseQuat + 1] !== idQuat[1] ||
          highQuatArray[baseQuat + 2] !== idQuat[2] ||
          highQuatArray[baseQuat + 3] !== idQuat[3]
        ) {
          highPosArray[basePos] = hiddenPos;
          highPosArray[basePos + 1] = hiddenPos;
          highPosArray[basePos + 2] = hiddenPos;
          highQuatArray[baseQuat] = idQuat[0];
          highQuatArray[baseQuat + 1] = idQuat[1];
          highQuatArray[baseQuat + 2] = idQuat[2];
          highQuatArray[baseQuat + 3] = idQuat[3];
          highTransformTouched = true;
        }

        if (
          lowPosArray[basePos] !== hiddenPos ||
          lowPosArray[basePos + 1] !== hiddenPos ||
          lowPosArray[basePos + 2] !== hiddenPos ||
          lowQuatArray[baseQuat] !== idQuat[0] ||
          lowQuatArray[baseQuat + 1] !== idQuat[1] ||
          lowQuatArray[baseQuat + 2] !== idQuat[2] ||
          lowQuatArray[baseQuat + 3] !== idQuat[3]
        ) {
          lowPosArray[basePos] = hiddenPos;
          lowPosArray[basePos + 1] = hiddenPos;
          lowPosArray[basePos + 2] = hiddenPos;
          lowQuatArray[baseQuat] = idQuat[0];
          lowQuatArray[baseQuat + 1] = idQuat[1];
          lowQuatArray[baseQuat + 2] = idQuat[2];
          lowQuatArray[baseQuat + 3] = idQuat[3];
          lowTransformTouched = true;
        }

        if (highTailParamsArray[tailIndex] !== 0 || highTailParamsArray[tailIndex + 1] !== 0 || highTailParamsArray[tailIndex + 2] !== 0) {
          highTailParamsArray[tailIndex] = 0;
          highTailParamsArray[tailIndex + 1] = 0;
          highTailParamsArray[tailIndex + 2] = 0;
          highTailTouched = true;
        }
        if (lowTailParamsArray[tailIndex] !== 0 || lowTailParamsArray[tailIndex + 1] !== 0 || lowTailParamsArray[tailIndex + 2] !== 0) {
          lowTailParamsArray[tailIndex] = 0;
          lowTailParamsArray[tailIndex + 1] = 0;
          lowTailParamsArray[tailIndex + 2] = 0;
          lowTailTouched = true;
        }

        if (lodFlags) {
          lodFlags[i] = 0;
        }
        if (prevVel) {
          prevVel[basePos] = 0;
          prevVel[basePos + 1] = 0;
          prevVel[basePos + 2] = 0;
        }
        continue;
      }

      const vx = velocities ? velocities[basePos] : 0;
      const vy = velocities ? velocities[basePos + 1] : 0;
      const vz = velocities ? velocities[basePos + 2] : 0;
      const speed = Math.hypot(vx, vy, vz);
      const prevVx = prevVel ? prevVel[basePos] : 0;
      const prevVy = prevVel ? prevVel[basePos + 1] : 0;
      const prevVz = prevVel ? prevVel[basePos + 2] : 0;
      const prevLen = Math.hypot(prevVx, prevVy, prevVz);

      let turnAmount = 0;
      if (prevLen > 1e-5 && speed > 1e-5) {
        // 前フレームからの向きの差異を Y 軸回転量として近似
        const crossY = prevVz * vx - prevVx * vz;
        const dot = prevVx * vx + prevVy * vy + prevVz * vz;
        turnAmount = Math.atan2(crossY, dot);
      }

      const tailSpeedValue = animateTail ? speed : 0;
      const tailTurnValue = animateTail ? turnAmount : 0;
      const driveValue = animateTail ? 1 : 0;

      if (isNear) {
        highPosArray[basePos] = px;
        highPosArray[basePos + 1] = py;
        highPosArray[basePos + 2] = pz;
        highQuatArray[baseQuat] = qx;
        highQuatArray[baseQuat + 1] = qy;
        highQuatArray[baseQuat + 2] = qz;
        highQuatArray[baseQuat + 3] = qw;
        highTransformTouched = true;

        if (
          lowPosArray[basePos] !== hiddenPos ||
          lowPosArray[basePos + 1] !== hiddenPos ||
          lowPosArray[basePos + 2] !== hiddenPos ||
          lowQuatArray[baseQuat] !== idQuat[0] ||
          lowQuatArray[baseQuat + 1] !== idQuat[1] ||
          lowQuatArray[baseQuat + 2] !== idQuat[2] ||
          lowQuatArray[baseQuat + 3] !== idQuat[3]
        ) {
          lowPosArray[basePos] = hiddenPos;
          lowPosArray[basePos + 1] = hiddenPos;
          lowPosArray[basePos + 2] = hiddenPos;
          lowQuatArray[baseQuat] = idQuat[0];
          lowQuatArray[baseQuat + 1] = idQuat[1];
          lowQuatArray[baseQuat + 2] = idQuat[2];
          lowQuatArray[baseQuat + 3] = idQuat[3];
          lowTransformTouched = true;
        }

        highTailParamsArray[tailIndex] = tailSpeedValue;
        highTailParamsArray[tailIndex + 1] = tailTurnValue;
        highTailParamsArray[tailIndex + 2] = driveValue;
        highTailTouched = true;

        if (
          lowTailParamsArray[tailIndex] !== 0 ||
          lowTailParamsArray[tailIndex + 1] !== 0 ||
          lowTailParamsArray[tailIndex + 2] !== 0
        ) {
          lowTailParamsArray[tailIndex] = 0;
          lowTailParamsArray[tailIndex + 1] = 0;
          lowTailParamsArray[tailIndex + 2] = 0;
          lowTailTouched = true;
        }
      } else {
        lowPosArray[basePos] = px;
        lowPosArray[basePos + 1] = py;
        lowPosArray[basePos + 2] = pz;
        lowQuatArray[baseQuat] = qx;
        lowQuatArray[baseQuat + 1] = qy;
        lowQuatArray[baseQuat + 2] = qz;
        lowQuatArray[baseQuat + 3] = qw;
        lowTransformTouched = true;

        if (
          highPosArray[basePos] !== hiddenPos ||
          highPosArray[basePos + 1] !== hiddenPos ||
          highPosArray[basePos + 2] !== hiddenPos ||
          highQuatArray[baseQuat] !== idQuat[0] ||
          highQuatArray[baseQuat + 1] !== idQuat[1] ||
          highQuatArray[baseQuat + 2] !== idQuat[2] ||
          highQuatArray[baseQuat + 3] !== idQuat[3]
        ) {
          highPosArray[basePos] = hiddenPos;
          highPosArray[basePos + 1] = hiddenPos;
          highPosArray[basePos + 2] = hiddenPos;
          highQuatArray[baseQuat] = idQuat[0];
          highQuatArray[baseQuat + 1] = idQuat[1];
          highQuatArray[baseQuat + 2] = idQuat[2];
          highQuatArray[baseQuat + 3] = idQuat[3];
          highTransformTouched = true;
        }

        lowTailParamsArray[tailIndex] = tailSpeedValue;
        lowTailParamsArray[tailIndex + 1] = tailTurnValue;
        lowTailParamsArray[tailIndex + 2] = driveValue;
        lowTailTouched = true;

        if (
          highTailParamsArray[tailIndex] !== 0 ||
          highTailParamsArray[tailIndex + 1] !== 0 ||
          highTailParamsArray[tailIndex + 2] !== 0
        ) {
          highTailParamsArray[tailIndex] = 0;
          highTailParamsArray[tailIndex + 1] = 0;
          highTailParamsArray[tailIndex + 2] = 0;
          highTailTouched = true;
        }
      }

      if (lodFlags) {
        lodFlags[i] = isNear ? 1 : 2;
      }

      if (prevVel) {
        prevVel[basePos] = vx;
        prevVel[basePos + 1] = vy;
        prevVel[basePos + 2] = vz;
      }
    }

    const visibleCount = predatorCount > 0 ? Math.max(0, count - predatorCount) : count;
    this.instancedMeshHigh.count = visibleCount;
    this.instancedMeshLow.count = visibleCount;

    this.applyBufferSet(this.instancedMeshHigh, this.bufferSetHigh, currentIndex);
    this.applyBufferSet(this.instancedMeshLow, this.bufferSetLow, currentIndex);

    highPosAttr.needsUpdate = highTransformTouched;
    highQuatAttr.needsUpdate = highTransformTouched;
    lowPosAttr.needsUpdate = lowTransformTouched;
    lowQuatAttr.needsUpdate = lowTransformTouched;
    highTailParamsAttr.needsUpdate = highTailTouched;
    lowTailParamsAttr.needsUpdate = lowTailTouched;

    this.bufferCursor = nextIndex;

    return {
      visibleCount,
      lodFlags,
    };
  }

  forEachInstancingMaterial(callback) {
    for (const material of this.instancingMaterials) {
      callback(material);
    }
  }

  getMeshes() {
    return {
      high: this.instancedMeshHigh,
      low: this.instancedMeshLow,
    };
  }

  getPredatorMeshes() {
    return this.predatorMeshes;
  }

  setTailUniformTime(time) {
    if (!this.tailAnimation?.uniforms) return;
    if (this.tailAnimation.uniforms.uTailTime) {
      this.tailAnimation.uniforms.uTailTime.value = time;
    }
  }

  /**
   * 元マテリアルへ尾びれアニメ用 uniform と頂点シェーダ拡張を差し込みます。
   */
  patchMaterial(material) {
    if (!material || material.userData?.instancingPatched) return;
    const uniforms = this.tailAnimation?.uniforms;
    if (!uniforms) {
      console.warn('BoidInstancing.patchMaterial: tailAnimation.uniforms が未設定です');
    }

    material.onBeforeCompile = (shader) => {
      if (uniforms) {
        // 元シェーダーへ尾びれアニメ用の uniform を全て注入
        Object.entries(uniforms).forEach(([key, uniform]) => {
          shader.uniforms[key] = uniform;
        });
      }

      shader.vertexShader = shader.vertexShader
        .replace(
          '#include <common>',
          `#include <common>\nattribute vec3 instancePos;\nattribute vec4 instanceQuat;\nattribute float aBodyCoord;\nattribute float instanceTailPhase;\nattribute vec3 instanceTailParams;\nuniform float uTailTime;\nuniform float uTailAmplitude;\nuniform float uTailFrequency;\nuniform float uTailPhaseStride;\nuniform float uTailTurnStrength;\nuniform float uTailSpeedScale;\nuniform vec3 uTailRight;\nuniform vec3 uTailForward;\nuniform vec3 uTailUp;\nuniform float uTailEnable;\nuniform sampler2D uSinLut;\nuniform float uLutSize;\nvec3 quatTransform(vec3 v, vec4 q) {\n  vec3 t = 2.0 * cross(q.xyz, v);\n  return v + q.w * t + cross(q.xyz, t);\n}\nvec2 sampleSinCos(float angle) {\n  float u = fract(angle * 0.15915494309189535);\n  u = u * (uLutSize - 1.0) + 0.5;\n  vec4 lutSample = texture(uSinLut, vec2(u / uLutSize, 0.5));\n  return lutSample.rg * 2.0 - 1.0;\n}`
        )
        .replace(
          '#include <instancing_vertex>',
          `#ifdef USE_INSTANCING
#endif
#ifdef USE_INSTANCING_COLOR
  vColor.xyz *= instanceColor.xyz;
#endif`
        )
        .replace(
          '#include <begin_vertex>',
          `#include <begin_vertex>\nvec3 tailParams = instanceTailParams;\nif (uTailEnable > 0.5) {\n  float driveRaw = tailParams.z;\n  if (driveRaw > 0.01) {\n    float drive = clamp(driveRaw, 0.0, 1.0);\n    vec3 originalPos = transformed;\n    vec3 right = normalize(uTailRight);\n    vec3 forward = normalize(uTailForward);\n    vec3 up = normalize(uTailUp);\n\n    float localX = dot(originalPos, right);\n    float localY = dot(originalPos, forward);\n    float localZ = dot(originalPos, up);\n\n    float bodyCoord = clamp(aBodyCoord, 0.0, 1.0);\n    float tailWeight = smoothstep(0.0, 0.35, bodyCoord);\n    float speedFactor = clamp(tailParams.x * uTailSpeedScale, 0.0, 2.0);\n\n    float phase = instanceTailPhase + uTailTime * uTailFrequency;\n    float wavePhase = phase + bodyCoord * uTailPhaseStride;\n    vec2 waveSC = sampleSinCos(wavePhase);\n    float wag = waveSC.x * uTailAmplitude * drive;\n    float turnOffset = tailParams.y * uTailTurnStrength * drive;\n    float motion = wag * (0.4 + 0.6 * speedFactor) + turnOffset;\n\n    float tipDamping = 1.0 - smoothstep(0.7, 1.0, bodyCoord) * 0.3;\n    float bendStrength = mix(0.02, 1.0, tailWeight) * tipDamping;\n\n    float bendAngle = motion * bendStrength;\n    vec2 bendSC = sampleSinCos(bendAngle);\n    float s = bendSC.x;\n    float c = bendSC.y;\n    float rotX = localX * c - localY * s;\n    float rotY = localX * s + localY * c;\n\n    float sway = motion * 0.4 * tipDamping;\n    rotX += sway * (0.05 + 0.95 * bodyCoord);\n\n    vec3 rotated = right * rotX + forward * rotY + up * localZ;\n    transformed = rotated;\n  }\n}\ntransformed = quatTransform(transformed, instanceQuat) + instancePos;`
        )
        .replace(
          '#include <beginnormal_vertex>',
          '#include <beginnormal_vertex>\nobjectNormal = quatTransform(objectNormal, instanceQuat);'
        )
        .replace(
          '#include <project_vertex>',
          `#include <project_vertex>\n#ifdef DEPTH_PACKING\nvec3 tailParams = instanceTailParams;\nif (uTailEnable > 0.5) {\n  float driveRaw = tailParams.z;\n  if (driveRaw > 0.01) {\n    float drive = clamp(driveRaw, 0.0, 1.0);\n    vec3 viewPos = mvPosition.xyz;\n    mat3 normalMatrix3 = mat3(normalMatrix);\n    vec3 right = normalize(normalMatrix3 * uTailRight);\n    vec3 forward = normalize(normalMatrix3 * uTailForward);\n    vec3 up = normalize(normalMatrix3 * uTailUp);\n\n    float localX = dot(viewPos, right);\n    float localY = dot(viewPos, forward);\n    float localZ = dot(viewPos, up);\n\n    float bodyCoord = clamp(aBodyCoord, 0.0, 1.0);\n    float tailWeight = smoothstep(0.0, 0.35, bodyCoord);\n    float speedFactor = clamp(tailParams.x * uTailSpeedScale, 0.0, 2.0);\n\n    float phase = instanceTailPhase + uTailTime * uTailFrequency;\n    float wavePhase = phase + bodyCoord * uTailPhaseStride;\n    vec2 waveSC = sampleSinCos(wavePhase);\n    float wag = waveSC.x * uTailAmplitude * drive;\n    float turnOffset = tailParams.y * uTailTurnStrength * drive;\n    float motion = wag * (0.4 + 0.6 * speedFactor) + turnOffset;\n\n    float tipDamping = 1.0 - smoothstep(0.7, 1.0, bodyCoord) * 0.3;\n    float bendStrength = mix(0.02, 1.0, tailWeight) * tipDamping;\n\n    float bendAngle = motion * bendStrength;\n    vec2 bendSC = sampleSinCos(bendAngle);\n    float s = bendSC.x;\n    float c = bendSC.y;\n    float rotX = localX * c - localY * s;\n    float rotY = localX * s + localY * c;\n\n    float sway = motion * 0.4 * tipDamping;\n    rotX += sway * (0.05 + 0.95 * bodyCoord);\n\n    vec3 rotated = right * rotX + forward * rotY + up * localZ;\n    mvPosition.xyz = rotated;\n    gl_Position = projectionMatrix * mvPosition;\n  }\n}\n#endif`
        );

      material.userData = {
        ...(material.userData || {}),
        instancingPatched: true,
      };
    };

    material.needsUpdate = true;
    this.instancingMaterials.add(material);
  }

  createDepthMaterial(sourceMaterial) {
    const depthMaterial = new THREE.MeshDepthMaterial({
      depthPacking: THREE.RGBADepthPacking,
      alphaTest: sourceMaterial.alphaTest ?? 0,
    });
    depthMaterial.map = sourceMaterial.map ?? null;
    depthMaterial.alphaMap = sourceMaterial.alphaMap ?? null;
    depthMaterial.transparent = sourceMaterial.transparent ?? false;
    this.patchMaterial(depthMaterial);
    return depthMaterial;
  }

  createDistanceMaterial(sourceMaterial) {
    const distanceMaterial = new THREE.MeshDistanceMaterial({
      alphaTest: sourceMaterial.alphaTest ?? 0,
    });
    distanceMaterial.map = sourceMaterial.map ?? null;
    distanceMaterial.alphaMap = sourceMaterial.alphaMap ?? null;
    distanceMaterial.transparent = sourceMaterial.transparent ?? false;
    this.patchMaterial(distanceMaterial);
    return distanceMaterial;
  }

  configureInstancedMesh(mesh, material) {
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.frustumCulled = false;
    mesh.customDepthMaterial = this.createDepthMaterial(material);
    mesh.customDistanceMaterial = this.createDistanceMaterial(material);

    const white = new THREE.Color(1, 1, 1);
    for (let i = 0; i < mesh.count; i++) {
      mesh.setColorAt(i, white);
    }
    if (mesh.instanceColor) {
      mesh.instanceColor.needsUpdate = true;
    }
  }

  createAttributeSet(count, itemSize, ArrayType = Float32Array) {
    const length = count * itemSize;
    const attributes = [];
    for (let i = 0; i < this.tripleBufferSize; i++) {
      const attr = new THREE.InstancedBufferAttribute(new ArrayType(length), itemSize);
      attr.setUsage(this.streamUsage);
      attributes.push(attr);
    }
    return attributes;
  }

  createBufferSet(count) {
    return {
      pos: this.createAttributeSet(count, 3),
      quat: this.createAttributeSet(count, 4),
      tailPhase: this.createAttributeSet(count, 1),
      tailParams: this.createAttributeSet(count, 3),
    };
  }

  resetBufferSetToHidden(bufferSet) {
    // 初期状態では全インスタンスを画面外へ退避させておく
    for (const attr of bufferSet.pos) {
      const array = attr.array;
      for (let i = 0; i < array.length; i += 3) {
        array[i] = this.hiddenPosition;
        array[i + 1] = this.hiddenPosition;
        array[i + 2] = this.hiddenPosition;
      }
    }
    for (const attr of bufferSet.quat) {
      const array = attr.array;
      for (let i = 0; i < array.length; i += 4) {
        array[i] = this.identityQuaternion[0];
        array[i + 1] = this.identityQuaternion[1];
        array[i + 2] = this.identityQuaternion[2];
        array[i + 3] = this.identityQuaternion[3];
      }
    }
    for (const attr of bufferSet.tailPhase) {
      attr.array.fill(0);
    }
    for (const attr of bufferSet.tailParams) {
      attr.array.fill(0);
    }
  }

  applyBufferSet(mesh, bufferSet, index) {
    mesh.geometry.setAttribute('instancePos', bufferSet.pos[index]);
    mesh.geometry.setAttribute('instanceQuat', bufferSet.quat[index]);
    mesh.geometry.setAttribute('instanceTailPhase', bufferSet.tailPhase[index]);
    mesh.geometry.setAttribute('instanceTailParams', bufferSet.tailParams[index]);
  }

  createTailPhaseArray(count) {
    const array = new Float32Array(count);
    for (let i = 0; i < count; i++) {
      array[i] = Math.random() * Math.PI * 2;
    }
    return array;
  }

  applyTailPhaseSeeds(bufferSet, seeds) {
    if (!bufferSet?.tailPhase) return;
    for (const attr of bufferSet.tailPhase) {
      attr.array.set(seeds);
      attr.needsUpdate = true;
    }
  }

  ensureTailRuntimeBuffers(count) {
    const targetLength = count * 3;
    if (!this.previousVelocities || this.previousVelocities.length !== targetLength) {
      this.previousVelocities = new Float32Array(targetLength);
    }
  }

  ensureLodFlagBuffer(count) {
    if (count <= 0) {
      this.previousLodFlags = null;
      return null;
    }
    if (!this.previousLodFlags || this.previousLodFlags.length !== count) {
      this.previousLodFlags = new Uint8Array(count);
    }
    return this.previousLodFlags;
  }

  ensureBodyCoordAttribute(geometry) {
    if (geometry.getAttribute('aBodyCoord')) return;
    const position = geometry.getAttribute('position');
    if (!position) return;

    const count = position.count;
    const array = position.array;
    let minX = Infinity;
    let minY = Infinity;
    let minZ = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    let maxZ = -Infinity;

    for (let i = 0; i < count; i++) {
      const ix = i * 3;
      const x = array[ix];
      const y = array[ix + 1];
      const z = array[ix + 2];
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
    }

    const rangeX = maxX - minX;
    const rangeY = maxY - minY;
    const rangeZ = maxZ - minZ;

    let axis = 'z';
    let min = minZ;
    let range = rangeZ;
    if (rangeX > rangeY && rangeX > rangeZ) {
      axis = 'x';
      min = minX;
      range = rangeX;
    } else if (rangeY > rangeZ) {
      axis = 'y';
      min = minY;
      range = rangeY;
    }

    const bodyCoord = new Float32Array(count);
    const denom = range > 0 ? range : 1;
    for (let i = 0; i < count; i++) {
      const ix = i * 3;
      const value = axis === 'x' ? array[ix] : axis === 'y' ? array[ix + 1] : array[ix + 2];
      bodyCoord[i] = (value - min) / denom;
    }

    // 魚体長に沿った 0-1 の指標を埋め込むことで尾びれアニメの重み付けを制御
    geometry.setAttribute('aBodyCoord', new THREE.BufferAttribute(bodyCoord, 1));
  }

  ensurePredatorMeshes(count) {
    if (!this.scene || !this.predatorModel) return false;

    const target = Math.max(0, count);
    while (this.predatorMeshes.length > target) {
      const mesh = this.predatorMeshes.pop();
      if (mesh) {
        this.scene.remove(mesh);
      }
    }

    while (this.predatorMeshes.length < target) {
      const clone = this.predatorModel.clone(true);
      clone.traverse((child) => {
        if (child.isMesh) {
          child.castShadow = true;
          child.receiveShadow = true;
        }
      });
      clone.visible = false;
      this.scene.add(clone);
      this.predatorMeshes.push(clone);
    }
    return this.predatorMeshes.length === target;
  }

  dispose(scene = this.scene) {
    if (this.instancedMeshHigh) {
      scene?.remove(this.instancedMeshHigh);
      this.instancedMeshHigh.geometry?.dispose?.();
      this.instancedMeshHigh.material?.dispose?.();
      this.instancedMeshHigh = null;
    }
    if (this.instancedMeshLow) {
      scene?.remove(this.instancedMeshLow);
      this.instancedMeshLow.geometry?.dispose?.();
      this.instancedMeshLow.material?.dispose?.();
      this.instancedMeshLow = null;
    }
    if (scene) {
      for (const mesh of this.predatorMeshes) {
        scene.remove(mesh);
      }
    }
    this.instancingMaterials.clear();
    this.bufferSetHigh = null;
    this.bufferSetLow = null;
    this.tailPhaseSeeds = null;
    this.bufferCursor = 0;
    this.previousVelocities = null;
    this.previousLodFlags = null;
    this.predatorMeshes = [];
    this.predatorMeshCountCache = -1;
    this.scene = scene ?? null;
    this.count = 0;
  }
}
