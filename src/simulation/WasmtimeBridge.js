/**
 * WASM モジュールとのやり取りをまとめるラッパークラス。
 * Vue レイヤーでの wasm 呼び出しを一元化し、ポインタのキャッシュと
 * DataView の再利用で GC 負荷を抑えます。
 */
export class WasmtimeBridge {
  constructor(wasmModule) {
    if (!wasmModule) {
      throw new Error('WasmtimeBridge: wasmModule が未定義です');
    }

    this.wasm = wasmModule;
    this.latestStatePtr = 0;
    this.latestStateHeaderPtr = 0;
    this.latestStateHeaderView = null;
    this.latestPositionsPtr = 0;
    this.latestVelocitiesPtr = 0;
    this.latestOrientationsPtr = 0;
    this.latestBoidCount = 0;

    this.cachedHeapBuffer = null;
    this.cachedPositionsPtr = 0;
    this.cachedOrientationsPtr = 0;
    this.cachedVelocitiesPtr = 0;
    this.cachedSpeciesIdsPtr = 0;
    this.cachedPositionsView = null;
    this.cachedOrientationsView = null;
    this.cachedVelocitiesView = null;
    this.cachedSpeciesIdsView = null;
    this.cachedPositionsCount = 0;
    this.cachedOrientationsCount = 0;
    this.cachedVelocitiesCount = 0;
    this.cachedSpeciesIdsCount = 0;
    this.cachedSpeciesIdsBuffer = null;
    this.cachedUnitDensityPtr = 0;
    this.cachedUnitDensityCount = 0;
    this.cachedUnitDensityView = null;
    this.cachedSpeciesEnvelopePtr = 0;
    this.cachedSpeciesEnvelopeCount = 0;
    this.cachedSpeciesEnvelopeView = null;

    this.cachedSpeciesClusterPtr = 0;
    this.cachedSpeciesClusterCount = 0;
    this.cachedSpeciesClusterView = null;

    this.cachedSpeciesSchoolClusterPtr = 0;
    this.cachedSpeciesSchoolClusterCount = 0;
    this.cachedSpeciesSchoolClusterView = null;

    // cwrap された wasm 関数を遅延束縛で保持
    this.stepSimulationHandle = this.createWrappedFunction('stepSimulation', 'number', ['number']);
    this.buildHandle = this.createWrappedFunction('build', 'void', ['number']);
    this.exportTreeStructureHandle = this.createWrappedFunction('exportTreeStructure', 'object', []);
    this.boidUnitMappingPtrHandle = this.createWrappedFunction('boidUnitMappingPtr', 'number', []);
    this.currentFirstBoidXHandle = this.createWrappedFunction('currentFirstBoidX', 'number', []);
    this.speciesIdsPtrHandle = this.createWrappedFunction('speciesIdsPtr', 'number', []);
    this.syncReadToWriteBuffersHandle = this.createWrappedFunction('syncReadToWriteBuffers', 'void', []);
    this.getSimulationTuningParamsHandle = this.createWrappedFunction('getSimulationTuningParams', 'object', []);
    this.setSimulationTuningParamsHandle = this.createWrappedFunction('setSimulationTuningParams', 'void', ['object']);
    this.unitSimpleDensityPtrHandle = this.createWrappedFunction('unitSimpleDensityPtr', 'number', []);
    this.unitSimpleDensityCountHandle = this.createWrappedFunction('unitSimpleDensityCount', 'number', []);
    this.speciesEnvelopesPtrHandle = this.createWrappedFunction('speciesEnvelopesPtr', 'number', []);
    this.speciesEnvelopesCountHandle = this.createWrappedFunction('speciesEnvelopesCount', 'number', []);

    // Species clusters debug export
    this.speciesClustersPtrHandle = this.createWrappedFunction('speciesClustersPtr', 'number', []);
    this.speciesClustersCountHandle = this.createWrappedFunction('speciesClustersCount', 'number', []);

    // Species school clusters debug export
    this.speciesSchoolClustersPtrHandle = this.createWrappedFunction('speciesSchoolClustersPtr', 'number', []);
    this.speciesSchoolClustersCountHandle = this.createWrappedFunction('speciesSchoolClustersCount', 'number', []);

    this.configureGroundPlaneHandle = this.createWrappedFunction(
      'configureGroundPlane',
      'void',
      ['boolean', 'number', 'number', 'number', 'number'],
    );
  }

  getSimulationTuningParams() {
    if (!this.wasm) {
      return null;
    }
    const handle = this.ensureHandle(this.getSimulationTuningParamsHandle, 'getSimulationTuningParams');
    return handle ? handle() : null;
  }

  setSimulationTuningParams(params) {
    if (!this.wasm) {
      return;
    }
    const handle = this.ensureHandle(this.setSimulationTuningParamsHandle, 'setSimulationTuningParams');
    if (!handle || !params) {
      return;
    }
    const toNumber = (value, fallback) => {
      const num = Number(value);
      return Number.isFinite(num) ? num : fallback;
    };
    handle({
      threatDecay: toNumber(params.threatDecay, 0.5),
      maxEscapeWeight: toNumber(params.maxEscapeWeight, 1.0),
      baseEscapeStrength: toNumber(params.baseEscapeStrength, 3.0),
      fastAttractStrength: toNumber(params.fastAttractStrength, 1.0),
      schoolPullCoefficient: Math.max(
        0,
        toNumber(params.schoolPullCoefficient, 0.0008)
      ),
    });
  }

  configureGroundPlane(options) {
    if (!this.wasm) {
      return;
    }
    const handle = this.ensureHandle(this.configureGroundPlaneHandle, 'configureGroundPlane');
    if (!handle || !options) {
      return;
    }
    const toNumber = (value, fallback) => {
      const num = Number(value);
      return Number.isFinite(num) ? num : fallback;
    };
    const enabled = options.enabled !== false;
    const height = toNumber(options.height, 0);
    const blendDistance = Math.max(0, toNumber(options.blendDistance, 2));
    const stiffness = Math.max(0, toNumber(options.stiffness, 18));
    const damping = Math.max(0, toNumber(options.damping, 8));
    handle(enabled, height, blendDistance, stiffness, damping);
  }

  /**
   * 種族設定から wasm に渡すベクタを生成。
   * App.vue::createSpeciesParamsVector を移植してください。
   */
  buildSpeciesParams(settings) {
    if (!Array.isArray(settings)) {
      return null;
    }

    // wasm 側の VectorSpeciesParams を生成し、一件ずつ型を整えて push
    const vector = new this.wasm.VectorSpeciesParams();
    settings.forEach((raw, index) => {
      if (!raw) {
        return;
      }

      const toNumber = (value, fallback = 0) => {
        const num = Number(value);
        return Number.isFinite(num) ? num : fallback;
      };

      vector.push_back({
        species: typeof raw.species === 'string' ? raw.species : `Species ${index + 1}`,
        count: Math.max(0, Math.floor(toNumber(raw.count, 0))),
        speciesId: Math.max(0, Math.floor(toNumber(raw.speciesId, index))),
        cohesion: toNumber(raw.cohesion, 0),
        separation: toNumber(raw.separation, 0),
        alignment: toNumber(raw.alignment, 0),
        maxSpeed: toNumber(raw.maxSpeed, 1),
        minSpeed: toNumber(raw.minSpeed, 0),
        maxTurnAngle: toNumber(raw.maxTurnAngle, 0),
        separationRange: toNumber(raw.separationRange, 0),
        alignmentRange: toNumber(raw.alignmentRange, 0),
        cohesionRange: toNumber(raw.cohesionRange, 0),
        maxNeighbors: Math.max(0, Math.floor(toNumber(raw.maxNeighbors, 0))),
        lambda: toNumber(raw.lambda, 0),
        tau: toNumber(raw.tau, 0),
        horizontalTorque: toNumber(raw.horizontalTorque, 0),
        velocityEpsilon: toNumber(raw.velocityEpsilon, 0.0001),
        torqueStrength: toNumber(raw.torqueStrength, 0),
        // 形状パラメータは省略された場合もデフォルト値で補完しておく。
        bodyHeadLength: toNumber(raw.bodyHeadLength, -0.15),
        bodyTailLength: toNumber(raw.bodyTailLength, 0.33),
        bodyRadius: toNumber(raw.bodyRadius, 0.035),
        predatorAlertRadius: toNumber(raw.predatorAlertRadius, 1.0),
        isPredator: Boolean(raw.isPredator),
        densityReturnStrength: toNumber(raw.densityReturnStrength ?? raw.centerAttractStrength, 0.0),
      });
    });
    return vector;
  }

  /**
   * シミュレーションを 1 ステップ進めて最新ポインタを更新します。
   * App.vue::stepSimulationAndUpdateState を移植。
   */
  stepSimulation(deltaTime) {
    const handle = this.ensureHandle(this.stepSimulationHandle, 'stepSimulation');

    const { wasm } = this;
    const dt = Number.isFinite(deltaTime) ? deltaTime : 0;
    const statePtr = handle(dt);
    if (!statePtr) {
      this.latestStatePtr = 0;
      this.latestStateHeaderPtr = 0;
      this.latestPositionsPtr = 0;
      this.latestVelocitiesPtr = 0;
      this.latestOrientationsPtr = 0;
      this.latestStateHeaderView = null;
      this.latestBoidCount = 0;
      return 0;
    }

    const heapU8Buffer = wasm.HEAPU8.buffer;
    if (
      !this.latestStateHeaderView ||
      this.latestStateHeaderPtr !== statePtr ||
      this.latestStateHeaderView.buffer !== heapU8Buffer
    ) {
      this.latestStateHeaderView = new DataView(heapU8Buffer, statePtr, 16);
      this.latestStateHeaderPtr = statePtr;
    }

    this.latestStatePtr = statePtr;
    this.latestPositionsPtr = this.latestStateHeaderView.getUint32(0, true);
    this.latestVelocitiesPtr = this.latestStateHeaderView.getUint32(4, true);
    this.latestOrientationsPtr = this.latestStateHeaderView.getUint32(8, true);
    this.latestBoidCount = this.latestStateHeaderView.getInt32(12, true);

    return this.latestBoidCount;
  }

  /**
   * wasm 側の positions / orientations 等のビューを再利用して返す。
   * buffer が差し替わった場合のみ新しい TypedArray を生成する。
   */
  getBuffers(count) {
    if (!this.wasm) {
      return {
        positions: new Float32Array(0),
        orientations: new Float32Array(0),
        velocities: new Float32Array(0),
        speciesIds: new Int32Array(0),
      };
    }

    const heapBuffer = this.wasm.HEAPF32.buffer;
    const heapBufferI32 = this.wasm.HEAP32.buffer;

    if (this.cachedHeapBuffer !== heapBuffer || this.cachedSpeciesIdsBuffer !== heapBufferI32) {
      this.cachedHeapBuffer = heapBuffer;
      this.cachedSpeciesIdsBuffer = heapBufferI32;
      this.cachedPositionsPtr = 0;
      this.cachedOrientationsPtr = 0;
      this.cachedVelocitiesPtr = 0;
      this.cachedSpeciesIdsPtr = 0;
      this.cachedPositionsView = null;
      this.cachedOrientationsView = null;
      this.cachedVelocitiesView = null;
      this.cachedSpeciesIdsView = null;
      this.cachedPositionsCount = 0;
      this.cachedOrientationsCount = 0;
      this.cachedVelocitiesCount = 0;
      this.cachedSpeciesIdsCount = 0;
      this.cachedUnitDensityPtr = 0;
      this.cachedUnitDensityCount = 0;
      this.cachedUnitDensityView = null;
      this.cachedSpeciesEnvelopePtr = 0;
      this.cachedSpeciesEnvelopeCount = 0;
      this.cachedSpeciesEnvelopeView = null;

      this.cachedSpeciesClusterPtr = 0;
      this.cachedSpeciesClusterCount = 0;
      this.cachedSpeciesClusterView = null;

      this.cachedSpeciesSchoolClusterPtr = 0;
      this.cachedSpeciesSchoolClusterCount = 0;
      this.cachedSpeciesSchoolClusterView = null;
    }

    if (!this.latestPositionsPtr || !this.latestOrientationsPtr || !this.latestVelocitiesPtr || count <= 0) {
      return {
        positions: this.cachedPositionsView ?? new Float32Array(0),
        orientations: this.cachedOrientationsView ?? new Float32Array(0),
        velocities: this.cachedVelocitiesView ?? new Float32Array(0),
        speciesIds: this.cachedSpeciesIdsView ?? new Int32Array(0),
      };
    }

    if (
      this.cachedPositionsView === null ||
      this.cachedPositionsCount !== count ||
      this.cachedPositionsPtr !== this.latestPositionsPtr
    ) {
      this.cachedPositionsPtr = this.latestPositionsPtr;
      this.cachedPositionsView = new Float32Array(heapBuffer, this.cachedPositionsPtr, count * 3);
      this.cachedPositionsCount = count;
    }

    if (
      this.cachedOrientationsView === null ||
      this.cachedOrientationsCount !== count ||
      this.cachedOrientationsPtr !== this.latestOrientationsPtr
    ) {
      this.cachedOrientationsPtr = this.latestOrientationsPtr;
      this.cachedOrientationsView = new Float32Array(heapBuffer, this.cachedOrientationsPtr, count * 4);
      this.cachedOrientationsCount = count;
    }

    if (
      this.cachedVelocitiesView === null ||
      this.cachedVelocitiesCount !== count ||
      this.cachedVelocitiesPtr !== this.latestVelocitiesPtr
    ) {
      this.cachedVelocitiesPtr = this.latestVelocitiesPtr;
      this.cachedVelocitiesView = new Float32Array(heapBuffer, this.cachedVelocitiesPtr, count * 3);
      this.cachedVelocitiesCount = count;
    }

    const speciesPtr = typeof this.speciesIdsPtrHandle === 'function' ? this.speciesIdsPtrHandle() : 0;
    if (
      speciesPtr && (
        this.cachedSpeciesIdsView === null ||
        this.cachedSpeciesIdsPtr !== speciesPtr ||
        this.cachedSpeciesIdsCount !== count ||
        this.cachedSpeciesIdsBuffer !== heapBufferI32
      )
    ) {
      this.cachedSpeciesIdsPtr = speciesPtr;
      this.cachedSpeciesIdsCount = count;
      this.cachedSpeciesIdsBuffer = heapBufferI32;
      this.cachedSpeciesIdsView = new Int32Array(heapBufferI32, speciesPtr, count);
    }

    return {
      positions: this.cachedPositionsView,
      orientations: this.cachedOrientationsView,
      velocities: this.cachedVelocitiesView,
      speciesIds: this.cachedSpeciesIdsView,
    };
  }

  /** wasm の triple buffer から現状態をコピーしてスナップショットを作る */
  captureState() {
    if (!this.wasm) {
      return null;
    }

    const currentCount = this.stepSimulation(0);
    if (!currentCount || currentCount <= 0) {
      return { count: 0 };
    }

    const { positions, velocities, orientations, speciesIds } = this.getBuffers(currentCount);

    return {
      count: currentCount,
      positions: positions ? new Float32Array(positions) : null,
      velocities: velocities ? new Float32Array(velocities) : null,
      orientations: orientations ? new Float32Array(orientations) : null,
      speciesIds: speciesIds ? new Int32Array(speciesIds) : null,
    };
  }

  /** 過去の状態を復元。restoreFlockState を移植。 */
  /** 過去のスナップショットを現在のバッファへ復元する */
  restoreState(previousState, oldSettings, newSettings) {
    if (!previousState || previousState.count <= 0) {
      return;
    }

    const currentCount = this.stepSimulation(0);
    if (!currentCount || currentCount <= 0) {
      return;
    }

    const { positions, velocities, orientations } = this.getBuffers(currentCount);
    if (!positions || !velocities || !orientations) {
      return;
    }

    const prevPositions = previousState.positions;
    const prevVelocities = previousState.velocities;
    const prevOrientations = previousState.orientations;

    const oldRanges = buildSpeciesRanges(oldSettings);
    const newRanges = buildSpeciesRanges(newSettings);
    const oldRangeMap = new Map(oldRanges.map((info) => [info.name, info]));

    let restored = 0;

    for (const newInfo of newRanges) {
      if (!newInfo.count) continue;
      const oldInfo = oldRangeMap.get(newInfo.name);
      if (!oldInfo || !oldInfo.count) continue;

      const transferable = Math.min(oldInfo.count, newInfo.count);
      for (let i = 0; i < transferable; i += 1) {
        const srcIndex = oldInfo.start + i;
        const dstIndex = newInfo.start + i;
        if (srcIndex >= previousState.count || dstIndex >= currentCount) {
          break;
        }

        if (prevPositions) {
          const srcPosBase = srcIndex * 3;
          const dstPosBase = dstIndex * 3;
          positions[dstPosBase] = prevPositions[srcPosBase];
          positions[dstPosBase + 1] = prevPositions[srcPosBase + 1];
          positions[dstPosBase + 2] = prevPositions[srcPosBase + 2];
        }

        if (prevVelocities) {
          const srcVelBase = srcIndex * 3;
          const dstVelBase = dstIndex * 3;
          velocities[dstVelBase] = prevVelocities[srcVelBase];
          velocities[dstVelBase + 1] = prevVelocities[srcVelBase + 1];
          velocities[dstVelBase + 2] = prevVelocities[srcVelBase + 2];
        }

        if (prevOrientations) {
          const srcOriBase = srcIndex * 4;
          const dstOriBase = dstIndex * 4;
          orientations[dstOriBase] = prevOrientations[srcOriBase];
          orientations[dstOriBase + 1] = prevOrientations[srcOriBase + 1];
          orientations[dstOriBase + 2] = prevOrientations[srcOriBase + 2];
          orientations[dstOriBase + 3] = prevOrientations[srcOriBase + 3];
        }
        restored += 1;
      }
    }

    if (restored > 0) {
      this.syncReadToWriteBuffers();
    }
  }

  /** wasm 側の空間インデックス生成をトリガー */
  buildSpatialIndex(maxPerUnit) {
    const handle = this.ensureHandle(this.buildHandle, 'build');
    handle(maxPerUnit);
  }

  /** wasm から木構造データを取得 */
  exportTreeStructure() {
    if (typeof this.exportTreeStructureHandle !== 'function') {
      return null;
    }
    return this.exportTreeStructureHandle();
  }

  /** デバッグ用の先頭 Boid の X 座標取得 */
  currentFirstBoidX() {
    if (typeof this.currentFirstBoidXHandle !== 'function') {
      return 0;
    }
    return this.currentFirstBoidXHandle();
  }

  /** ユニット ID と Boid インデックスのマッピングバッファを返す */
  getUnitMappings(count) {
    if (!count || count <= 0 || typeof this.boidUnitMappingPtrHandle !== 'function') {
      return null;
    }

    const ptr = this.boidUnitMappingPtrHandle();
    if (!ptr) {
      return null;
    }

    return new Int32Array(this.wasm.HEAP32.buffer, ptr, count * 2);
  }

  /** ユニット密度スカラーの Float32Array を返す */
  getUnitSimpleDensities() {
    if (!this.wasm) {
      return null;
    }
    if (
      typeof this.unitSimpleDensityPtrHandle !== 'function' ||
      typeof this.unitSimpleDensityCountHandle !== 'function'
    ) {
      return null;
    }

    const count = this.unitSimpleDensityCountHandle();
    const ptr = this.unitSimpleDensityPtrHandle();
    if (!ptr || !count || count <= 0) {
      this.cachedUnitDensityPtr = 0;
      this.cachedUnitDensityCount = 0;
      this.cachedUnitDensityView = null;
      return null;
    }

    const heapBuffer = this.wasm.HEAPF32.buffer;
    if (
      this.cachedUnitDensityView === null ||
      this.cachedUnitDensityPtr !== ptr ||
      this.cachedUnitDensityCount !== count ||
      this.cachedHeapBuffer !== heapBuffer
    ) {
      this.cachedUnitDensityPtr = ptr;
      this.cachedUnitDensityCount = count;
      this.cachedUnitDensityView = new Float32Array(heapBuffer, ptr, count);
    }

    return this.cachedUnitDensityView;
  }

  /** 種族エンベロープ (center.xyz, radius, count) の Float32Array を返す */
  getSpeciesEnvelopes() {
    if (!this.wasm) {
      return null;
    }

    if (
      typeof this.speciesEnvelopesPtrHandle !== 'function' ||
      typeof this.speciesEnvelopesCountHandle !== 'function'
    ) {
      return null;
    }

    const floatCount = this.speciesEnvelopesCountHandle();
    const ptr = this.speciesEnvelopesPtrHandle();
    if (!ptr || !floatCount || floatCount <= 0) {
      this.cachedSpeciesEnvelopePtr = 0;
      this.cachedSpeciesEnvelopeCount = 0;
      this.cachedSpeciesEnvelopeView = null;
      return null;
    }

    const heapBuffer = this.wasm.HEAPF32.buffer;
    if (
      this.cachedSpeciesEnvelopeView === null ||
      this.cachedSpeciesEnvelopePtr !== ptr ||
      this.cachedSpeciesEnvelopeCount !== floatCount ||
      this.cachedHeapBuffer !== heapBuffer
    ) {
      this.cachedSpeciesEnvelopePtr = ptr;
      this.cachedSpeciesEnvelopeCount = floatCount;
      this.cachedSpeciesEnvelopeView = new Float32Array(heapBuffer, ptr, floatCount);
      this.cachedHeapBuffer = heapBuffer;
    }

    const envelopeCount = Math.floor(floatCount / 5);
    return {
      buffer: this.cachedSpeciesEnvelopeView,
      count: envelopeCount,
    };
  }

  /** 種族クラスター (speciesId, center.xyz, radius, weight) の Float32Array を返す */
  getSpeciesClusters() {
    if (!this.wasm) {
      return null;
    }

    if (
      typeof this.speciesClustersPtrHandle !== 'function' ||
      typeof this.speciesClustersCountHandle !== 'function'
    ) {
      return null;
    }

    const floatCount = this.speciesClustersCountHandle();
    const ptr = this.speciesClustersPtrHandle();
    if (!ptr || !floatCount || floatCount <= 0) {
      this.cachedSpeciesClusterPtr = 0;
      this.cachedSpeciesClusterCount = 0;
      this.cachedSpeciesClusterView = null;
      return null;
    }

    const heapBuffer = this.wasm.HEAPF32.buffer;
    if (
      this.cachedSpeciesClusterView === null ||
      this.cachedSpeciesClusterPtr !== ptr ||
      this.cachedSpeciesClusterCount !== floatCount ||
      this.cachedHeapBuffer !== heapBuffer
    ) {
      this.cachedSpeciesClusterPtr = ptr;
      this.cachedSpeciesClusterCount = floatCount;
      this.cachedSpeciesClusterView = new Float32Array(heapBuffer, ptr, floatCount);
      this.cachedHeapBuffer = heapBuffer;
    }

    const clusterCount = Math.floor(floatCount / 6);
    return {
      buffer: this.cachedSpeciesClusterView,
      count: clusterCount,
    };
  }

  /** 種族の群れクラスタ (speciesId, center.xyz, radius, weight) の Float32Array を返す */
  getSpeciesSchoolClusters() {
    if (!this.wasm) {
      return null;
    }

    if (
      typeof this.speciesSchoolClustersPtrHandle !== 'function' ||
      typeof this.speciesSchoolClustersCountHandle !== 'function'
    ) {
      return null;
    }

    const floatCount = this.speciesSchoolClustersCountHandle();
    const ptr = this.speciesSchoolClustersPtrHandle();
    if (!ptr || !floatCount || floatCount <= 0) {
      this.cachedSpeciesSchoolClusterPtr = 0;
      this.cachedSpeciesSchoolClusterCount = 0;
      this.cachedSpeciesSchoolClusterView = null;
      return null;
    }

    const heapBuffer = this.wasm.HEAPF32.buffer;
    if (
      this.cachedSpeciesSchoolClusterView === null ||
      this.cachedSpeciesSchoolClusterPtr !== ptr ||
      this.cachedSpeciesSchoolClusterCount !== floatCount ||
      this.cachedHeapBuffer !== heapBuffer
    ) {
      this.cachedSpeciesSchoolClusterPtr = ptr;
      this.cachedSpeciesSchoolClusterCount = floatCount;
      this.cachedSpeciesSchoolClusterView = new Float32Array(heapBuffer, ptr, floatCount);
      this.cachedHeapBuffer = heapBuffer;
    }

    const clusterCount = Math.floor(floatCount / 6);
    return {
      buffer: this.cachedSpeciesSchoolClusterView,
      count: clusterCount,
    };
  }

  /** wasm の triple buffer 読み書きを同期 */
  syncReadToWriteBuffers() {
    if (typeof this.syncReadToWriteBuffersHandle === 'function') {
      this.syncReadToWriteBuffersHandle();
    }
  }

  /** cwrap または素の関数を返し、存在しなければ null */
  createWrappedFunction(name, returnType, argTypes) {
    const { wasm } = this;
    if (!wasm) {
      return null;
    }

    if (typeof wasm[name] === 'function') {
      return wasm[name].bind(wasm);
    }

    if (typeof wasm.cwrap === 'function') {
      try {
        return wasm.cwrap(name, returnType, argTypes);
      } catch (error) {
        console.warn(`WasmtimeBridge: ${name} のラップに失敗`, error);
      }
    }
    return null;
  }

  /** ハンドルが未初期化でないことを確認 */
  ensureHandle(handle, name) {
    if (typeof handle !== 'function') {
      throw new Error(`WasmtimeBridge: ${name} ハンドルが初期化されていません`);
    }
    return handle;
  }
}

// 連番の設定配列から {name,start,count} を生成し種族ごとの範囲を求める
function buildSpeciesRanges(list = []) {
  const ranges = [];
  let offset = 0;
  for (const item of list) {
    if (!item) continue;
    const count = Math.max(0, Number(item.count) || 0);
    ranges.push({
      name: item.species || '',
      start: offset,
      count,
    });
    offset += count;
  }
  return ranges;
}
