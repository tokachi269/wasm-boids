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
    this.stepSimulationHandle = createWrappedFunction(this.wasm, 'stepSimulation', 'number', ['number']);
    // 空間インデックス再構築（引数なし）
    this.buildHandle = createWrappedFunction(this.wasm, 'build', 'void', []);
    this.exportTreeStructureHandle = createWrappedFunction(this.wasm, 'exportTreeStructure', 'object', []);
    this.boidUnitMappingPtrHandle = createWrappedFunction(this.wasm, 'boidUnitMappingPtr', 'number', []);
    this.currentFirstBoidXHandle = createWrappedFunction(this.wasm, 'currentFirstBoidX', 'number', []);
    this.speciesIdsPtrHandle = createWrappedFunction(this.wasm, 'speciesIdsPtr', 'number', []);
    this.syncReadToWriteBuffersHandle = createWrappedFunction(this.wasm, 'syncReadToWriteBuffers', 'void', []);
    this.setSimulationTuningParamsHandle = createWrappedFunction(this.wasm, 'setSimulationTuningParams', 'void', ['object']);
    this.unitSimpleDensityPtrHandle = createWrappedFunction(this.wasm, 'unitSimpleDensityPtr', 'number', []);
    this.unitSimpleDensityCountHandle = createWrappedFunction(this.wasm, 'unitSimpleDensityCount', 'number', []);
    this.speciesEnvelopesPtrHandle = createWrappedFunction(this.wasm, 'speciesEnvelopesPtr', 'number', []);
    this.speciesEnvelopesCountHandle = createWrappedFunction(this.wasm, 'speciesEnvelopesCount', 'number', []);

    // Species clusters debug export
    this.speciesClustersPtrHandle = createWrappedFunction(this.wasm, 'speciesClustersPtr', 'number', []);
    this.speciesClustersCountHandle = createWrappedFunction(this.wasm, 'speciesClustersCount', 'number', []);

    // Species school clusters debug export
    this.speciesSchoolClustersPtrHandle = createWrappedFunction(this.wasm, 'speciesSchoolClustersPtr', 'number', []);
    this.speciesSchoolClustersCountHandle = createWrappedFunction(this.wasm, 'speciesSchoolClustersCount', 'number', []);

    this.configureGroundPlaneHandle = createWrappedFunction(
      this.wasm,
      'configureGroundPlane',
      'void',
      ['boolean', 'number', 'number', 'number', 'number'],
    );
    this.cachedUnitMappingsPtr = 0;
    this.cachedUnitMappingsCount = 0;
    this.cachedUnitMappingsView = null;
    this.cachedUnitMappingsBuffer = null;

    // 解析/デバッグ用の戻り値は同一オブジェクトを使い回す（毎フレームのGCを避ける）。
    this.cachedDiagnostics = {
      treeStructure: null,
      firstBoidX: 0,
      unitMappings: null,
      unitDensities: null,
      speciesEnvelopes: null,
      speciesClusters: null,
      speciesSchoolClusters: null,
    };
  }

  /** 群れの初期化を 1 箇所に集約（呼び順を隠蔽） */
  initializeFlock(settings, options = {}) {
    if (!this.wasm) {
      return 0;
    }

    const spatialScale = Number.isFinite(options.spatialScale)
      ? Number(options.spatialScale)
      : 1;
    const posRange = Number.isFinite(options.posRange) ? Number(options.posRange) : 4;
    const velRange = Number.isFinite(options.velRange) ? Number(options.velRange) : 0.25;

    // 種族設定を wasm 側の VectorSpeciesParams に変換
    const source = Array.isArray(settings) ? settings : Array.from(settings ?? []);
    const vector = buildSpeciesParams(this.wasm, source);
    if (!vector) {
      return 0;
    }

    // wasm 側で初期化
    try {
      if (typeof this.wasm.callInitBoids !== 'function') {
        throw new Error('WasmtimeBridge: callInitBoids が見つかりません');
      }
      this.wasm.callInitBoids(vector, spatialScale, posRange, velRange);
    } finally {
      if (typeof vector.delete === 'function') {
        vector.delete();
      }
    }

    // 初期化時に環境（地面など）を先に適用して、以降の更新に影響を漏らさない。
    if (options.groundPlane) {
      applyGroundPlane(this.configureGroundPlaneHandle, options.groundPlane);
    }

    // 空間インデックスを再構築
    ensureHandle(this.buildHandle, 'build')();

    // 最新ポインタを更新して総数を返す
    return this.stepSimulation(0);
  }

  /** 種族設定を wasm へ反映（vector生成/破棄まで含める） */
  applySpeciesParams(settings, options = {}) {
    if (!this.wasm || typeof this.wasm.setGlobalSpeciesParamsFromJS !== 'function') {
      return false;
    }

    const spatialScale = Number.isFinite(options.spatialScale)
      ? Number(options.spatialScale)
      : 1;

    const source = Array.isArray(settings) ? settings : Array.from(settings ?? []);
    const vector = buildSpeciesParams(this.wasm, source);
    if (!vector) {
      return false;
    }

    try {
      this.wasm.setGlobalSpeciesParamsFromJS(vector, spatialScale);
    } finally {
      if (typeof vector.delete === 'function') {
        vector.delete();
      }
    }
    return true;
  }

  /** チューニング反映（呼び出し側はこれだけ使う） */
  applyTuningParams(params) {
    if (!this.wasm) {
      return;
    }
    if (!params) {
      return;
    }
    const handle = ensureHandle(this.setSimulationTuningParamsHandle, 'setSimulationTuningParams');
    const toNumber = (value, fallback) => {
      const num = Number(value);
      return Number.isFinite(num) ? num : fallback;
    };
    handle({
      threatDecay: toNumber(params.threatDecay, 0.5),
      maxEscapeWeight: toNumber(params.maxEscapeWeight, 1.0),
      baseEscapeStrength: toNumber(params.baseEscapeStrength, 3.0),
      fastAttractStrength: toNumber(params.fastAttractStrength, 1.0),
      schoolPullCoefficient: Math.max(0, toNumber(params.schoolPullCoefficient, 0.0008)),
    });
  }

  /** 地面設定（初期化手順からだけ呼ぶ） */
  /** 解析/デバッグ用データをまとめて返す（必要なものだけ取る） */
  getDiagnostics(options = {}) {
    if (!this.wasm) {
      return null;
    }

    const resolved = (options && typeof options === 'object') ? options : {};
    const boidCount = Number.isFinite(resolved.boidCount) ? Math.max(0, Math.floor(resolved.boidCount)) : 0;
    const includeTreeStructure = Boolean(resolved.treeStructure);
    const includeFirstBoidX = Boolean(resolved.firstBoidX);
    const includeUnitMappings = Boolean(resolved.unitMappings);
    const includeUnitDensities = Boolean(resolved.unitDensities);
    const includeSpeciesEnvelopes = Boolean(resolved.speciesEnvelopes);
    const includeSpeciesClusters = Boolean(resolved.speciesClusters);
    const includeSpeciesSchoolClusters = Boolean(resolved.speciesSchoolClusters);

    const result = this.cachedDiagnostics;
    result.treeStructure =
      includeTreeStructure && typeof this.exportTreeStructureHandle === 'function'
        ? this.exportTreeStructureHandle()
        : null;

    result.firstBoidX =
      includeFirstBoidX && typeof this.currentFirstBoidXHandle === 'function'
        ? this.currentFirstBoidXHandle()
        : 0;

    // ユニットマッピング（boidIndex -> unitId）は必要時だけ作る。
    if (includeUnitMappings && boidCount > 0 && typeof this.boidUnitMappingPtrHandle === 'function') {
      const ptr = this.boidUnitMappingPtrHandle();
      const heapBufferI32 = this.wasm.HEAP32.buffer;
      if (ptr && (
        this.cachedUnitMappingsView === null ||
        this.cachedUnitMappingsPtr !== ptr ||
        this.cachedUnitMappingsCount !== boidCount ||
        this.cachedUnitMappingsBuffer !== heapBufferI32
      )) {
        this.cachedUnitMappingsPtr = ptr;
        this.cachedUnitMappingsCount = boidCount;
        this.cachedUnitMappingsBuffer = heapBufferI32;
        this.cachedUnitMappingsView = new Int32Array(heapBufferI32, ptr, boidCount * 2);
      }
      result.unitMappings = this.cachedUnitMappingsView;
    } else {
      result.unitMappings = null;
    }

    // ユニット密度
    result.unitDensities = includeUnitDensities ? getUnitSimpleDensitiesInternal(this) : null;

    // 種族の可視化データ
    result.speciesEnvelopes = includeSpeciesEnvelopes ? getSpeciesEnvelopesInternal(this) : null;
    result.speciesClusters = includeSpeciesClusters ? getSpeciesClustersInternal(this) : null;
    result.speciesSchoolClusters = includeSpeciesSchoolClusters ? getSpeciesSchoolClustersInternal(this) : null;

    return result;
  }

  /**
   * シミュレーションを 1 ステップ進めて最新ポインタを更新します。
   * App.vue::stepSimulationAndUpdateState を移植。
   */
  stepSimulation(deltaTime) {
    const handle = ensureHandle(this.stepSimulationHandle, 'stepSimulation');

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
      if (typeof this.syncReadToWriteBuffersHandle === 'function') {
        this.syncReadToWriteBuffersHandle();
      }
    }
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

// cwrap または素の関数を返し、存在しなければ null
function createWrappedFunction(wasm, name, returnType, argTypes) {
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

// ハンドルが未初期化でないことを確認
function ensureHandle(handle, name) {
  if (typeof handle !== 'function') {
    throw new Error(`WasmtimeBridge: ${name} ハンドルが初期化されていません`);
  }
  return handle;
}

// 地面設定は初期化手順からのみ適用する
function applyGroundPlane(configureGroundPlaneHandle, options) {
  if (!options) {
    return;
  }
  const handle = ensureHandle(configureGroundPlaneHandle, 'configureGroundPlane');

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

// 種族設定を wasm VectorSpeciesParams に変換
function buildSpeciesParams(wasm, settings) {
  if (!wasm || !Array.isArray(settings)) {
    return null;
  }

  const vector = new wasm.VectorSpeciesParams();
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

function getUnitSimpleDensitiesInternal(bridge) {
  if (!bridge?.wasm) {
    return null;
  }
  if (
    typeof bridge.unitSimpleDensityPtrHandle !== 'function' ||
    typeof bridge.unitSimpleDensityCountHandle !== 'function'
  ) {
    return null;
  }

  const count = bridge.unitSimpleDensityCountHandle();
  const ptr = bridge.unitSimpleDensityPtrHandle();
  if (!ptr || !count || count <= 0) {
    bridge.cachedUnitDensityPtr = 0;
    bridge.cachedUnitDensityCount = 0;
    bridge.cachedUnitDensityView = null;
    return null;
  }

  const heapBuffer = bridge.wasm.HEAPF32.buffer;
  if (
    bridge.cachedUnitDensityView === null ||
    bridge.cachedUnitDensityPtr !== ptr ||
    bridge.cachedUnitDensityCount !== count ||
    bridge.cachedHeapBuffer !== heapBuffer
  ) {
    bridge.cachedUnitDensityPtr = ptr;
    bridge.cachedUnitDensityCount = count;
    bridge.cachedUnitDensityView = new Float32Array(heapBuffer, ptr, count);
  }

  return bridge.cachedUnitDensityView;
}

function getSpeciesEnvelopesInternal(bridge) {
  if (!bridge?.wasm) {
    return null;
  }
  if (
    typeof bridge.speciesEnvelopesPtrHandle !== 'function' ||
    typeof bridge.speciesEnvelopesCountHandle !== 'function'
  ) {
    return null;
  }

  const floatCount = bridge.speciesEnvelopesCountHandle();
  const ptr = bridge.speciesEnvelopesPtrHandle();
  if (!ptr || !floatCount || floatCount <= 0) {
    bridge.cachedSpeciesEnvelopePtr = 0;
    bridge.cachedSpeciesEnvelopeCount = 0;
    bridge.cachedSpeciesEnvelopeView = null;
    return null;
  }

  const heapBuffer = bridge.wasm.HEAPF32.buffer;
  if (
    bridge.cachedSpeciesEnvelopeView === null ||
    bridge.cachedSpeciesEnvelopePtr !== ptr ||
    bridge.cachedSpeciesEnvelopeCount !== floatCount ||
    bridge.cachedHeapBuffer !== heapBuffer
  ) {
    bridge.cachedSpeciesEnvelopePtr = ptr;
    bridge.cachedSpeciesEnvelopeCount = floatCount;
    bridge.cachedSpeciesEnvelopeView = new Float32Array(heapBuffer, ptr, floatCount);
    bridge.cachedHeapBuffer = heapBuffer;
  }

  return {
    buffer: bridge.cachedSpeciesEnvelopeView,
    count: Math.floor(floatCount / 5),
  };
}

function getSpeciesClustersInternal(bridge) {
  if (!bridge?.wasm) {
    return null;
  }
  if (
    typeof bridge.speciesClustersPtrHandle !== 'function' ||
    typeof bridge.speciesClustersCountHandle !== 'function'
  ) {
    return null;
  }

  const floatCount = bridge.speciesClustersCountHandle();
  const ptr = bridge.speciesClustersPtrHandle();
  if (!ptr || !floatCount || floatCount <= 0) {
    bridge.cachedSpeciesClusterPtr = 0;
    bridge.cachedSpeciesClusterCount = 0;
    bridge.cachedSpeciesClusterView = null;
    return null;
  }

  const heapBuffer = bridge.wasm.HEAPF32.buffer;
  if (
    bridge.cachedSpeciesClusterView === null ||
    bridge.cachedSpeciesClusterPtr !== ptr ||
    bridge.cachedSpeciesClusterCount !== floatCount ||
    bridge.cachedHeapBuffer !== heapBuffer
  ) {
    bridge.cachedSpeciesClusterPtr = ptr;
    bridge.cachedSpeciesClusterCount = floatCount;
    bridge.cachedSpeciesClusterView = new Float32Array(heapBuffer, ptr, floatCount);
    bridge.cachedHeapBuffer = heapBuffer;
  }

  return {
    buffer: bridge.cachedSpeciesClusterView,
    count: Math.floor(floatCount / 6),
  };
}

function getSpeciesSchoolClustersInternal(bridge) {
  if (!bridge?.wasm) {
    return null;
  }
  if (
    typeof bridge.speciesSchoolClustersPtrHandle !== 'function' ||
    typeof bridge.speciesSchoolClustersCountHandle !== 'function'
  ) {
    return null;
  }

  const floatCount = bridge.speciesSchoolClustersCountHandle();
  const ptr = bridge.speciesSchoolClustersPtrHandle();
  if (!ptr || !floatCount || floatCount <= 0) {
    bridge.cachedSpeciesSchoolClusterPtr = 0;
    bridge.cachedSpeciesSchoolClusterCount = 0;
    bridge.cachedSpeciesSchoolClusterView = null;
    return null;
  }

  const heapBuffer = bridge.wasm.HEAPF32.buffer;
  if (
    bridge.cachedSpeciesSchoolClusterView === null ||
    bridge.cachedSpeciesSchoolClusterPtr !== ptr ||
    bridge.cachedSpeciesSchoolClusterCount !== floatCount ||
    bridge.cachedHeapBuffer !== heapBuffer
  ) {
    bridge.cachedSpeciesSchoolClusterPtr = ptr;
    bridge.cachedSpeciesSchoolClusterCount = floatCount;
    bridge.cachedSpeciesSchoolClusterView = new Float32Array(heapBuffer, ptr, floatCount);
    bridge.cachedHeapBuffer = heapBuffer;
  }

  return {
    buffer: bridge.cachedSpeciesSchoolClusterView,
    count: Math.floor(floatCount / 6),
  };
}
