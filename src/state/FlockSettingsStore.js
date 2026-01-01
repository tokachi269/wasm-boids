import { reactive, computed, toRaw } from 'vue';

const STORAGE_KEY = 'boids_settings';
// 設定キャッシュ互換性（メジャーバージョン）
// - 重大な挙動変更（パラメータの意味変更/追加/削除）が入った場合に +1 する。
// - メジャーが一致しない（または未保存=旧ユーザ）場合は、localStorage の設定を使わず
//   デフォルト設定で上書きし、再アクセス時の「変な動き」を防ぐ。
// 2026-01-01: 調整項目（cohesionBoost / separationMinFactor / alignmentBoost）削除のため +1
const SETTINGS_SCHEMA_MAJOR_VERSION = 1;
const STORAGE_VERSION_KEY = 'boids_settings_schema_major';
const DEFAULT_SPECIES_FALLBACK = {
  species: 'Species',
  count: 10000,
  cohesion: 12.5,
  cohesionRange: 5,
  separation: 2.0,
  separationRange: 0.4,
  alignment: 7.0,
  alignmentRange: 1,
  maxSpeed: 0.33,
  minSpeed: 0,
  // maxTurnAngle は「1秒あたりの旋回速度(rad/sec)」。
  maxTurnAngle: 0.25,
  maxNeighbors: 4,
  horizontalTorque: 0.02,
  torqueStrength: 1.0,
  lambda: 0.8,
  tau: 0.8,
  velocityEpsilon: 0.0001,
  predatorAlertRadius: 1,
  isPredator: false,
  speciesId: 0,
  densityReturnStrength: 10.0,
};

/**
 * Flock 設定のロード/保存/編集を司る小さなストアヘルパー。
 * localStorage に保存された配列を復元し、Vue からはこのストア経由で操作します。
 */
export function createFlockSettingsStore(defaultSettings = [], defaultSystem = {}) {
  const defaultTemplates = sanitizeSpeciesList(defaultSettings, defaultSettings);
  const systemDefaults = { ...defaultSystem };
  const settings = reactive([]);
  const systemSettings = reactive({});

  // まずはデフォルトを適用する。
  replaceSettings(defaultTemplates);
  assignSystemSettings(systemDefaults);

  // 旧ユーザ（バージョン未保存）や、設定の互換性が壊れた場合は
  // キャッシュを読まずにデフォルトへ強制上書きする。
  if (shouldResetStorageBySchemaMajor()) {
    // NOTE:
    // 設定スキーマが壊れていても「個体数(count)」だけはユーザが意図して設定していることが多い。
    // 体感性能/見た目に直結するため、ここだけは旧保存値を拾って引き継ぐ。
    const previousCounts = tryLoadSpeciesCountsFromStorage();

    clearSettingsStorage();
    replaceSettings(defaultTemplates);
    assignSystemSettings(systemDefaults);
    applySpeciesCounts(settings, previousCounts);

    // saveToStorage() がスキーマバージョンも保存する。
    saveToStorage();
  } else if (!loadFromStorage()) {
    replaceSettings(defaultTemplates);
    assignSystemSettings(systemDefaults);
  }

  const totalBoids = computed(() => getTotalBoidCount(settings));

  function loadFromStorage() {
    if (typeof localStorage === 'undefined') {
      return false;
    }

    try {
      const saved = localStorage.getItem(STORAGE_KEY);
      if (!saved) {
        return false;
      }
      const parsed = JSON.parse(saved);
      if (Array.isArray(parsed) && parsed.length > 0) {
        replaceSettings(parsed);
        return true;
      }
      if (parsed && typeof parsed === 'object') {
        if (Array.isArray(parsed.species) && parsed.species.length > 0) {
          replaceSettings(parsed.species);
        }
        if (parsed.system && typeof parsed.system === 'object') {
          assignSystemSettings(parsed.system);
        }
        return true;
      }
    } catch (error) {
      console.warn('Failed to load settings from localStorage:', error);
    }
    return false;
  }

  /**
   * localStorage のスナップショットをパースして返す。
   * - 旧形式（配列）/新形式（{species, system}）の両方に対応
   */
  function readStorageSnapshot() {
    if (typeof localStorage === 'undefined') {
      return null;
    }

    try {
      const saved = localStorage.getItem(STORAGE_KEY);
      if (!saved) {
        return null;
      }
      const parsed = JSON.parse(saved);
      if (Array.isArray(parsed)) {
        return { species: parsed, system: null };
      }
      if (parsed && typeof parsed === 'object') {
        return {
          species: Array.isArray(parsed.species) ? parsed.species : null,
          system: parsed.system && typeof parsed.system === 'object' ? parsed.system : null,
        };
      }
    } catch (error) {
      console.warn('Failed to parse settings snapshot from localStorage:', error);
    }
    return null;
  }

  /**
   * 保存済み設定から「個体数(count)」だけを抜き出す。
   * - speciesId がある場合はそれを優先してキー化する（名前変更に強い）
   * - ない場合は species 名 + predator フラグで一致させる
   */
  function tryLoadSpeciesCountsFromStorage() {
    const snapshot = readStorageSnapshot();
    const list = snapshot?.species;
    if (!Array.isArray(list)) {
      return new Map();
    }

    const counts = new Map();
    list.forEach((entry) => {
      if (!entry || typeof entry !== 'object') {
        return;
      }
      const rawCount = Number(entry.count);
      if (!Number.isFinite(rawCount)) {
        return;
      }
      const count = Math.max(0, Math.floor(rawCount));
      const isPredator = Boolean(entry.isPredator);
      const species = typeof entry.species === 'string' ? entry.species : '';
      const speciesId = Number(entry.speciesId);

      if (Number.isFinite(speciesId)) {
        counts.set(`id:${speciesId}|pred:${isPredator}`, count);
      }
      if (species) {
        counts.set(`name:${species}|pred:${isPredator}`, count);
      }
    });
    return counts;
  }

  /**
   * settings 配列（リアクティブ）へ個体数を反映する。
   * - 既存キーがない場合は何もしない（デフォルトを維持）
   */
  function applySpeciesCounts(targetSettings, countsMap) {
    if (!Array.isArray(targetSettings) || !(countsMap instanceof Map)) {
      return;
    }

    targetSettings.forEach((entry) => {
      if (!entry || typeof entry !== 'object') {
        return;
      }
      const isPredator = Boolean(entry.isPredator);
      const speciesId = Number(entry.speciesId);
      const species = typeof entry.species === 'string' ? entry.species : '';

      let nextCount;
      if (Number.isFinite(speciesId)) {
        nextCount = countsMap.get(`id:${speciesId}|pred:${isPredator}`);
      }
      if (!Number.isFinite(nextCount) && species) {
        nextCount = countsMap.get(`name:${species}|pred:${isPredator}`);
      }
      if (Number.isFinite(nextCount)) {
        entry.count = Math.max(0, Math.floor(nextCount));
      }
    });
  }

  /**
   * localStorage の設定を破棄する（スキーマメジャーが変わった時に使用）。
   * NOTE: バージョンキー自体は saveToStorage() で再度保存する。
   */
  function clearSettingsStorage() {
    if (typeof localStorage === 'undefined') {
      return false;
    }
    try {
      localStorage.removeItem(STORAGE_KEY);
      return true;
    } catch (error) {
      console.warn('Failed to clear settings in localStorage:', error);
      return false;
    }
  }

  /**
   * スキーマメジャーが一致しない（または未保存）場合に true。
   * - 未保存は「今回の対応より前にアクセスしたユーザ」と判断してリセット対象にする。
   */
  function shouldResetStorageBySchemaMajor() {
    if (typeof localStorage === 'undefined') {
      return false;
    }

    try {
      const raw = localStorage.getItem(STORAGE_VERSION_KEY);
      if (!raw) {
        // 旧ユーザ（バージョン未保存）
        return true;
      }

      const parsed = Number(raw);
      if (!Number.isFinite(parsed)) {
        return true;
      }

      return parsed !== SETTINGS_SCHEMA_MAJOR_VERSION;
    } catch (error) {
      // 取得に失敗した場合も安全側（リセット）
      console.warn('Failed to read settings schema version from localStorage:', error);
      return true;
    }
  }

  function saveToStorage() {
    if (typeof localStorage === 'undefined') {
      return false;
    }

    try {
      const snapshot = {
        species: settings.map((entry) => ({ ...toRaw(entry) })),
        system: { ...toRaw(systemSettings) },
      };
      localStorage.setItem(STORAGE_KEY, JSON.stringify(snapshot));
      // 互換性の目印（メジャー）。読み込み側で不一致ならデフォルトへ強制。
      localStorage.setItem(STORAGE_VERSION_KEY, String(SETTINGS_SCHEMA_MAJOR_VERSION));
      return true;
    } catch (error) {
      console.warn('Failed to save settings to localStorage:', error);
      return false;
    }
  }

  function replaceSettings(nextList) {
    const sanitized = sanitizeSpeciesList(nextList, defaultTemplates);
    settings.splice(0, settings.length, ...sanitized);
    return sanitized;
  }

  function assignSystemSettings(patch = {}) {
    const merged = { ...systemDefaults, ...(patch || {}) };
    Object.keys(systemSettings).forEach((key) => {
      if (!(key in merged)) {
        delete systemSettings[key];
      }
    });
    Object.entries(merged).forEach(([key, value]) => {
      systemSettings[key] = value;
    });
    return systemSettings;
  }

  function resetToDefaults() {
    replaceSettings(defaultTemplates);
    assignSystemSettings(systemDefaults);
    saveToStorage();
  }

  function addSpecies(template) {
    const basePool = settings.length > 0 ? settings : defaultTemplates;
    const fallbackTemplate =
      basePool.find((item) => item && !item.isPredator) ||
      defaultTemplates.find((item) => item && !item.isPredator) ||
      DEFAULT_SPECIES_FALLBACK;

    const candidate = template
      ? sanitizeSpeciesEntry(template, fallbackTemplate)
      : cloneSpeciesTemplate(fallbackTemplate);

    candidate.isPredator = false;
    candidate.count = 0;
    candidate.species = ensureUniqueName(candidate.species, new Set(settings.map((s) => s.species)));
    settings.push(candidate);
    return candidate;
  }

  function removeSpecies(index) {
    if (index < 0 || index >= settings.length) {
      return null;
    }
    const removed = settings.splice(index, 1);
    return removed.length ? removed[0] : null;
  }

  return {
    settings,
    systemSettings,
    totalBoids,
    loadFromStorage,
    saveToStorage,
    replaceSettings,
    assignSystemSettings,
    resetToDefaults,
    addSpecies,
    removeSpecies,
  };
}

function getTotalBoidCount(list) {
  return list.reduce((sum, entry) => sum + Math.max(0, Number(entry?.count) || 0), 0);
}

function sanitizeSpeciesList(list = [], fallbackList = []) {
  if (!Array.isArray(list)) {
    return [];
  }

  const sanitized = [];
  const takenNames = new Set();

  list.forEach((entry, index) => {
    const fallback = fallbackList[index] || fallbackList[0] || DEFAULT_SPECIES_FALLBACK;
    const clean = sanitizeSpeciesEntry(entry, fallback);
    clean.species = ensureUniqueName(clean.species, takenNames);
    sanitized.push(clean);
  });

  return sanitized;
}

function sanitizeSpeciesEntry(entry = {}, fallback = DEFAULT_SPECIES_FALLBACK) {
  const base = { ...DEFAULT_SPECIES_FALLBACK, ...fallback };
  const merged = { ...base, ...entry };

  const toFinite = (value, fallbackValue = 0) => {
    const num = Number(value);
    return Number.isFinite(num) ? num : fallbackValue;
  };

  const result = { ...merged };
  result.count = Math.max(0, Math.floor(toFinite(merged.count, base.count ?? 0)));
  result.maxNeighbors = Math.max(0, Math.floor(toFinite(merged.maxNeighbors, base.maxNeighbors ?? 0)));
  result.speciesId = Math.max(0, Math.floor(toFinite(merged.speciesId, base.speciesId ?? 0)));

  result.cohesion = toFinite(merged.cohesion, base.cohesion ?? 0);
  result.cohesionRange = Math.max(0, toFinite(merged.cohesionRange, base.cohesionRange ?? 0));
  result.separation = toFinite(merged.separation, base.separation ?? 0);
  result.separationRange = Math.max(0, toFinite(merged.separationRange, base.separationRange ?? 0));
  result.alignment = toFinite(merged.alignment, base.alignment ?? 0);
  result.alignmentRange = Math.max(0, toFinite(merged.alignmentRange, base.alignmentRange ?? 0));
  result.maxSpeed = Math.max(0, toFinite(merged.maxSpeed, base.maxSpeed ?? 0));
  result.minSpeed = Math.max(0, toFinite(merged.minSpeed, base.minSpeed ?? 0));
  result.maxTurnAngle = toFinite(merged.maxTurnAngle, base.maxTurnAngle ?? 0);
  // 旧設定の互換: 以前は maxTurnAngle が「1ステップの最大旋回角(rad/step)」だった。
  // 保存済みの値が旧レンジ(<=1.0)の場合は、おおよそ60fps相当として rad/sec に換算する。
  if (result.maxTurnAngle > 0 && result.maxTurnAngle <= 1.0 && (base.maxTurnAngle ?? 0) > 1.0) {
    result.maxTurnAngle *= 60;
  }
  result.horizontalTorque = toFinite(merged.horizontalTorque, base.horizontalTorque ?? 0);
  result.torqueStrength = toFinite(merged.torqueStrength, base.torqueStrength ?? 0);
  result.lambda = toFinite(merged.lambda, base.lambda ?? 0);
  result.tau = toFinite(merged.tau, base.tau ?? 0);
  result.velocityEpsilon = Math.max(0, toFinite(merged.velocityEpsilon, base.velocityEpsilon ?? 0.0001));
  result.predatorAlertRadius = Math.max(0, toFinite(merged.predatorAlertRadius, base.predatorAlertRadius ?? 1));
  const baseReturnStrength = base.densityReturnStrength ?? base.centerAttractStrength ?? 0;
  const mergedReturnStrength =
    merged.densityReturnStrength ?? merged.centerAttractStrength ?? baseReturnStrength;
  result.densityReturnStrength = Math.max(0, toFinite(mergedReturnStrength, baseReturnStrength));

  const speciesName = typeof merged.species === 'string' ? merged.species.trim() : '';
  result.species = speciesName || base.species || DEFAULT_SPECIES_FALLBACK.species;
  result.isPredator = Boolean(merged.isPredator);

  return result;
}

function ensureUniqueName(name, taken, defaultName = 'Species') {
  const base = typeof name === 'string' && name.trim() ? name.trim() : defaultName;
  let candidate = base;
  let index = 2;
  while (taken.has(candidate)) {
    candidate = `${base} ${index}`;
    index += 1;
  }
  taken.add(candidate);
  return candidate;
}

function cloneSpeciesTemplate(template) {
  return JSON.parse(JSON.stringify(template));
}
