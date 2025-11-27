import { reactive, computed, toRaw } from 'vue';

const STORAGE_KEY = 'boids_settings';
const DEFAULT_SPECIES_FALLBACK = {
  species: 'Species',
  count: 0,
  cohesion: 20,
  cohesionRange: 30,
  separation: 5,
  separationRange: 1,
  alignment: 10,
  alignmentRange: 6,
  maxSpeed: 0.3,
  minSpeed: 0,
  maxTurnAngle: 0.2,
  maxNeighbors: 6,
  horizontalTorque: 0.02,
  torqueStrength: 5,
  lambda: 0.5,
  tau: 1.5,
  velocityEpsilon: 0.0001,
  predatorAlertRadius: 1,
  isPredator: false,
  speciesId: 0,
  centerAttractStrength: 0.0,
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

  replaceSettings(defaultTemplates);
  assignSystemSettings(systemDefaults);
  if (!loadFromStorage()) {
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
  result.horizontalTorque = toFinite(merged.horizontalTorque, base.horizontalTorque ?? 0);
  result.torqueStrength = toFinite(merged.torqueStrength, base.torqueStrength ?? 0);
  result.lambda = toFinite(merged.lambda, base.lambda ?? 0);
  result.tau = toFinite(merged.tau, base.tau ?? 0);
  result.velocityEpsilon = Math.max(0, toFinite(merged.velocityEpsilon, base.velocityEpsilon ?? 0.0001));
  result.predatorAlertRadius = Math.max(0, toFinite(merged.predatorAlertRadius, base.predatorAlertRadius ?? 1));
  result.centerAttractStrength = Math.max(0, toFinite(merged.centerAttractStrength, base.centerAttractStrength ?? 0));

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
