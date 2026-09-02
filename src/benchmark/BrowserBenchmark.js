const STAGE_NAMES = [
  'frame_total',
  'wasm_simulation',
  'wasm_to_js_views',
  'js_instance_packing',
  'render_submission_cpu',
];

function integerParam(params, name, fallback, minimum = 0) {
  const value = Number(params.get(name));
  return Number.isFinite(value) ? Math.max(minimum, Math.floor(value)) : fallback;
}

export function readBrowserBenchmarkConfig(search = '') {
  const params = new URLSearchParams(search);
  const enabled = params.get('bench') === '1';
  if (!enabled) return null;
  const warmup = integerParam(params, 'warmup', 300, 0);
  const samples = integerParam(params, 'samples', 1000, 1);
  return {
    enabled,
    warmup,
    samples,
    totalFrames: warmup + samples,
    seed: integerParam(params, 'seed', 1, 0) >>> 0,
    boids: integerParam(params, 'boids', 5000, 3),
    taskLimit: integerParam(params, 'tasks', 1, 0),
  };
}

function summarize(values) {
  if (!values.length) {
    return { mean: 0, median: 0, p95: 0, p99: 0, max: 0 };
  }
  const sorted = [...values].sort((a, b) => a - b);
  const percentile = (fraction) =>
    sorted[Math.max(0, Math.ceil(fraction * sorted.length) - 1)];
  return {
    mean: sorted.reduce((sum, value) => sum + value, 0) / sorted.length,
    median: percentile(0.5),
    p95: percentile(0.95),
    p99: percentile(0.99),
    max: sorted[sorted.length - 1],
  };
}

function checksumPositions(positions) {
  const bytes = new Uint8Array(
    positions.buffer,
    positions.byteOffset,
    positions.byteLength,
  );
  let hash = 0xcbf29ce484222325n;
  const prime = 0x100000001b3n;
  const mask = 0xffffffffffffffffn;
  for (let i = 0; i < bytes.length; i += 1) {
    hash ^= BigInt(bytes[i]);
    hash = (hash * prime) & mask;
  }
  return hash.toString(16).padStart(16, '0');
}

export class BrowserBenchmark {
  constructor(config) {
    this.config = config;
    this.frameIndex = 0;
    this.frameStart = 0;
    this.samples = Object.fromEntries(STAGE_NAMES.map((name) => [name, []]));
    this.gpuSamples = [];
    this.gpu = null;
    this.activeGpuQuery = null;
    this.pendingGpuQueries = [];
    this.finished = false;
  }

  get measuring() {
    return this.frameIndex >= this.config.warmup;
  }

  beginFrame() {
    this.frameStart = performance.now();
    return this.frameIndex;
  }

  shouldResetPhaseTimings() {
    return this.frameIndex === this.config.warmup;
  }

  measure(name, callback) {
    const start = performance.now();
    const result = callback();
    if (this.measuring) {
      this.samples[name].push(performance.now() - start);
    }
    return result;
  }

  initializeGpu(gl) {
    if (this.gpu || !gl) return;
    const extension = gl.getExtension('EXT_disjoint_timer_query_webgl2');
    this.gpu = { gl, extension };
  }

  beginGpu(gl) {
    this.initializeGpu(gl);
    if (!this.measuring || !this.gpu?.extension || this.activeGpuQuery) return;
    const query = this.gpu.gl.createQuery();
    if (!query) return;
    this.gpu.gl.beginQuery(this.gpu.extension.TIME_ELAPSED_EXT, query);
    this.activeGpuQuery = query;
  }

  endGpu() {
    if (!this.activeGpuQuery || !this.gpu?.extension) return;
    this.gpu.gl.endQuery(this.gpu.extension.TIME_ELAPSED_EXT);
    this.pendingGpuQueries.push(this.activeGpuQuery);
    this.activeGpuQuery = null;
    this.pollGpuQueries();
  }

  pollGpuQueries() {
    if (!this.gpu?.extension) return;
    const { gl, extension } = this.gpu;
    const disjoint = gl.getParameter(extension.GPU_DISJOINT_EXT);
    const remaining = [];
    for (const query of this.pendingGpuQueries) {
      const available = gl.getQueryParameter(query, gl.QUERY_RESULT_AVAILABLE);
      if (!available) {
        remaining.push(query);
        continue;
      }
      if (!disjoint) {
        this.gpuSamples.push(gl.getQueryParameter(query, gl.QUERY_RESULT) / 1e6);
      }
      gl.deleteQuery(query);
    }
    this.pendingGpuQueries = remaining;
  }

  endFrame() {
    if (this.measuring) {
      this.samples.frame_total.push(performance.now() - this.frameStart);
    }
    this.frameIndex += 1;
    return this.frameIndex >= this.config.totalFrames;
  }

  async finish({ bridge, positions, renderer }) {
    if (this.finished) return window.__boidsBenchmarkResult;
    this.finished = true;
    const deadline = performance.now() + 5000;
    while (this.pendingGpuQueries.length > 0 && performance.now() < deadline) {
      await new Promise((resolve) => setTimeout(resolve, 16));
      this.pollGpuQueries();
    }

    const gl = renderer?.getContext?.() ?? null;
    const debugInfo = gl?.getExtension?.('WEBGL_debug_renderer_info') ?? null;
    const result = {
      build: 'Release',
      frames: this.config.samples,
      warmup: this.config.warmup,
      seed: this.config.seed,
      boids: this.config.boids,
      task_limit: this.config.taskLimit,
      timing_ms: Object.fromEntries(
        STAGE_NAMES.map((name) => [name, summarize(this.samples[name])]),
      ),
      phases: bridge.getPhaseTimings(),
      parallel: bridge.getParallelTimings(),
      locality: bridge.getLocalityStats(),
      gpu_ms: {
        available: Boolean(this.gpu?.extension),
        samples: this.gpuSamples.length,
        ...summarize(this.gpuSamples),
      },
      checksum: checksumPositions(positions),
      environment: {
        user_agent: navigator.userAgent,
        hardware_concurrency: navigator.hardwareConcurrency ?? null,
        webgl_renderer:
          gl && debugInfo ? gl.getParameter(debugInfo.UNMASKED_RENDERER_WEBGL) : null,
        draw_calls: renderer?.info?.render?.calls ?? null,
        triangles: renderer?.info?.render?.triangles ?? null,
      },
    };
    window.__boidsBenchmarkResult = result;
    const line = JSON.stringify(result);
    console.log(`BOIDS_BENCHMARK ${line}`);
    const output = document.createElement('pre');
    output.id = 'benchmark-result';
    output.textContent = line;
    output.style.cssText =
      'position:fixed;inset:8px;z-index:99999;overflow:auto;padding:12px;background:#111;color:#eee;font:12px monospace;white-space:pre-wrap;';
    document.body.appendChild(output);
    return result;
  }
}
