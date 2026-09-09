import { GpuTimer } from './GpuTimer.js';

const DEFAULT_SAMPLE_COUNT = 1800;
const TEXT_UPDATE_MS = 250;
const GRAPH_UPDATE_MS = 1000 / 30;

function mean(values) {
  if (!values.length) return 0;
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

function percentile(values, fraction) {
  if (!values.length) return 0;
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.max(0, Math.ceil(sorted.length * fraction) - 1)];
}

export class LivePerformanceMonitor {
  constructor(sampleCount = DEFAULT_SAMPLE_COUNT) {
    this.sampleCount = sampleCount;
    this.frame = new Float32Array(sampleCount);
    this.simulation = new Float32Array(sampleCount);
    this.javascript = new Float32Array(sampleCount);
    this.cpu = new Float32Array(sampleCount);
    this.gpu = new Float32Array(sampleCount);
    this.sequence = new Uint32Array(sampleCount);
    this.gpu.fill(Number.NaN);
    this.gpuTimer = new GpuTimer();
    this.enabled = false;
    this.lastFrameTime = null;
    this.frameStart = 0;
    this.currentSlot = -1;
    this.currentSequence = 0;
    this.currentSimulation = 0;
    this.currentJavascript = 0;
    this.writeIndex = 0;
    this.size = 0;
    this.nextSequence = 1;
    this.lastTextUpdate = 0;
    this.lastGraphUpdate = 0;
    this.peakFrameMs = 0;
  }

  setEnabled(enabled) {
    const next = Boolean(enabled);
    if (next === this.enabled) return;
    this.enabled = next;
    this.reset();
    if (!next) this.gpuTimer.dispose();
  }

  reset() {
    this.lastFrameTime = null;
    this.currentSlot = -1;
    this.currentSequence = 0;
    this.currentSimulation = 0;
    this.currentJavascript = 0;
    this.writeIndex = 0;
    this.size = 0;
    this.nextSequence = 1;
    this.sequence.fill(0);
    this.gpu.fill(Number.NaN);
    this.lastTextUpdate = 0;
    this.lastGraphUpdate = 0;
    this.peakFrameMs = 0;
  }

  beginFrame(frameTime = performance.now()) {
    if (!this.enabled) return;
    this.gpuTimer.poll();
    this.currentSimulation = 0;
    this.currentJavascript = 0;
    this.currentSlot = -1;
    this.frameStart = performance.now();

    if (this.lastFrameTime !== null) {
      const interval = frameTime - this.lastFrameTime;
      if (interval > 0 && interval < 1000) {
        const slot = this.writeIndex;
        const sequence = this.nextSequence++;
        this.sequence[slot] = sequence;
        this.frame[slot] = interval;
        this.peakFrameMs = Math.max(this.peakFrameMs, interval);
        this.simulation[slot] = 0;
        this.javascript[slot] = 0;
        this.cpu[slot] = 0;
        this.gpu[slot] = Number.NaN;
        this.currentSlot = slot;
        this.currentSequence = sequence;
        this.writeIndex = (slot + 1) % this.sampleCount;
        this.size = Math.min(this.size + 1, this.sampleCount);
      }
    }
    this.lastFrameTime = frameTime;
  }

  measureSimulation(callback) {
    return this.measure('simulation', callback);
  }

  measureJavascript(callback) {
    return this.measure('javascript', callback);
  }

  measure(name, callback) {
    if (!this.enabled || this.currentSlot < 0) return callback();
    const start = performance.now();
    const result = callback();
    const elapsed = performance.now() - start;
    if (name === 'simulation') this.currentSimulation += elapsed;
    if (name === 'javascript') this.currentJavascript += elapsed;
    return result;
  }

  beginGpu(gl) {
    if (!this.enabled || this.currentSlot < 0) return;
    const slot = this.currentSlot;
    const sequence = this.currentSequence;
    this.gpuTimer.begin(gl, (milliseconds) => {
      if (this.sequence[slot] === sequence) this.gpu[slot] = milliseconds;
    });
  }

  endGpu() {
    if (this.enabled) this.gpuTimer.end();
  }

  endFrame(canvas) {
    if (!this.enabled || this.currentSlot < 0) return null;
    const now = performance.now();
    const slot = this.currentSlot;
    if (this.sequence[slot] === this.currentSequence) {
      this.simulation[slot] = this.currentSimulation;
      this.javascript[slot] = this.currentJavascript;
      this.cpu[slot] = now - this.frameStart;
    }
    this.gpuTimer.poll();

    if (now - this.lastGraphUpdate >= GRAPH_UPDATE_MS) {
      this.draw(canvas);
      this.lastGraphUpdate = now;
    }
    if (now - this.lastTextUpdate < TEXT_UPDATE_MS) return null;
    this.lastTextUpdate = now;
    return this.snapshot();
  }

  orderedIndices() {
    const result = [];
    const start = (this.writeIndex - this.size + this.sampleCount) % this.sampleCount;
    for (let i = 0; i < this.size; i += 1) {
      result.push((start + i) % this.sampleCount);
    }
    return result;
  }

  recentIndices(milliseconds = 1000) {
    const ordered = this.orderedIndices();
    const result = [];
    let elapsed = 0;
    for (let i = ordered.length - 1; i >= 0; i -= 1) {
      const index = ordered[i];
      result.push(index);
      elapsed += this.frame[index];
      if (elapsed >= milliseconds) break;
    }
    return result;
  }

  snapshot() {
    const indices = this.recentIndices();
    const frameValues = indices.map((index) => this.frame[index]);
    const gpuValues = indices
      .map((index) => this.gpu[index])
      .filter(Number.isFinite);
    const totalInterval = frameValues.reduce((sum, value) => sum + value, 0);
    return {
      fps: totalInterval > 0 ? (frameValues.length * 1000) / totalInterval : 0,
      frameMs: mean(frameValues),
      p95Ms: percentile(frameValues, 0.95),
      peakFrameMs: this.peakFrameMs,
      simulationMs: mean(indices.map((index) => this.simulation[index])),
      javascriptMs: mean(indices.map((index) => this.javascript[index])),
      cpuMs: mean(indices.map((index) => this.cpu[index])),
      gpuMs: gpuValues.length ? mean(gpuValues) : null,
      gpuAvailable: this.gpuTimer.available,
    };
  }

  draw(canvas) {
    if (!canvas || this.size < 2) return;
    const width = Math.max(1, Math.round(canvas.clientWidth));
    const height = Math.max(1, Math.round(canvas.clientHeight));
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const pixelWidth = Math.round(width * dpr);
    const pixelHeight = Math.round(height * dpr);
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    const context = canvas.getContext('2d');
    if (!context) return;
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.clearRect(0, 0, width, height);

    const indices = this.orderedIndices();
    const frameValues = indices.map((index) => this.frame[index]);
    const minFrame = Math.min(...frameValues);
    const maxFrame = Math.max(...frameValues);
    const observedRange = Math.max(0.5, maxFrame - minFrame);
    const frameMargin = Math.max(0.35, observedRange * 0.12);
    let frameMinimum = Math.max(0, minFrame - frameMargin);
    let frameMaximum = maxFrame + frameMargin;
    frameMinimum = Math.min(frameMinimum, 16.67);
    frameMaximum = Math.max(frameMaximum, 16.67);
    if (frameMaximum - frameMinimum < 2) {
      const center = (frameMinimum + frameMaximum) * 0.5;
      frameMinimum = Math.max(0, center - 1);
      frameMaximum = center + 1;
    }
    const graphLeft = 2;
    const graphRight = width - 2;
    const graphTop = 4;
    const graphBottom = Math.round(height * 0.6);
    const xFor = (order) =>
      graphLeft + (order / Math.max(1, this.sampleCount - 1)) * (graphRight - graphLeft);
    const yFor = (value, top, bottom, minimum, maximum) =>
      bottom -
      (Math.min(Math.max(value, minimum), maximum) - minimum) /
        (maximum - minimum) *
        (bottom - top);

    context.font = '9px system-ui, sans-serif';
    context.textAlign = 'right';
    for (const threshold of [16.67, 33.33]) {
      if (threshold < frameMinimum || threshold > frameMaximum) continue;
      const y = yFor(
        threshold,
        graphTop,
        graphBottom,
        frameMinimum,
        frameMaximum,
      );
      context.setLineDash([2, 4]);
      context.strokeStyle = 'rgba(205, 236, 248, 0.22)';
      context.beginPath();
      context.moveTo(graphLeft, y);
      context.lineTo(graphRight, y);
      context.stroke();
      context.fillStyle = 'rgba(220, 243, 250, 0.48)';
      context.fillText(threshold.toFixed(1), graphRight, y - 2);
    }
    context.setLineDash([]);

    const drawSeries = (values, top, bottom, minimum, maximum, color) => {
      context.strokeStyle = color;
      context.lineWidth = 1.25;
      context.beginPath();
      let drawing = false;
      const offset = this.sampleCount - indices.length;
      indices.forEach((index, order) => {
        const value = values[index];
        if (!Number.isFinite(value)) {
          drawing = false;
          return;
        }
        const x = xFor(offset + order);
        const y = yFor(value, top, bottom, minimum, maximum);
        if (drawing) context.lineTo(x, y);
        else context.moveTo(x, y);
        drawing = true;
      });
      context.stroke();
    };

    drawSeries(
      this.frame,
      graphTop,
      graphBottom,
      frameMinimum,
      frameMaximum,
      'rgba(225, 247, 255, 0.88)',
    );

    const sparkTop = graphBottom + 10;
    const sparkMiddle = Math.round((sparkTop + height - 3) / 2);
    const sparkScale = Math.max(
      16.67,
      ...indices.map((index) => this.simulation[index]),
      ...indices.map((index) => Number.isFinite(this.gpu[index]) ? this.gpu[index] : 0),
    );
    drawSeries(this.simulation, sparkTop, sparkMiddle - 2, 0, sparkScale, 'rgba(111, 219, 242, 0.72)');
    drawSeries(this.gpu, sparkMiddle + 2, height - 3, 0, sparkScale, 'rgba(179, 215, 237, 0.68)');
  }

  dispose() {
    this.enabled = false;
    this.gpuTimer.dispose();
    this.reset();
  }
}
