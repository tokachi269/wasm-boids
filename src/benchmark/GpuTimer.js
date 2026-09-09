export class GpuTimer {
  constructor() {
    this.gl = null;
    this.extension = null;
    this.initialized = false;
    this.active = null;
    this.pending = [];
  }

  get available() {
    return Boolean(this.extension);
  }

  get pendingCount() {
    return this.pending.length + (this.active ? 1 : 0);
  }

  initialize(gl) {
    if (this.initialized && this.gl === gl) return;
    this.dispose();
    this.gl = gl ?? null;
    this.extension = this.gl?.getExtension?.(
      'EXT_disjoint_timer_query_webgl2',
    ) ?? null;
    this.initialized = true;
  }

  begin(gl, onResult) {
    this.initialize(gl);
    this.poll();
    if (!this.extension || this.active) return false;
    const query = this.gl.createQuery();
    if (!query) return false;
    this.gl.beginQuery(this.extension.TIME_ELAPSED_EXT, query);
    this.active = { query, onResult };
    return true;
  }

  end() {
    if (!this.active || !this.extension) return;
    this.gl.endQuery(this.extension.TIME_ELAPSED_EXT);
    this.pending.push(this.active);
    this.active = null;
    this.poll();
  }

  poll() {
    if (!this.extension || this.pending.length === 0) return;
    const disjoint = this.gl.getParameter(this.extension.GPU_DISJOINT_EXT);
    if (disjoint) {
      for (const { query } of this.pending) this.gl.deleteQuery(query);
      this.pending = [];
      return;
    }

    const remaining = [];
    for (const item of this.pending) {
      const available = this.gl.getQueryParameter(
        item.query,
        this.gl.QUERY_RESULT_AVAILABLE,
      );
      if (!available) {
        remaining.push(item);
        continue;
      }
      const milliseconds =
        this.gl.getQueryParameter(item.query, this.gl.QUERY_RESULT) / 1e6;
      this.gl.deleteQuery(item.query);
      item.onResult?.(milliseconds);
    }
    this.pending = remaining;
  }

  dispose() {
    if (this.gl) {
      if (this.active) {
        try {
          this.gl.endQuery(this.extension.TIME_ELAPSED_EXT);
        } catch {
          // Context loss can make an active query impossible to close.
        }
        try {
          this.gl.deleteQuery?.(this.active.query);
        } catch {
          // The context may already be unavailable.
        }
      }
      for (const { query } of this.pending) {
        try {
          this.gl.deleteQuery?.(query);
        } catch {
          // The context may already be unavailable.
        }
      }
    }
    this.gl = null;
    this.extension = null;
    this.initialized = false;
    this.active = null;
    this.pending = [];
  }
}
