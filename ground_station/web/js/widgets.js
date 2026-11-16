// Canvas 2D instruments for the dashboard (no chart library):
// ArcGauge, TimeChart (crosshair + tooltip + keyboard), AttitudeIndicator, TrackPlot.
// Colours come from CSS custom properties (css/gs.css) so the palette lives in one place.

const cssCache = new Map();
/** Read a CSS custom property from :root (cached). */
export function css(name) {
  if (!cssCache.has(name)) {
    cssCache.set(name, getComputedStyle(document.documentElement).getPropertyValue(name).trim() || '#888');
  }
  return cssCache.get(name);
}

const FONT = '12px system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';
const FONT_SMALL = '11px system-ui, -apple-system, "Segoe UI", Roboto, sans-serif';

/** Size the backing store to the CSS box x devicePixelRatio; returns {ctx, w, h}. */
export function fitCanvas(canvas) {
  const dpr = Math.max(1, Math.min(3, window.devicePixelRatio || 1));
  const rect = canvas.getBoundingClientRect();
  const w = Math.max(10, Math.round(rect.width));
  const h = Math.max(10, Math.round(rect.height));
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

/** Redraw `widget` whenever its canvas changes size. */
function observe(widget, el) {
  if (typeof ResizeObserver === 'undefined') return;
  const ro = new ResizeObserver(() => widget.draw());
  ro.observe(el);
}

/** "nice" tick step for a span (1, 2, 2.5, 5 x 10^n) */
export function niceStep(span, target = 4) {
  const raw = span / Math.max(1, target);
  const mag = 10 ** Math.floor(Math.log10(raw));
  for (const m of [1, 2, 2.5, 5, 10]) if (raw <= m * mag) return m * mag;
  return 10 * mag;
}

const TIME_STEPS = [1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600, 7200, 10800, 21600, 43200];
/** time-axis step [s] giving at most maxTicks intervals over spanS */
export function timeStep(spanS, maxTicks) {
  for (const s of TIME_STEPS) if (spanS / s <= maxTicks) return s;
  return TIME_STEPS[TIME_STEPS.length - 1];
}
function agoLabel(s) {
  if (s < 60) return `−${s} s`;
  if (s < 3600) return `−${Number.isInteger(s / 60) ? s / 60 : (s / 60).toFixed(1)} min`;
  return `−${Number.isInteger(s / 3600) ? s / 3600 : (s / 3600).toFixed(1)} h`;
}

function fmtNum(v, decimals) {
  if (v == null || !Number.isFinite(v)) return '—';
  return v.toFixed(decimals);
}

// ================================================================ ArcGauge
/** 240° arc meter: thin track, value arc in one hue, threshold tick, min/max labels.
 *  The value itself is HTML text next to the canvas (accessible, crisp). */
export class ArcGauge {
  constructor(canvas, { min, max, threshold = null, decimals = 1 }) {
    this.canvas = canvas;
    this.min = min; this.max = max; this.threshold = threshold; this.decimals = decimals;
    this.value = null;
    this.status = 'normal';
    observe(this, canvas);
  }

  set(value, status = 'normal') {
    this.value = value;
    this.status = status;
    this.draw();
  }

  draw() {
    const { ctx, w, h } = fitCanvas(this.canvas);
    ctx.clearRect(0, 0, w, h);
    if (w < 40 || h < 30) return; // not laid out (hidden / collapsed)
    const r = Math.min(w / 2 - 8, h * 0.62);
    const cx = w / 2; const cy = r + 8;
    const a0 = Math.PI * (5 / 6); const a1 = Math.PI * (13 / 6); // 150° .. 390°
    const lw = Math.max(6, r * 0.13);
    ctx.lineCap = 'round';
    ctx.lineWidth = lw;
    ctx.strokeStyle = css('--track');
    ctx.beginPath(); ctx.arc(cx, cy, r, a0, a1); ctx.stroke();
    const frac = (v) => Math.max(0, Math.min(1, (v - this.min) / (this.max - this.min)));
    if (this.value != null && Number.isFinite(this.value)) {
      const a = a0 + (a1 - a0) * frac(this.value);
      ctx.strokeStyle = this.status === 'critical' ? css('--critical')
        : this.status === 'warning' ? css('--warning') : css('--series-1');
      ctx.beginPath(); ctx.arc(cx, cy, r, a0, Math.max(a0 + 0.001, a)); ctx.stroke();
    }
    if (this.threshold != null) {
      const a = a0 + (a1 - a0) * frac(this.threshold);
      ctx.lineCap = 'butt';
      ctx.lineWidth = 2;
      ctx.strokeStyle = css('--critical');
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(a) * (r - lw), cy + Math.sin(a) * (r - lw));
      ctx.lineTo(cx + Math.cos(a) * (r + lw), cy + Math.sin(a) * (r + lw));
      ctx.stroke();
    }
    ctx.fillStyle = css('--muted');
    ctx.font = FONT_SMALL;
    ctx.textBaseline = 'top';
    const ly = cy + Math.sin(a0) * r + lw / 2 + 3;
    ctx.textAlign = 'center';
    ctx.fillText(String(this.min), cx + Math.cos(a0) * r, ly);
    ctx.fillText(String(this.max), cx + Math.cos(a1) * r, ly);
  }
}

// =============================================================== TimeChart
/**
 * Line chart of telemetry history. One y-axis (never dual-axis), 2 px lines,
 * hairline grid, end dot + selective end label, crosshair + tooltip on hover,
 * keyboard (← → Home End Esc) when focused.
 */
export class TimeChart {
  constructor(canvas, { series, decimals = 1, unit = '', minSpan = 1, yMin = null, yMax = null, tip = null }) {
    this.canvas = canvas;
    this.series = series; // [{key, label, color}]
    this.decimals = decimals;
    this.unit = unit;
    this.minSpan = minSpan;
    this.yMin = yMin; this.yMax = yMax;
    this.tip = tip;       // absolutely positioned tooltip element
    this.data = [];
    this.windowS = 300;
    this.hover = null;    // index into this.view
    this.view = [];
    canvas.tabIndex = 0;
    canvas.addEventListener('mousemove', (e) => this._onMove(e));
    canvas.addEventListener('mouseleave', () => { this.hover = null; this.draw(); });
    canvas.addEventListener('keydown', (e) => this._onKey(e));
    canvas.addEventListener('blur', () => { this.hover = null; this.draw(); });
    observe(this, canvas);
  }

  setData(samples) { this.data = samples; this.draw(); }
  setWindow(s) { this.windowS = s; this.hover = null; this.draw(); }

  _layout(w, h) { return { l: 44, r: 52, t: 8, b: 20, pw: w - 96, ph: h - 28 }; }

  _onMove(e) {
    if (!this.view.length) return;
    const rect = this.canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const { l, pw } = this._layout(rect.width, rect.height);
    const t = this.t0 + ((x - l) / pw) * (this.t1 - this.t0);
    let best = 0; let bd = Infinity;
    this.view.forEach((s, i) => { const d = Math.abs(s.t - t); if (d < bd) { bd = d; best = i; } });
    this.hover = best;
    this.draw();
  }

  _onKey(e) {
    if (!this.view.length) return;
    const n = this.view.length;
    let i = this.hover == null ? n - 1 : this.hover;
    if (e.key === 'ArrowLeft') i = Math.max(0, i - 1);
    else if (e.key === 'ArrowRight') i = Math.min(n - 1, i + 1);
    else if (e.key === 'Home') i = 0;
    else if (e.key === 'End') i = n - 1;
    else if (e.key === 'Escape') { this.hover = null; this.draw(); return; } else return;
    e.preventDefault();
    e.stopPropagation();
    this.hover = i;
    this.draw();
  }

  draw() {
    const { ctx, w, h } = fitCanvas(this.canvas);
    ctx.clearRect(0, 0, w, h);
    if (w < 120 || h < 50) return; // not laid out (hidden / collapsed)
    const { l, t, pw, ph } = this._layout(w, h);
    const now = this.data.length ? this.data[this.data.length - 1].t : Date.now();
    const tStart = this.windowS ? now - this.windowS * 1000 : (this.data.length ? this.data[0].t : now - 60000);
    this.view = this.data.filter((s) => s.t >= tStart && this.series.some((sr) => s[sr.key] != null));
    this.t0 = this.windowS ? tStart : Math.min(tStart, now - 10000);
    this.t1 = Math.max(now, this.t0 + 1000);
    // y range from the visible data
    let lo = Infinity; let hi = -Infinity;
    for (const s of this.view) {
      for (const sr of this.series) {
        const v = s[sr.key];
        if (v != null && Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
      }
    }
    if (!Number.isFinite(lo)) { lo = this.yMin ?? 0; hi = this.yMax ?? 1; }
    if (hi - lo < this.minSpan) { const m = (hi + lo) / 2; lo = m - this.minSpan / 2; hi = m + this.minSpan / 2; }
    const pad = (hi - lo) * 0.08;
    lo -= pad; hi += pad;
    if (this.yMin != null) lo = Math.max(lo, this.yMin);
    if (this.yMax != null) hi = Math.min(hi, this.yMax);
    const step = niceStep(hi - lo, 3);
    lo = Math.floor(lo / step) * step;
    hi = Math.ceil(hi / step) * step;
    const X = (tt) => l + ((tt - this.t0) / (this.t1 - this.t0)) * pw;
    const Y = (v) => t + ph - ((v - lo) / (hi - lo)) * ph;
    // grid + y ticks
    ctx.font = FONT_SMALL;
    ctx.lineWidth = 1;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    const tickDec = step < 0.1 ? 2 : step < 1 ? 1 : 0;
    for (let v = lo; v <= hi + step / 2; v += step) {
      const y = Math.round(Y(v)) + 0.5;
      ctx.strokeStyle = css('--grid');
      ctx.beginPath(); ctx.moveTo(l, y); ctx.lineTo(l + pw, y); ctx.stroke();
      ctx.fillStyle = css('--muted');
      ctx.fillText(v.toFixed(tickDec), l - 6, y);
    }
    // x ticks (relative time)
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const spanS = (this.t1 - this.t0) / 1000;
    const xs = timeStep(spanS, Math.max(2, Math.floor(pw / 72)));
    for (let k = 0; k * xs <= spanS + 1e-6; k++) {
      const tt = this.t1 - k * xs * 1000;
      const x = X(tt);
      if (x < l + 12) break;
      ctx.fillStyle = css('--muted');
      ctx.fillText(k === 0 ? 'now' : agoLabel(k * xs), Math.min(x, l + pw - 12), t + ph + 5);
    }
    ctx.strokeStyle = css('--axis');
    ctx.beginPath(); ctx.moveTo(l, Math.round(t + ph) + 0.5); ctx.lineTo(l + pw, Math.round(t + ph) + 0.5); ctx.stroke();
    if (!this.view.length) {
      ctx.fillStyle = css('--muted');
      ctx.font = FONT;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('waiting for telemetry', l + pw / 2, t + ph / 2);
      if (this.tip) this.tip.hidden = true;
      return;
    }
    // lines (break across gaps > 3x the median sample interval)
    const dts = [];
    for (let i = 1; i < this.view.length; i++) dts.push(this.view[i].t - this.view[i - 1].t);
    dts.sort((a, b) => a - b);
    const gapMs = Math.max(2500, 3 * (dts[Math.floor(dts.length / 2)] || 1000));
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    for (const sr of this.series) {
      ctx.strokeStyle = sr.color;
      ctx.beginPath();
      let pen = false; let prevT = null;
      for (const s of this.view) {
        const v = s[sr.key];
        if (v == null || !Number.isFinite(v)) { pen = false; continue; }
        const x = X(s.t); const y = Y(v);
        if (pen && prevT != null && s.t - prevT <= gapMs) ctx.lineTo(x, y); else ctx.moveTo(x, y);
        pen = true; prevT = s.t;
      }
      ctx.stroke();
    }
    // end dots + end labels (selective direct labels)
    const labels = [];
    for (const sr of this.series) {
      const last = [...this.view].reverse().find((s) => s[sr.key] != null && Number.isFinite(s[sr.key]));
      if (!last) continue;
      const x = X(last.t); const y = Y(last[sr.key]);
      ctx.fillStyle = css('--surface');
      ctx.beginPath(); ctx.arc(x, y, 6, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = sr.color;
      ctx.beginPath(); ctx.arc(x, y, 4, 0, 2 * Math.PI); ctx.fill();
      labels.push({ y, text: fmtNum(last[sr.key], this.decimals) });
    }
    labels.sort((a, b) => a.y - b.y);
    for (let i = 1; i < labels.length; i++) labels[i].y = Math.max(labels[i].y, labels[i - 1].y + 13);
    ctx.font = FONT_SMALL;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = css('--ink-2');
    for (const lb of labels) ctx.fillText(lb.text, l + pw + 8, Math.min(t + ph, Math.max(t + 4, lb.y)));
    // crosshair + tooltip
    if (this.hover != null && this.view[this.hover]) {
      const s = this.view[this.hover];
      const x = Math.round(X(s.t)) + 0.5;
      ctx.strokeStyle = css('--axis');
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x, t); ctx.lineTo(x, t + ph); ctx.stroke();
      for (const sr of this.series) {
        const v = s[sr.key];
        if (v == null || !Number.isFinite(v)) continue;
        ctx.fillStyle = css('--surface');
        ctx.beginPath(); ctx.arc(x, Y(v), 6, 0, 2 * Math.PI); ctx.fill();
        ctx.fillStyle = sr.color;
        ctx.beginPath(); ctx.arc(x, Y(v), 4, 0, 2 * Math.PI); ctx.fill();
      }
      if (this.tip) {
        const time = new Date(s.t).toISOString().slice(11, 19);
        this.tip.innerHTML = `<div class="tip-time">${time} UTC</div>` + this.series.map((sr) =>
          `<div class="tip-row"><span class="sw" style="background:${sr.color}"></span>${sr.label}` +
          `<b>${fmtNum(s[sr.key], this.decimals)}${this.unit ? ' ' + this.unit : ''}</b></div>`).join('');
        this.tip.hidden = false;
        const tw = this.tip.offsetWidth;
        this.tip.style.left = `${x + 12 + tw > w ? x - tw - 12 : x + 12}px`;
        this.tip.style.top = `${t + 4}px`;
      }
    } else if (this.tip) {
      this.tip.hidden = true;
    }
  }
}

// ======================================================= AttitudeIndicator
/** Artificial horizon: +pitch = nose up (horizon moves down), +roll = right side down. */
export class AttitudeIndicator {
  constructor(canvas) {
    this.canvas = canvas;
    this.roll = 0; this.pitch = 0; this.valid = false;
    observe(this, canvas);
  }

  set(roll, pitch) {
    this.valid = roll != null && pitch != null;
    this.roll = roll || 0; this.pitch = pitch || 0;
    this.draw();
  }

  draw() {
    const { ctx, w, h } = fitCanvas(this.canvas);
    ctx.clearRect(0, 0, w, h);
    if (Math.min(w, h) < 40) return; // not laid out (hidden / collapsed)
    const r = Math.min(w, h) / 2 - 6;
    const cx = w / 2; const cy = h / 2;
    const pxPerDeg = r / 35;
    ctx.save();
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2 * Math.PI); ctx.clip();
    ctx.translate(cx, cy);
    ctx.rotate((-this.roll * Math.PI) / 180);
    const off = this.pitch * pxPerDeg;
    ctx.fillStyle = css('--sky');
    ctx.fillRect(-2 * r, -3 * r + off, 4 * r, 3 * r);
    ctx.fillStyle = css('--ground');
    ctx.fillRect(-2 * r, off, 4 * r, 3 * r);
    ctx.strokeStyle = css('--ink');
    ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(-2 * r, off); ctx.lineTo(2 * r, off); ctx.stroke();
    ctx.font = FONT_SMALL;
    ctx.fillStyle = css('--ink-2');
    ctx.textBaseline = 'middle';
    ctx.lineWidth = 1;
    for (const p of [-30, -20, -10, 10, 20, 30]) {
      const y = off - p * pxPerDeg;
      const half = r * (p % 20 === 0 ? 0.28 : 0.16);
      ctx.strokeStyle = css('--ink-2');
      ctx.beginPath(); ctx.moveTo(-half, y); ctx.lineTo(half, y); ctx.stroke();
      if (p % 20 === 0) { ctx.textAlign = 'left'; ctx.fillText(String(Math.abs(p)), half + 4, y); }
    }
    ctx.restore();
    // roll scale (fixed) + pointer (rotates)
    ctx.save();
    ctx.translate(cx, cy);
    ctx.strokeStyle = css('--ink-2');
    ctx.lineWidth = 1.5;
    for (const a of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
      const ang = ((a - 90) * Math.PI) / 180;
      const len = a % 30 === 0 ? 9 : 5;
      ctx.beginPath();
      ctx.moveTo(Math.cos(ang) * (r - 1), Math.sin(ang) * (r - 1));
      ctx.lineTo(Math.cos(ang) * (r - 1 - len), Math.sin(ang) * (r - 1 - len));
      ctx.stroke();
    }
    ctx.rotate((-this.roll * Math.PI) / 180);
    ctx.fillStyle = css('--ink');
    ctx.beginPath(); ctx.moveTo(0, -r + 11); ctx.lineTo(-5, -r + 19); ctx.lineTo(5, -r + 19); ctx.closePath(); ctx.fill();
    ctx.restore();
    // fixed rover symbol
    ctx.strokeStyle = css('--page');
    ctx.lineWidth = 5;
    const sym = () => {
      ctx.beginPath();
      ctx.moveTo(cx - r * 0.55, cy); ctx.lineTo(cx - r * 0.18, cy); ctx.lineTo(cx - r * 0.1, cy + 7);
      ctx.moveTo(cx + r * 0.55, cy); ctx.lineTo(cx + r * 0.18, cy); ctx.lineTo(cx + r * 0.1, cy + 7);
      ctx.stroke();
    };
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    sym();
    ctx.strokeStyle = css('--ink');
    ctx.lineWidth = 2.5;
    sym();
    ctx.fillStyle = css('--ink');
    ctx.beginPath(); ctx.arc(cx, cy, 2.5, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = css('--border-strong');
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2 * Math.PI); ctx.stroke();
    if (!this.valid) {
      ctx.fillStyle = 'rgba(13,17,23,0.6)';
      ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = css('--muted');
      ctx.font = FONT;
      ctx.textAlign = 'center';
      ctx.fillText('no attitude data', cx, cy + r * 0.45);
    }
  }
}

// ================================================================ TrackPlot
/** 2-D pose history (map frame: x right, y up), equal aspect, auto-fit. */
export class TrackPlot {
  constructor(canvas, { tip = null } = {}) {
    this.canvas = canvas;
    this.tip = tip;
    this.points = []; // [{t, x, y}]
    this.pose = null; // {x, y, heading}
    this.hover = null;
    canvas.addEventListener('mousemove', (e) => this._onMove(e));
    canvas.addEventListener('mouseleave', () => { this.hover = null; this.draw(); });
    observe(this, canvas);
  }

  set(points, pose) { this.points = points; this.pose = pose; this.draw(); }

  _onMove(e) {
    if (!this.points.length || !this._tf) return;
    const rect = this.canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left; const my = e.clientY - rect.top;
    let best = null; let bd = 24 * 24; // 24 px hit radius
    for (const p of this.points) {
      const [x, y] = this._tf(p.x, p.y);
      const d = (x - mx) ** 2 + (y - my) ** 2;
      if (d < bd) { bd = d; best = p; }
    }
    this.hover = best;
    this.draw();
  }

  draw() {
    const { ctx, w, h } = fitCanvas(this.canvas);
    ctx.clearRect(0, 0, w, h);
    if (w < 40 || h < 40) return; // not laid out (hidden / collapsed)
    const pts = this.points;
    let x0 = -1; let x1 = 1; let y0 = -1; let y1 = 1;
    for (const p of pts) { x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y); }
    if (this.pose) {
      x0 = Math.min(x0, this.pose.x); x1 = Math.max(x1, this.pose.x);
      y0 = Math.min(y0, this.pose.y); y1 = Math.max(y1, this.pose.y);
    }
    const m = 0.8;
    x0 -= m; x1 += m; y0 -= m; y1 += m;
    const pad = 10;
    const s = Math.min((w - 2 * pad) / (x1 - x0), (h - 2 * pad) / (y1 - y0));
    const ox = pad + ((w - 2 * pad) - s * (x1 - x0)) / 2;
    const oy = pad + ((h - 2 * pad) - s * (y1 - y0)) / 2;
    const tf = (x, y) => [ox + (x - x0) * s, h - (oy + (y - y0) * s)];
    this._tf = tf;
    // grid
    const step = niceStep(Math.max(x1 - x0, y1 - y0), 5);
    ctx.lineWidth = 1;
    ctx.strokeStyle = css('--grid');
    for (let gx = Math.ceil(x0 / step) * step; gx <= x1; gx += step) {
      const [px] = tf(gx, 0);
      ctx.beginPath(); ctx.moveTo(Math.round(px) + 0.5, 0); ctx.lineTo(Math.round(px) + 0.5, h); ctx.stroke();
    }
    for (let gy = Math.ceil(y0 / step) * step; gy <= y1; gy += step) {
      const [, py] = tf(0, gy);
      ctx.beginPath(); ctx.moveTo(0, Math.round(py) + 0.5); ctx.lineTo(w, Math.round(py) + 0.5); ctx.stroke();
    }
    // origin
    const [ox0, oy0] = tf(0, 0);
    ctx.strokeStyle = css('--axis');
    ctx.beginPath(); ctx.arc(ox0, oy0, 5, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = css('--muted');
    ctx.font = FONT_SMALL;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillText('start', ox0 + 7, oy0 - 3);
    // trail
    if (pts.length > 1) {
      ctx.strokeStyle = css('--series-1');
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.beginPath();
      pts.forEach((p, i) => { const [x, y] = tf(p.x, p.y); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
      ctx.stroke();
    }
    // rover
    if (this.pose) {
      const [px, py] = tf(this.pose.x, this.pose.y);
      const a = (-this.pose.heading * Math.PI) / 180;
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(a);
      ctx.fillStyle = css('--ink');
      ctx.strokeStyle = css('--surface');
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(11, 0); ctx.lineTo(-7, 7); ctx.lineTo(-3, 0); ctx.lineTo(-7, -7); ctx.closePath();
      ctx.stroke(); ctx.fill();
      ctx.restore();
    }
    // scale bar
    const barPx = step * s;
    ctx.strokeStyle = css('--ink-2');
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(10, h - 10); ctx.lineTo(10 + barPx, h - 10); ctx.stroke();
    ctx.fillStyle = css('--ink-2');
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`${step} m`, 10, h - 14);
    // hover
    if (this.hover) {
      const [hx, hy] = tf(this.hover.x, this.hover.y);
      ctx.fillStyle = css('--surface');
      ctx.beginPath(); ctx.arc(hx, hy, 6, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = css('--series-1');
      ctx.beginPath(); ctx.arc(hx, hy, 4, 0, 2 * Math.PI); ctx.fill();
      if (this.tip) {
        this.tip.innerHTML = `<div class="tip-time">${new Date(this.hover.t).toISOString().slice(11, 19)} UTC</div>` +
          `<div class="tip-row">x<b>${this.hover.x.toFixed(2)} m</b></div><div class="tip-row">y<b>${this.hover.y.toFixed(2)} m</b></div>`;
        this.tip.hidden = false;
        const tw = this.tip.offsetWidth;
        this.tip.style.left = `${hx + 12 + tw > w ? hx - tw - 12 : hx + 12}px`;
        this.tip.style.top = `${Math.max(4, hy - 30)}px`;
      }
    } else if (this.tip) {
      this.tip.hidden = true;
    }
  }
}
