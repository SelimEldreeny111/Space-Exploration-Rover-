// Built-in virtual rover: a JavaScript port of ground_station/python/virtual_rover.py
// (same command semantics, same world model) plus a simulated HC-12 radio link
// (9600 baud pacing, air latency, bit errors) so the ground station can be
// demonstrated without hardware.

import { Frame, StreamDecoder } from './ssp.js';
import {
  Addr, ArmSetJoints, Drive, Fault, IMAGE_CHUNK_PAYLOAD, ImageChunkHeader, LEGACY_IMAGE, Mode, Nack,
  NackReason, NO_ECHO, PT, TlmLegacy, TlmV2, clampI16, clampI32, clampU16,
} from './protocol.js';

export const ROVER = Object.freeze({
  maxSpeed: 0.45, // m/s (rover_core.params.RoverParams.max_speed)
  maxYawRate: 1.0, // rad/s
  // ultrasonic mounting (x, y, yaw_deg) in base_link, ICD §9.3 order
  usPoses: [[0.50, 0.18, 30], [0.52, 0.0, 0], [0.50, -0.18, -30], [0.10, 0.33, 90], [0.10, -0.33, -90],
    [-0.50, 0.0, 180]],
  usFovDeg: 15,
  usRange: [0.02, 4.0],
});
export const JPEG_W = 160;
export const JPEG_H = 120;
const G = 9.81;
const DT = 0.05;
const BODY_R = 0.45; // collision radius of the rover body [m]
const ARM_LIMITS = [[0, 180], [15, 165], [0, 180], [0, 180], [0, 180], [10, 80]];
const BRAIN_REQUESTS = new Map([
  [PT.PING, [0]], [PT.GET_IMAGE, [0, 1]], [PT.CAP_IMAGE, [0]], [PT.GET_TLM, [0]], [PT.START_SCAN, [0]],
  [PT.STOP_SCAN, [0]], [PT.COLLECT_OBJECT, [0]], [PT.ESTOP, [0]], [PT.CLEAR_ESTOP, [0]], [PT.SET_MODE, [1]],
  [PT.DRIVE, [4]], [PT.ARM_SET_POSE, [1]], [PT.ARM_SET_JOINTS, [8]],
]);
const LEGACY_REQUESTS = new Set([PT.PING, PT.GET_IMAGE, PT.CAP_IMAGE, PT.GET_TLM, PT.START_SCAN, PT.STOP_SCAN,
  PT.COLLECT_OBJECT]);
const SAFE_LINK_REQUESTS = new Set([PT.PING, PT.GET_TLM, PT.STOP_SCAN, PT.ESTOP, PT.CLEAR_ESTOP]);

// ------------------------------------------------------------------ helpers
export function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

class Rng {
  constructor(seed) { this.random = mulberry32(seed); this._spare = null; }
  uniform(a, b) { return a + (b - a) * this.random(); }
  int(n) { return Math.floor(this.random() * n); }
  gauss(mu = 0, sigma = 1) {
    if (this._spare !== null) { const s = this._spare; this._spare = null; return mu + sigma * s; }
    let u = 0;
    while (u === 0) u = this.random();
    const v = this.random();
    const r = Math.sqrt(-2 * Math.log(u));
    this._spare = r * Math.sin(2 * Math.PI * v);
    return mu + sigma * r * Math.cos(2 * Math.PI * v);
  }
}

const wrapPi = (a) => {
  const t = (a + Math.PI) % (2 * Math.PI);
  return (t < 0 ? t + 2 * Math.PI : t) - Math.PI;
};
const wrapDeg = (a) => {
  let t = (a + 180) % 360;
  if (t < 0) t += 360;
  t -= 180;
  return t === -180 ? 180 : t;
};
const deg = (r) => (r * 180) / Math.PI;
const rad = (d) => (d * Math.PI) / 180;
const clampInt = (v, lo, hi) => Math.max(lo, Math.min(hi, Math.trunc(v)));

// ==================================================================== world
export class World {
  static HALF = 10.0;

  constructor(seed = 7) {
    const rng = new Rng(seed);
    this.rocks = [];
    let guard = 0;
    while (this.rocks.length < 16 && guard++ < 10000) {
      const x = rng.uniform(-9, 9); const y = rng.uniform(-9, 9); const r = rng.uniform(0.2, 0.6);
      if (Math.hypot(x, y) < 2.2) continue;
      if (this.rocks.some(([a, b, c]) => Math.hypot(x - a, y - b) < r + c + 0.9)) continue;
      this.rocks.push([x, y, r]);
    }
    this.metal = [[1.8, 0.3]];
    for (let i = 0; i < 5; i++) this.metal.push([rng.uniform(-7, 7), rng.uniform(-7, 7)]);
    this.gas = [[rng.uniform(-5, 5), rng.uniform(-5, 5), 1.8], [rng.uniform(-6, 6), rng.uniform(-6, 6), 2.4]];
  }

  static height(x, y) {
    return 0.18 * Math.sin(0.45 * x) * Math.cos(0.38 * y) + 0.07 * Math.sin(1.1 * x + 0.8 * y);
  }

  static soil(x, y) { return 600 + 170 * Math.sin(0.55 * x + 0.3) * Math.cos(0.42 * y); }

  gasLevel(x, y, k) {
    const [gx, gy, s] = this.gas[k];
    return Math.exp(-((x - gx) ** 2 + (y - gy) ** 2) / (2 * s * s));
  }

  raycast(x, y, ang, rmax) {
    const dx = Math.cos(ang); const dy = Math.sin(ang);
    let best = rmax;
    for (const [cx, cy, r] of this.rocks) {
      const fx = x - cx; const fy = y - cy;
      const b = fx * dx + fy * dy;
      const c = fx * fx + fy * fy - r * r;
      const disc = b * b - c;
      if (disc < 0) continue;
      const sq = Math.sqrt(disc);
      const t = (-b - sq >= 0) ? -b - sq : -b + sq;
      if (t >= 0 && t < best) best = t;
    }
    const h = World.HALF;
    for (const [p, d] of [[x, dx], [y, dy]]) {
      if (d > 1e-9) best = Math.min(best, (h - p) / d);
      else if (d < -1e-9) best = Math.min(best, (-h - p) / d);
    }
    return Math.max(0, best);
  }
}

// =================================================================== camera
function hash01(ix, iy, seed) {
  let n = (Math.imul(ix, 374761393) + Math.imul(iy, 668265263) + Math.imul(seed, 1442695041)) >>> 0;
  n = Math.imul(n ^ (n >>> 13), 1274126177) >>> 0;
  return ((n ^ (n >>> 16)) & 0xFFFF) / 65535;
}

function vnoise(x, y, seed) {
  const ix = Math.floor(x); const iy = Math.floor(y);
  const fx = x - ix; const fy = y - iy;
  const sx = fx * fx * (3 - 2 * fx); const sy = fy * fy * (3 - 2 * fy);
  const a = hash01(ix, iy, seed); const b = hash01(ix + 1, iy, seed);
  const c = hash01(ix, iy + 1, seed); const d = hash01(ix + 1, iy + 1, seed);
  return a + (b - a) * sx + (c - a) * sy + (a - b - c + d) * sx * sy;
}

function fbm(x, y, seed, octaves = 4) {
  let s = 0; let amp = 1; let f = 1; let norm = 0;
  for (let o = 0; o < octaves; o++) {
    s += amp * vnoise(x * f, y * f, seed + o * 101);
    norm += amp; amp *= 0.5; f *= 2;
  }
  return s / norm;
}

/** What the camera saw at CAP_IMAGE time (rendered on demand at any size). */
export class Scene {
  constructor({ seed, pitchDeg = 0, rollDeg = 0, rockBearings = [] }) {
    this.seed = seed; this.pitchDeg = pitchDeg; this.rollDeg = rollDeg; this.rockBearings = rockBearings;
  }

  render(w, h) {
    const rng = new Rng(this.seed);
    const horizon = h * (0.36 + this.pitchDeg / 70);
    const tilt = Math.tan(rad(this.rollDeg));
    const rocks = [];
    for (const [bearing, dist, r] of this.rockBearings) {
      const cx = 0.5 - bearing / 62;
      const depth = Math.max(0.08, Math.min(1, 1.4 / Math.max(dist, 0.3)));
      const cy = horizon / h + (1 - horizon / h) * Math.min(0.92, 0.18 + 0.75 * depth);
      rocks.push([cx, cy, Math.max(0.035, 0.09 * r * depth * 3), rng.uniform(0.55, 0.8)]);
    }
    const nPebbles = 2 + rng.int(3);
    for (let i = 0; i < nPebbles; i++) {
      rocks.push([rng.uniform(0.05, 0.95), rng.uniform(0.62, 0.95), rng.uniform(0.02, 0.06), rng.uniform(0.5, 0.75)]);
    }
    rocks.sort((a, b) => a[1] - b[1]);
    const out = new Uint8Array(w * h);
    const s = this.seed & 0xFFFF;
    for (let py = 0; py < h; py++) {
      for (let px = 0; px < w; px++) {
        const u = px / w; const v = py / h;
        const hz = horizon + (px - w / 2) * tilt;
        let val;
        if (py < hz) {
          const ridge = hz - h * (0.05 + 0.06 * fbm((px / w) * 3, 0.5, s + 7, 3));
          if (py > ridge) val = 120 + 30 * fbm(px / 9, py / 9, s + 3, 3);
          else val = 150 + 55 * (py / Math.max(1, hz)) + 6 * fbm(u * 3, v * 3, s + 11, 2);
        } else {
          const depth = (py - hz) / Math.max(1, h - hz);
          const sc = 4 + 18 * depth;
          val = 92 + 70 * fbm(u * sc, depth * sc * 1.7, s, 4) + 25 * depth;
          for (const [cx, cy, r, shade] of rocks) {
            const dx = (u - cx) / r; const dy = (v - cy) / (r * 0.62);
            const d2 = dx * dx + dy * dy;
            if (d2 < 1 && dy < 0.55) {
              const nz = Math.sqrt(1 - d2);
              const lam = Math.max(0, -0.55 * dx - 0.45 * dy + 0.7 * nz);
              val = 40 + 170 * shade * lam + 18 * fbm(u * 40, v * 40, s + 5, 2);
            } else if (dy > 0 && dy < 1.6 && dx * dx + (dy - 0.55) ** 2 < 1.1) {
              val *= 0.62;
            }
          }
        }
        val *= 1 - 0.25 * ((u - 0.5) ** 2 + (v - 0.5) ** 2);
        out[py * w + px] = Math.max(0, Math.min(255, Math.trunc(val)));
      }
    }
    return out;
  }

  /** JPEG via OffscreenCanvas (browser); null where unavailable (Node). */
  async jpeg(quality) {
    if (typeof OffscreenCanvas === 'undefined') return null;
    try {
      const px = this.render(JPEG_W, JPEG_H);
      const cv = new OffscreenCanvas(JPEG_W, JPEG_H);
      const ctx = cv.getContext('2d');
      const img = ctx.createImageData(JPEG_W, JPEG_H);
      for (let i = 0; i < px.length; i++) {
        img.data[4 * i] = img.data[4 * i + 1] = img.data[4 * i + 2] = px[i];
        img.data[4 * i + 3] = 255;
      }
      ctx.putImageData(img, 0, 0);
      const blob = await cv.convertToBlob({ type: 'image/jpeg', quality: Math.max(1, Math.min(95, quality)) / 100 });
      return new Uint8Array(await blob.arrayBuffer());
    } catch (e) {
      return null;
    }
  }
}

// ================================================================ the rover
/** State machine + physics of the rover as seen from the GS link. */
export class RoverModel {
  constructor({ seed = 1, legacy = false, safeLink = false, timeScale = 1.0, clock = null, world = null } = {}) {
    this.rng = new Rng(seed);
    this.seed = seed;
    this.legacy = legacy;
    this.safeLink = safeLink;
    this.timeScale = timeScale;
    this.clock = clock || (() => performance.now() / 1000);
    this.world = world || new World();
    this.t0 = this.t = this.clock();
    this.mode = safeLink ? Mode.SAFE : Mode.IDLE;
    this.estop = false;
    this.cmdTimeout = false;
    this.x = 0; this.y = 0; this.th = 0;
    this.v = 0; this.w = 0; this.dv = 0;
    this.drive = null; // [v, w, until]
    this.soc = 0.86;
    this.current = 1.4;
    this.collectUntil = 0;
    this.captureBusyUntil = 0;
    this.scene = null;
    this.captures = 0;
    this.roll = 0; this.pitch = 0; this.rollRate = 0; this.pitchRate = 0;
    this.yawDrift = 0;
    this.noise = { temp: 0, rh: 0, p: 0, mq2: 0, mq4: 0, soil: 0, rain: 0 };
    this.rangesM = new Array(6).fill(ROVER.usRange[1]);
    this._avoidTurn = 0;
    this.handled = new Map();
    this._updateRanges();
  }

  // ------------------------------------------------------------ simulation
  advance(now = null) {
    now = now == null ? this.clock() : now;
    let gap = now - this.t;
    if (gap <= 0) return;
    if (gap > 30) { this.t = now - 30; gap = 30; }
    const n = Math.max(1, Math.ceil(gap / DT));
    const dt = gap / n;
    for (let i = 0; i < n; i++) { this.t += dt; this._step(dt); }
    this.t = now;
  }

  _updateRanges() {
    const [rmin, rmax] = ROVER.usRange;
    const half = rad(ROVER.usFovDeg / 2);
    const c = Math.cos(this.th); const s = Math.sin(this.th);
    this.rangesM = ROVER.usPoses.map(([sx, sy, syaw]) => {
      const wx = this.x + c * sx - s * sy; const wy = this.y + s * sx + c * sy;
      const a = this.th + rad(syaw);
      const r = Math.min(...[-1, 0, 1].map((k) => this.world.raycast(wx, wy, a + k * half, rmax + 0.01)));
      return Math.max(rmin, r);
    });
  }

  rangesMm() {
    const rmax = ROVER.usRange[1];
    return this.rangesM.map((r) => (r > rmax ? NO_ECHO : clampU16(r * 1000)));
  }

  _wander(t) {
    const [fl, fc, fr, left, right] = this.rangesM;
    const front = Math.min(fl, fc, fr);
    if (front < 0.9) {
      if (!this._avoidTurn) this._avoidTurn = fl >= fr ? 1 : -1; // commit to one direction (no dithering)
      return [front < 0.5 ? 0 : 0.08, 0.75 * this._avoidTurn];
    }
    this._avoidTurn = 0;
    if (Math.min(left, right) < 0.5) return [0.15, 0.6 * (right < left ? 1 : -1)]; // keep flanks clear
    return [0.28, 0.32 * Math.sin(t / 5) + 0.14 * Math.sin(t / 2.3 + 1)];
  }

  /** Rover body (radius BODY_R) against rocks and arena walls. */
  _collides(x, y) {
    if (Math.abs(x) > World.HALF - BODY_R || Math.abs(y) > World.HALF - BODY_R) return true;
    return this.world.rocks.some(([cx, cy, r]) => Math.hypot(x - cx, y - cy) < r + BODY_R);
  }

  _step(dt) {
    const t = this.t - this.t0;
    if (this.drive && this.t >= this.drive[2]) {
      if (this.drive[0] || this.drive[1]) this.cmdTimeout = true;
      this.drive = null;
    }
    let vCmd = 0; let wCmd = 0;
    const collecting = this.t < this.collectUntil;
    if (this.estop || collecting) { /* motors off */ } else if (this.mode === Mode.AUTONOMOUS) {
      [vCmd, wCmd] = this._wander(t);
    } else if (this.drive) {
      [vCmd, wCmd] = this.drive;
    }
    const a = Math.min(1, dt / 0.25);
    const vOld = this.v;
    this.v += (vCmd - this.v) * a;
    this.w += (wCmd - this.w) * a;
    if (this.estop) { this.v = 0; this.w = 0; }
    this._updateRanges();
    if (this.v > 0 && Math.min(...this.rangesM.slice(0, 3)) < 0.3) this.v = 0;
    if (this.v < 0 && this.rangesM[5] < 0.3) this.v = 0;
    this.dv = (this.v - vOld) / dt;
    this.th = wrapPi(this.th + this.w * dt);
    const nx = this.x + this.v * Math.cos(this.th) * dt;
    const ny = this.y + this.v * Math.sin(this.th) * dt;
    if (this._collides(nx, ny) && !this._collides(this.x, this.y)) this.v = 0; // blocked: stay put
    else { this.x = nx; this.y = ny; }
    const h = World.height; const ds = 0.3; const c = Math.cos(this.th); const s = Math.sin(this.th);
    const pitchT = deg(Math.atan2(h(this.x + ds * c, this.y + ds * s) - h(this.x - ds * c, this.y - ds * s), 2 * ds));
    const rollT = deg(Math.atan2(h(this.x - ds * s, this.y + ds * c) - h(this.x + ds * s, this.y - ds * c), 2 * ds));
    const vib = 0.6 * Math.abs(this.v) / ROVER.maxSpeed;
    const k = Math.min(1, dt / 0.3);
    const np = this.pitch + (pitchT - this.pitch) * k + this.rng.gauss(0, vib * 0.3 + 0.01);
    const nr = this.roll + (rollT - this.roll) * k + this.rng.gauss(0, vib * 0.3 + 0.01);
    this.pitchRate = (np - this.pitch) / dt; this.rollRate = (nr - this.roll) / dt;
    this.pitch = np; this.roll = nr;
    this.yawDrift += 0.005 * dt;
    this.current = 1.4 + 7 * Math.abs(this.v) / ROVER.maxSpeed + 2 * Math.abs(this.w) / ROVER.maxYawRate +
      (collecting ? 1.5 : 0);
    this.soc = Math.max(0, this.soc - this.current * dt / 3600 / 5);
    for (const [key, sigma] of [['temp', 0.05], ['rh', 0.15], ['p', 1.5], ['mq2', 2.0], ['mq4', 1.5],
      ['soil', 1.5], ['rain', 0.8]]) {
      this.noise[key] = 0.995 * this.noise[key] + this.rng.gauss(0, sigma);
    }
  }

  // --------------------------------------------------------------- sensors
  batteryV() {
    return 11.0 + 1.6 * this.soc ** 1.3 - 0.035 * this.current + this.rng.gauss(0, 0.005);
  }

  env() {
    const t = this.t - this.t0; const { x, y } = this; const n = this.noise;
    return {
      temp: 26.5 + 2.5 * Math.sin((2 * Math.PI * t) / 900) + n.temp,
      rh: 41 + 5 * Math.sin((2 * Math.PI * t) / 1200 + 1) + n.rh,
      p: 100850 + 40 * Math.sin((2 * Math.PI * t) / 1800) - 12 * World.height(x, y) + n.p,
      mq2: 140 + 420 * this.world.gasLevel(x, y, 0) + n.mq2,
      mq4: 95 + 300 * this.world.gasLevel(x, y, 1) + n.mq4,
      soil: World.soil(x, y) + n.soil,
      rain: 1010 - Math.abs(n.rain) * 3,
      metal: this.world.metal.some(([mx, my]) => Math.hypot(x - mx, y - my) < 0.6) ? 1 : 0,
    };
  }

  faults() {
    let f = 0;
    if (this.estop) f |= Fault.ESTOP;
    if (this.cmdTimeout) f |= Fault.CMD_TIMEOUT;
    if (this.batteryV() < 10.8) f |= Fault.LOW_BATTERY;
    if (this.safeLink) f |= Fault.SAFE_MODE;
    return f;
  }

  tlmV2() {
    const e = this.env();
    const rr = rad(this.roll); const pr = rad(this.pitch);
    const vib = 300 * Math.abs(this.v) / ROVER.maxSpeed;
    const noise = () => this.rng.gauss(0, vib + 15);
    const valid = this.rangesMm().filter((r) => r !== NO_ECHO);
    const t = new TlmV2({
      tlm_version: 2, mode: this.mode, fault_flags: this.faults(),
      battery_mV: clampU16(this.batteryV() * 1000), uptime_s: Math.trunc(this.t - this.t0),
      temperature_cC: clampI16(e.temp * 100), humidity_cP: clampU16(e.rh * 100),
      pressure_Pa: Math.round(e.p), roll_cdeg: clampI16(this.roll * 100), pitch_cdeg: clampI16(this.pitch * 100),
      yaw_cdeg: clampI16(wrapDeg(deg(this.th) + this.yawDrift) * 100),
      ax: clampI16((G * Math.sin(pr) + this.dv) * 1000 + noise()),
      ay: clampI16((G * Math.sin(rr) * Math.cos(pr) + this.v * this.w) * 1000 + noise()),
      az: clampI16(G * Math.cos(rr) * Math.cos(pr) * 1000 + noise()),
      gx: clampI16(rad(this.rollRate) * 1000), gy: clampI16(rad(this.pitchRate) * 1000),
      gz: clampI16(this.w * 1000 + this.rng.gauss(0, 3)),
      mq2_raw: clampInt(e.mq2, 0, 1023), mq4_raw: clampInt(e.mq4, 0, 1023),
      soil_raw: clampInt(e.soil, 0, 1023), rain_raw: clampInt(e.rain, 0, 1023), proximity: e.metal,
      x_mm: clampI32(this.x * 1000), y_mm: clampI32(this.y * 1000),
      heading_cdeg: clampI16(wrapDeg(deg(this.th)) * 100),
      range_min_mm: valid.length ? Math.min(...valid) : NO_ECHO,
    });
    if (this.safeLink) { // Mega safe mode: fields it cannot measure are 0 (ICD §7.1)
      const keep = new Set(['tlm_version', 'mode', 'fault_flags', 'battery_mV', 'uptime_s', 'range_min_mm']);
      for (const [name] of TlmV2.FIELDS) if (!keep.has(name)) t[name] = 0;
    }
    return t;
  }

  tlmLegacy() {
    const e = this.env();
    const gas = clampInt(e.mq2, 0, 1023);
    const cm = this.rangesMm().map((r) => (r === NO_ECHO ? 0 : Math.min(255, Math.trunc(r / 10))));
    return new TlmLegacy({ ir: e.metal, gas_hi: gas >> 8, gas_lo: gas & 0xFF, us1_cm: cm[1], us2_cm: cm[5],
      temp_c: clampInt(Math.round(e.temp), 0, 255), rh_pct: clampInt(Math.round(e.rh), 0, 255) });
  }

  // ---------------------------------------------------------------- camera
  capture() {
    this.captures += 1;
    const seen = [];
    for (const [cx, cy, r] of this.world.rocks) {
      const d = Math.hypot(cx - this.x, cy - this.y);
      const b = deg(wrapPi(Math.atan2(cy - this.y, cx - this.x) - this.th));
      if (d < 6 && Math.abs(b) < 31) seen.push([b, d, r]);
    }
    this.scene = new Scene({ seed: (this.seed * 7919 + this.captures * 104729) & 0x7FFFFFFF,
      pitchDeg: this.pitch, rollDeg: this.roll, rockBearings: seen });
    return this.scene;
  }

  async imageFrames(req, me, quality) {
    const scene = this.scene || this.capture();
    const { width: LW, height: LH, chunks: LN, chunkSize: LC } = LEGACY_IMAGE;
    if (quality == null) { // legacy: 50 x 200 B, type 0x08
      const px = scene.render(LW, LH);
      return Array.from({ length: LN }, (_, i) => new Frame(req.src, me, PT.GET_IMAGE, px.subarray(i * LC, (i + 1) * LC)));
    }
    let blob = null; let w = LW; let h = LH; let fmt = 0;
    if (quality > 0) {
      blob = await scene.jpeg(quality);
      if (blob) { w = JPEG_W; h = JPEG_H; fmt = 1; }
    }
    if (!blob) blob = scene.render(LW, LH);
    const total = Math.ceil(blob.length / IMAGE_CHUNK_PAYLOAD);
    const frames = [];
    for (let i = 0; i < total; i++) {
      const hdr = new ImageChunkHeader({ seq: i, total, width: w, height: h, fmt }).pack();
      const part = blob.subarray(i * IMAGE_CHUNK_PAYLOAD, (i + 1) * IMAGE_CHUNK_PAYLOAD);
      const data = new Uint8Array(hdr.length + part.length);
      data.set(hdr); data.set(part, hdr.length);
      frames.push(new Frame(req.src, me, PT.IMAGE_CHUNK, data));
    }
    return frames;
  }

  // -------------------------------------------------------------- commands
  /** Process one received frame -> [[delay_s, Frame], ...] (may return a Promise). */
  handle(fr, now = null) {
    now = now == null ? this.clock() : now;
    this.advance(now);
    if (![Addr.ROVER, Addr.MOBILITY, Addr.PAYLOAD, Addr.ARM, Addr.BROADCAST].includes(fr.dest)) return [];
    this.handled.set(fr.type, (this.handled.get(fr.type) || 0) + 1);
    const me = fr.dest === Addr.BROADCAST ? Addr.ROVER : fr.dest;
    if (this.legacy) return this._handleLegacy(fr, now);
    if (this.safeLink) return this._handleSafeLink(fr);
    if (me !== Addr.ROVER) return this._handleMcu(fr, me);
    return this._handleBrain(fr, me, now);
  }

  _out(fr, me, ptype, data = [], delay = 0) { return [delay * this.timeScale, new Frame(fr.src, me, ptype, data)]; }
  _ack(fr, me, delay = 0.003) { return [this._out(fr, me, PT.ACK, [fr.type], delay)]; }
  _nack(fr, me, reason) {
    return [this._out(fr, me, PT.NACK, new Nack({ rejected_type: fr.type, reason }).pack(), 0.003)];
  }

  _handleBrain(fr, me, now) {
    const t = fr.type; const d = fr.data;
    if (!BRAIN_REQUESTS.has(t)) return this._nack(fr, me, NackReason.UNKNOWN_TYPE);
    if (!BRAIN_REQUESTS.get(t).includes(d.length)) return this._nack(fr, me, NackReason.BAD_LENGTH);
    const collecting = now < this.collectUntil;
    switch (t) {
      case PT.PING: return [this._out(fr, me, PT.ACK, [], 0.002)];
      case PT.GET_TLM: return [this._out(fr, me, PT.GET_TLM, this.tlmV2().pack(), 0.005)];
      case PT.CAP_IMAGE:
        if (now < this.captureBusyUntil) return this._nack(fr, me, NackReason.BUSY);
        this.capture();
        this.captureBusyUntil = now + 0.35 * this.timeScale;
        return [this._out(fr, me, PT.CAP_IMAGE, [], 0.35)];
      case PT.GET_IMAGE:
        return this.imageFrames(fr, me, d.length ? d[0] : null)
          .then((frames) => frames.map((f) => [0.05 * this.timeScale, f]));
      case PT.START_SCAN:
        if (this.estop || this.mode === Mode.SAFE) return this._nack(fr, me, NackReason.NOT_PERMITTED);
        if (collecting) return this._nack(fr, me, NackReason.BUSY);
        this.mode = Mode.AUTONOMOUS; this.drive = null;
        return [this._out(fr, me, PT.START_SCAN, [], 0.01)];
      case PT.STOP_SCAN:
        if (this.mode !== Mode.SAFE) this.mode = Mode.IDLE;
        this.drive = null; this.v = 0; this.w = 0;
        return [this._out(fr, me, PT.STOP_SCAN, [], 0.01)];
      case PT.COLLECT_OBJECT:
        if (this.estop) return this._nack(fr, me, NackReason.NOT_PERMITTED);
        if (collecting) return this._nack(fr, me, NackReason.BUSY);
        if (this.mode === Mode.AUTONOMOUS) this.mode = Mode.IDLE;
        this.drive = null;
        this.collectUntil = now + 6 * this.timeScale;
        return [this._out(fr, me, PT.COLLECT_OBJECT, [], 0.01)];
      case PT.ESTOP:
        this.estop = true; this.drive = null; this.v = 0; this.w = 0;
        if (this.mode !== Mode.SAFE) this.mode = Mode.IDLE;
        return this._ack(fr, me, 0.002);
      case PT.CLEAR_ESTOP:
        this.estop = false; this.cmdTimeout = false;
        return this._ack(fr, me);
      case PT.SET_MODE: {
        const m = d[0];
        if (m > Mode.SAFE) return this._nack(fr, me, NackReason.NOT_PERMITTED);
        if (this.estop && (m === Mode.TELEOP || m === Mode.AUTONOMOUS)) return this._nack(fr, me, NackReason.NOT_PERMITTED);
        this.mode = m; this.drive = null; this.cmdTimeout = false;
        return this._ack(fr, me);
      }
      case PT.DRIVE: {
        if (this.estop) return this._nack(fr, me, NackReason.NOT_PERMITTED);
        if (this.mode === Mode.AUTONOMOUS || collecting) return this._nack(fr, me, NackReason.BUSY);
        const cmd = Drive.unpack(d);
        if (this.mode === Mode.IDLE) this.mode = Mode.TELEOP;
        const scale = this.mode === Mode.SAFE ? 0.3 : 1.0;
        const v = (Math.max(-100, Math.min(100, cmd.v_pct)) / 100) * ROVER.maxSpeed * scale;
        const w = (Math.max(-100, Math.min(100, cmd.w_pct)) / 100) * ROVER.maxYawRate * scale;
        this.drive = (v || w) && cmd.duration_ms ? [v, w, now + cmd.duration_ms / 1000] : null;
        this.cmdTimeout = false;
        return this._ack(fr, me);
      }
      case PT.ARM_SET_POSE:
      case PT.ARM_SET_JOINTS:
        return this._arm(fr, me);
      default:
        return this._nack(fr, me, NackReason.UNKNOWN_TYPE);
    }
  }

  _arm(fr, me) {
    if (this.estop) return this._nack(fr, me, NackReason.NOT_PERMITTED);
    if (fr.type === PT.ARM_SET_POSE) {
      if (fr.data.length !== 1) return this._nack(fr, me, NackReason.BAD_LENGTH);
      if (fr.data[0] > 7) return this._nack(fr, me, NackReason.NOT_PERMITTED);
      return this._ack(fr, me);
    }
    if (fr.data.length !== ArmSetJoints.size()) return this._nack(fr, me, NackReason.BAD_LENGTH);
    const js = ArmSetJoints.unpack(fr.data);
    if (js.deg.some((v, i) => v < ARM_LIMITS[i][0] || v > ARM_LIMITS[i][1])) {
      return this._nack(fr, me, NackReason.NOT_PERMITTED);
    }
    return this._ack(fr, me);
  }

  _handleMcu(fr, me) {
    const t = fr.type;
    if (t === PT.PING) return [this._out(fr, me, PT.ACK, [], 0.008)];
    if (me === Addr.MOBILITY && (t === PT.ESTOP || t === PT.CLEAR_ESTOP) && !fr.data.length) {
      this.estop = t === PT.ESTOP;
      if (this.estop) { this.drive = null; this.v = 0; this.w = 0; }
      return this._ack(fr, me, 0.008);
    }
    if (me === Addr.MOBILITY && t === PT.SET_VELOCITY && fr.data.length === 6) return [];
    if (me === Addr.ARM && (t === PT.ARM_SET_POSE || t === PT.ARM_SET_JOINTS)) return this._arm(fr, me);
    return this._nack(fr, me, NackReason.UNKNOWN_TYPE);
  }

  _handleLegacy(fr, now) {
    const me = Addr.ROVER;
    if (![Addr.ROVER, Addr.BROADCAST].includes(fr.dest) || !LEGACY_REQUESTS.has(fr.type)) return [];
    const t = fr.type;
    if (t === PT.PING) return [this._out(fr, me, PT.ACK, [], 0.004)];
    if (t === PT.GET_TLM) return [this._out(fr, me, PT.GET_TLM, this.tlmLegacy().pack(), 0.01)];
    if (t === PT.GET_IMAGE) {
      return this.imageFrames(fr, me, null).then((frames) => frames.map((f) => [0.05 * this.timeScale, f]));
    }
    if (t === PT.CAP_IMAGE) { this.capture(); return [this._out(fr, me, t, [], 0.35)]; }
    if (t === PT.START_SCAN) this.mode = Mode.AUTONOMOUS;
    else if (t === PT.STOP_SCAN) { this.mode = Mode.IDLE; this.drive = null; }
    else if (t === PT.COLLECT_OBJECT) this.collectUntil = now + 6 * this.timeScale;
    return [this._out(fr, me, t, [], 0.01)];
  }

  _handleSafeLink(fr) {
    const me = Addr.MOBILITY;
    if (![Addr.ROVER, Addr.MOBILITY, Addr.BROADCAST].includes(fr.dest)) return [];
    if (!SAFE_LINK_REQUESTS.has(fr.type)) return this._nack(fr, me, NackReason.NOT_PERMITTED);
    if (fr.data.length) return this._nack(fr, me, NackReason.BAD_LENGTH);
    const t = fr.type;
    if (t === PT.PING) return [this._out(fr, me, PT.ACK, [], 0.004)];
    if (t === PT.GET_TLM) return [this._out(fr, me, PT.GET_TLM, this.tlmV2().pack(), 0.006)];
    if (t === PT.STOP_SCAN) {
      this.drive = null; this.v = 0; this.w = 0;
      return [this._out(fr, me, PT.STOP_SCAN, [], 0.004)];
    }
    this.estop = t === PT.ESTOP;
    if (this.estop) { this.drive = null; this.v = 0; this.w = 0; }
    return this._ack(fr, me, 0.004);
  }
}

// ============================================================ radio link
/** Bit-error injector with geometric gaps (independent errors at rate `ber`). */
export class BitErrors {
  constructor(ber, rng) { this.ber = ber; this.rng = rng; this.flipped = 0; this._gap = this._next(); }
  /** change the rate now (geometric gaps are memoryless, so re-drawing is exact) */
  setBer(ber) { this.ber = ber; this._gap = this._next(); }
  _next() {
    if (this.ber <= 0) return Number.MAX_SAFE_INTEGER;
    return Math.floor(Math.log(1 - this.rng()) / Math.log(1 - this.ber));
  }
  apply(data) {
    if (this.ber <= 0 || !data.length) return data;
    const nbits = data.length * 8;
    if (this._gap >= nbits) { this._gap -= nbits; return data; }
    const out = Uint8Array.from(data);
    let pos = this._gap;
    while (pos < nbits) {
      out[pos >> 3] ^= 0x80 >> (pos & 7);
      this.flipped += 1;
      pos += 1 + this._next();
    }
    this._gap = pos - nbits;
    return out;
  }
}

/**
 * Simulated HC-12 link to a RoverModel. Implements the same interface as the
 * Web Serial link: open(), write(bytes), close(), onData(bytes), onClose(reason).
 *  - bytes are paced at `baud` (8N1 -> 10 bits/byte) in both directions,
 *  - each transmission gets the radio latency (`latencyMs` +- `jitterMs`),
 *  - bit errors are injected at `ber` in both directions,
 *  - received bytes are delivered in USB-serial-like chunks every `chunkMs`.
 */
export class VirtualLink {
  constructor({ baud = 9600, ber = 0, latencyMs = 22, jitterMs = 8, legacy = false, safeLink = false,
    seed = 1, timeScale = 1, chunkMs = 16, transparency = false, clock = null } = {}) {
    this.kind = 'virtual';
    this.baud = baud;
    this.latencyMs = latencyMs;
    this.jitterMs = jitterMs;
    this.chunkMs = chunkMs;
    this.transparency = transparency;
    this.clock = clock || (() => performance.now() / 1000);
    this.rng = mulberry32(seed * 7 + 3);
    this.up = new BitErrors(ber, this.rng);
    this.down = new BitErrors(ber, this.rng);
    this.model = new RoverModel({ seed, legacy, safeLink, timeScale, clock: this.clock });
    this.decoder = new StreamDecoder({ timeoutS: 0.05, transparency, clock: this.clock });
    this.name = legacy ? 'Virtual rover (legacy 2023)' : (safeLink ? 'Virtual rover (Mega safe link)' : 'Virtual rover (v2 Brain)');
    this.onData = null;
    this.onClose = null;
    this.isOpen = false;
    this._timers = new Set();
    this._upFree = 0;      // time the uplink is free again
    this._txQueue = [];    // [{due, bytes}] waiting for the downlink
    this._downBuf = [];    // bytes "on the air" towards the GS
    this._downFree = 0;
    this._lastPump = 0;
  }

  setBer(ber) { this.up.setBer(ber); this.down.setBer(ber); }

  async open() {
    this.isOpen = true;
    this._lastPump = this.clock();
    this._pumpTimer = setInterval(() => this._pump(), this.chunkMs);
  }

  _later(fn, ms) {
    const id = setTimeout(() => { this._timers.delete(id); if (this.isOpen) fn(); }, Math.max(0, ms));
    this._timers.add(id);
  }

  _latency() { return (this.latencyMs + (this.rng() * 2 - 1) * this.jitterMs) / 1000; }

  /** GS -> rover */
  write(bytes) {
    if (!this.isOpen) throw new Error('virtual link closed');
    const now = this.clock();
    const air = (bytes.length * 10) / this.baud;
    const start = Math.max(now, this._upFree);
    this._upFree = start + air;
    const arrive = this._upFree + this._latency();
    const data = this.up.apply(bytes);
    this._later(() => this._roverRx(data), (arrive - now) * 1000);
    return Promise.resolve();
  }

  _roverRx(data) {
    const now = this.clock();
    for (const fr of this.decoder.feed(data, now)) {
      Promise.resolve(this.model.handle(fr, now)).then((replies) => {
        for (const [delay, rsp] of replies) {
          this._later(() => this._roverTx(rsp), delay * 1000);
        }
      });
    }
  }

  /** rover -> GS: queue a frame on the downlink (paced in _pump). */
  _roverTx(frame) {
    const wire = frame.encode(this.transparency);
    const now = this.clock();
    const start = Math.max(now, this._downFree);
    this._downFree = start + (wire.length * 10) / this.baud;
    this._txQueue.push({ due: start + this._latency(), bytes: this.down.apply(wire) });
  }

  _pump() {
    if (!this.isOpen) return;
    const now = this.clock();
    // move due transmissions onto the air
    while (this._txQueue.length && this._txQueue[0].due <= now) {
      const { bytes } = this._txQueue.shift();
      for (const b of bytes) this._downBuf.push(b);
    }
    const budget = Math.max(1, Math.floor(((now - this._lastPump) * this.baud) / 10));
    this._lastPump = now;
    if (!this._downBuf.length) return;
    const n = Math.min(budget, this._downBuf.length);
    const chunk = Uint8Array.from(this._downBuf.splice(0, n));
    if (this.onData) this.onData(chunk);
  }

  async close() {
    if (!this.isOpen) return;
    this.isOpen = false;
    clearInterval(this._pumpTimer);
    for (const id of this._timers) clearTimeout(id);
    this._timers.clear();
    if (this.onClose) this.onClose('closed');
  }
}
