// SSP addresses, packet types and payload layouts -- ICD v2.0 §4, §6, §7.
//
// JavaScript mirror of ros2_ws/src/rover_core/rover_core/protocol.py.
// Every payload class offers pack() -> Uint8Array and static unpack(bytes).
// Field names are identical to the Python dataclasses (snake_case) so the
// Python-generated cross-check vectors can be compared field by field.
// All multi-byte fields are little-endian.

import { toHex } from './ssp.js';

export const Addr = Object.freeze({
  ROVER: 0x01, MOBILITY: 0x02, PAYLOAD: 0x03, ARM: 0x04, GROUND: 0x0B, BROADCAST: 0xFF,
});

/** Packet types. */
export const PT = Object.freeze({
  PING: 0x00, ACK: 0x01, NACK: 0x02, GET_IMAGE: 0x08, IMAGE_CHUNK: 0x09, CAP_IMAGE: 0x0C,
  GET_TLM: 0x0F, START_SCAN: 0x16, STOP_SCAN: 0x1C, COLLECT_OBJECT: 0x20,
  SET_VELOCITY: 0x30, WHEEL_ODOM: 0x31, RANGES: 0x32, ENV_TLM: 0x33, IMU: 0x34,
  ARM_SET_JOINTS: 0x40, ARM_SET_POSE: 0x41, ARM_STATE: 0x42, HEARTBEAT: 0x50,
  ESTOP: 0x51, CLEAR_ESTOP: 0x52, SET_MODE: 0x53, DRIVE: 0x60,
});

export const LEGACY_TYPES = new Set([PT.PING, PT.ACK, PT.GET_IMAGE, PT.CAP_IMAGE, PT.GET_TLM,
  PT.START_SCAN, PT.STOP_SCAN, PT.COLLECT_OBJECT]);
/** requests whose positive response echoes the request type with LEN 0 */
export const ECHO_ACK_TYPES = new Set([PT.CAP_IMAGE, PT.START_SCAN, PT.STOP_SCAN, PT.COLLECT_OBJECT]);

export const Mode = Object.freeze({ IDLE: 0, TELEOP: 1, AUTONOMOUS: 2, SAFE: 3 });
export const NackReason = Object.freeze({
  BAD_LENGTH: 1, UNKNOWN_TYPE: 2, BUSY: 3, NOT_PERMITTED: 4, HW_FAULT: 5,
});
export const Fault = Object.freeze({
  ESTOP: 1 << 0, CMD_TIMEOUT: 1 << 1, LOW_BATTERY: 1 << 2, ENCODER: 1 << 3, SAFE_MODE: 1 << 4,
});
export const ArmPose = Object.freeze({
  HOME: 0, STOW: 1, READY: 2, REACH_GROUND: 3, GRIP_OPEN: 4, GRIP_CLOSE: 5, LIFT_SAMPLE: 6, DEPOSIT: 7,
});

/** wheel order used in WHEEL_ODOM / motor arrays */
export const WHEELS = ['left_front', 'left_mid', 'left_rear', 'right_front', 'right_mid', 'right_rear'];
/** ultrasonic order used in RANGES */
export const ULTRASONICS = ['front_left', 'front_center', 'front_right', 'left', 'right', 'rear'];
export const NO_ECHO = 0xFFFF;
export const IMAGE_CHUNK_PAYLOAD = 200;
export const LEGACY_IMAGE = Object.freeze({ width: 100, height: 100, chunks: 50, chunkSize: 200 });

// ----------------------------------------------------- struct (Python-style)
const CODES = {
  B: { size: 1, get: 'getUint8', set: 'setUint8', min: 0, max: 0xFF, name: 'ubyte' },
  b: { size: 1, get: 'getInt8', set: 'setInt8', min: -0x80, max: 0x7F, name: 'byte' },
  H: { size: 2, get: 'getUint16', set: 'setUint16', min: 0, max: 0xFFFF, name: 'ushort' },
  h: { size: 2, get: 'getInt16', set: 'setInt16', min: -0x8000, max: 0x7FFF, name: 'short' },
  I: { size: 4, get: 'getUint32', set: 'setUint32', min: 0, max: 0xFFFFFFFF, name: 'uint' },
  i: { size: 4, get: 'getInt32', set: 'setInt32', min: -0x80000000, max: 0x7FFFFFFF, name: 'int' },
};

const fmtCache = new Map();
function parseFmt(fmt) {
  let codes = fmtCache.get(fmt);
  if (codes) return codes;
  if (fmt[0] !== '<') throw new Error(`only little-endian '<' formats are supported: ${fmt}`);
  codes = [];
  let num = '';
  for (const c of fmt.slice(1)) {
    if (c === ' ') continue;
    if (c >= '0' && c <= '9') { num += c; continue; }
    if (!CODES[c]) throw new Error(`unsupported struct code '${c}' in ${fmt}`);
    const n = num ? parseInt(num, 10) : 1;
    num = '';
    for (let k = 0; k < n; k++) codes.push(c);
  }
  fmtCache.set(fmt, codes);
  return codes;
}

export function structSize(fmt) {
  return parseFmt(fmt).reduce((s, c) => s + CODES[c].size, 0);
}

export function structPack(fmt, values) {
  const codes = parseFmt(fmt);
  if (values.length !== codes.length) {
    throw new Error(`pack expected ${codes.length} items for packing (got ${values.length})`);
  }
  const out = new Uint8Array(structSize(fmt));
  const dv = new DataView(out.buffer);
  let off = 0;
  codes.forEach((c, k) => {
    const spec = CODES[c];
    const v = values[k];
    if (!Number.isInteger(v)) throw new TypeError(`required argument is not an integer: ${v}`);
    if (v < spec.min || v > spec.max) {
      throw new RangeError(`'${c}' format requires ${spec.min} <= number <= ${spec.max} (got ${v})`);
    }
    if (spec.size === 1) dv[spec.set](off, v);
    else dv[spec.set](off, v, true);
    off += spec.size;
  });
  return out;
}

export function structUnpack(fmt, data) {
  const codes = parseFmt(fmt);
  const size = structSize(fmt);
  if (data.length !== size) throw new Error(`unpack requires a buffer of ${size} bytes (got ${data.length})`);
  const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const out = [];
  let off = 0;
  for (const c of codes) {
    const spec = CODES[c];
    out.push(spec.size === 1 ? dv[spec.get](off) : dv[spec.get](off, true));
    off += spec.size;
  }
  return out;
}

// ------------------------------------------------------------ Python helpers
/** Python 3 round(): round half to even. */
export function pyRound(x) {
  const r = (Math.abs(x % 1) === 0.5) ? 2 * Math.round(x / 2) : Math.round(x);
  return r === 0 ? 0 : r; // no negative zero (Python ints have none)
}
export const clampI16 = (x) => Math.max(-32768, Math.min(32767, pyRound(x)));
export const clampU16 = (x) => Math.max(0, Math.min(65535, pyRound(x)));
export const clampI32 = (x) => Math.max(-(2 ** 31), Math.min(2 ** 31 - 1, pyRound(x)));

// ----------------------------------------------------------- payload structs
/**
 * Fixed-layout payload described by FMT and FIELDS ([name, count, default]).
 * count > 1 means a list field (e.g. WheelOdom.ticks).
 */
class Struct {
  static FMT = '<';
  static FIELDS = [];

  constructor(init = {}) {
    for (const [name, count, def] of this.constructor.FIELDS) {
      let v = init[name];
      if (v === undefined) v = (typeof def === 'function') ? def() : def;
      this[name] = count > 1 ? Array.from(v) : v;
    }
  }

  values() {
    const vals = [];
    for (const [name, count] of this.constructor.FIELDS) {
      if (count > 1) vals.push(...this[name]);
      else vals.push(this[name]);
    }
    return vals;
  }

  pack() { return structPack(this.constructor.FMT, this.values()); }

  static size() { return structSize(this.FMT); }

  static checkLen(data) {
    if (data.length !== this.size()) {
      throw new Error(`${this.name}: expected ${this.size()} bytes, got ${data.length}`);
    }
  }

  static unpack(data) {
    this.checkLen(data);
    const v = structUnpack(this.FMT, data);
    const init = {};
    let k = 0;
    for (const [name, count] of this.FIELDS) {
      if (count > 1) { init[name] = v.slice(k, k + count); k += count; } else { init[name] = v[k++]; }
    }
    return new this(init);
  }

  asDict() {
    const o = {};
    for (const [name, count] of this.constructor.FIELDS) o[name] = count > 1 ? [...this[name]] : this[name];
    return o;
  }

  toJSON() { return this.asDict(); }

  equals(other) {
    return other instanceof this.constructor &&
      JSON.stringify(this.values()) === JSON.stringify(other.values());
  }
}

export class SetVelocity extends Struct {
  static FMT = '<hhH';
  static FIELDS = [['v_mm_s', 1, 0], ['w_mrad_s', 1, 0], ['timeout_ms', 1, 500]];

  static fromSI(v, w, timeoutS = 0.5) {
    return new SetVelocity({
      v_mm_s: clampI16(v * 1000.0), w_mrad_s: clampI16(w * 1000.0),
      timeout_ms: Math.max(0, Math.min(65535, Math.trunc(timeoutS * 1000))),
    });
  }
}

export class WheelOdom extends Struct {
  static FMT = '<I6i';
  static FIELDS = [['t_ms', 1, 0], ['ticks', 6, () => [0, 0, 0, 0, 0, 0]]];
}

export class Ranges {
  constructor(init = {}) {
    this.range_mm = Array.from(init.range_mm ?? [NO_ECHO, NO_ECHO, NO_ECHO, NO_ECHO, NO_ECHO, NO_ECHO]);
  }

  pack() { return structPack(`<B${this.range_mm.length}H`, [this.range_mm.length, ...this.range_mm]); }

  static unpack(data) {
    if (!data.length) throw new Error('Ranges: empty');
    const n = data[0];
    if (data.length !== 1 + 2 * n) throw new Error(`Ranges: expected ${1 + 2 * n} bytes, got ${data.length}`);
    return new Ranges({ range_mm: structUnpack(`<${n}H`, data.subarray(1)) });
  }

  meters() { return this.range_mm.map((r) => (r === NO_ECHO ? Infinity : r / 1000.0)); }

  asDict() { return { range_mm: [...this.range_mm] }; }

  toJSON() { return this.asDict(); }
}

export class EnvTlm extends Struct {
  static FMT = '<hHIhHHHHBB'; // 20 bytes
  static FIELDS = [['dht_temp_cC', 1, 0], ['dht_rh_cP', 1, 0], ['bmp_pressure_Pa', 1, 0],
    ['bmp_temp_cC', 1, 0], ['mq2_raw', 1, 0], ['mq4_raw', 1, 0], ['soil_raw', 1, 0],
    ['rain_raw', 1, 0], ['proximity', 1, 0], ['status', 1, 0]];
}

export class Imu extends Struct {
  static FMT = '<I8h'; // 20 bytes
  static FIELDS = [['t_ms', 1, 0], ['ax', 1, 0], ['ay', 1, 0], ['az', 1, 0], ['gx', 1, 0],
    ['gy', 1, 0], ['gz', 1, 0], ['roll_cdeg', 1, 0], ['pitch_cdeg', 1, 0]];
}

export class ArmSetJoints extends Struct {
  static FMT = '<6BH';
  static FIELDS = [['deg', 6, () => [90, 90, 90, 90, 90, 10]], ['duration_ms', 1, 1000]];
}

export class ArmState extends Struct {
  static FMT = '<7B';
  static FIELDS = [['deg', 6, () => [90, 90, 90, 90, 90, 90]], ['moving', 1, 0]];
}

export class Heartbeat extends Struct {
  static FMT = '<IBB';
  static FIELDS = [['uptime_ms', 1, 0], ['mode', 1, 0], ['fault_flags', 1, 0]];
}

export class Drive extends Struct {
  static FMT = '<bbH';
  static FIELDS = [['v_pct', 1, 0], ['w_pct', 1, 0], ['duration_ms', 1, 500]];
}

export class Nack extends Struct {
  static FMT = '<BB';
  static FIELDS = [['rejected_type', 1, 0], ['reason', 1, 0]];
}

export class ImageChunkHeader extends Struct {
  static FMT = '<HHHHB'; // 9 bytes
  static FIELDS = [['seq', 1, 0], ['total', 1, 0], ['width', 1, 0], ['height', 1, 0], ['fmt', 1, 0]];

  /** Like the Python version: parses data[:9] (payload may follow). */
  static unpack(data) {
    const n = this.size();
    const v = structUnpack(this.FMT, data.subarray(0, n));
    return new ImageChunkHeader({ seq: v[0], total: v[1], width: v[2], height: v[3], fmt: v[4] });
  }
}

/** 7-byte telemetry of the 2023 Arduino Mega firmware. */
export class TlmLegacy extends Struct {
  static FMT = '<7B';
  static FIELDS = [['ir', 1, 0], ['gas_hi', 1, 0], ['gas_lo', 1, 0], ['us1_cm', 1, 0],
    ['us2_cm', 1, 0], ['temp_c', 1, 0], ['rh_pct', 1, 0]];

  get gas_raw() { return (this.gas_hi << 8) | this.gas_lo; }
}

/** 56-byte GET_TLM response (ICD §7.1). */
export class TlmV2 extends Struct {
  static FMT = '<BBBHI hHI hhh hhh hhh HHHH B ii h H'.replace(/ /g, '');
  static FIELDS = [['tlm_version', 1, 2], ['mode', 1, 0], ['fault_flags', 1, 0],
    ['battery_mV', 1, 0], ['uptime_s', 1, 0], ['temperature_cC', 1, 0], ['humidity_cP', 1, 0],
    ['pressure_Pa', 1, 0], ['roll_cdeg', 1, 0], ['pitch_cdeg', 1, 0], ['yaw_cdeg', 1, 0],
    ['ax', 1, 0], ['ay', 1, 0], ['az', 1, 0], ['gx', 1, 0], ['gy', 1, 0], ['gz', 1, 0],
    ['mq2_raw', 1, 0], ['mq4_raw', 1, 0], ['soil_raw', 1, 0], ['rain_raw', 1, 0],
    ['proximity', 1, 0], ['x_mm', 1, 0], ['y_mm', 1, 0], ['heading_cdeg', 1, 0],
    ['range_min_mm', 1, NO_ECHO]];
}

export const PAYLOAD_CLASSES = {
  SetVelocity, WheelOdom, Ranges, EnvTlm, Imu, ArmSetJoints, ArmState, Heartbeat, Drive, Nack,
  ImageChunkHeader, TlmLegacy, TlmV2,
};

// ------------------------------------------------------ length plausibility
const range = (a, b) => Array.from({ length: b - a }, (_, i) => a + i);

/** allowed DATA lengths per packet type (union of request and response
 *  directions, legacy included). Types not listed accept any length. */
export const ALLOWED_LEN = new Map([
  [PT.PING, new Set([0])], [PT.ACK, new Set([0, 1])], [PT.NACK, new Set([2])],
  [PT.GET_IMAGE, new Set([0, 1, 200])], [PT.IMAGE_CHUNK, new Set(range(9, 210))],
  [PT.CAP_IMAGE, new Set([0])], [PT.GET_TLM, null], // legacy brains sent 7/17/68 B
  [PT.START_SCAN, new Set([0])], [PT.STOP_SCAN, new Set([0])], [PT.COLLECT_OBJECT, new Set([0])],
  [PT.SET_VELOCITY, new Set([6])], [PT.WHEEL_ODOM, new Set([28])],
  [PT.RANGES, new Set(range(0, 7).map((n) => 1 + 2 * n))],
  [PT.ENV_TLM, new Set([20])], [PT.IMU, new Set([20])], [PT.ARM_SET_JOINTS, new Set([8])],
  [PT.ARM_SET_POSE, new Set([1])], [PT.ARM_STATE, new Set([7])], [PT.HEARTBEAT, new Set([6])],
  [PT.ESTOP, new Set([0])], [PT.CLEAR_ESTOP, new Set([0])], [PT.SET_MODE, new Set([1])],
  [PT.DRIVE, new Set([4])],
]);

/** Length plausibility check for StreamDecoder (ICD §3 step 5a).
 *  strict=true additionally rejects unknown packet types. */
export function plausibleLength(ptype, n, strict = false) {
  if (!ALLOWED_LEN.has(ptype)) return !strict;
  const allowed = ALLOWED_LEN.get(ptype);
  return allowed === null ? true : allowed.has(n);
}

/** Decode a GET_TLM response of either generation. */
export function decodeTlm(data) {
  if (data.length === TlmLegacy.size()) return TlmLegacy.unpack(data);
  if (data.length === TlmV2.size() && data[0] === 2) return TlmV2.unpack(data);
  throw new Error(`unknown telemetry layout (${data.length} bytes)`);
}

// ================================================================ GS helpers
// (not part of the Python reference: names and human-readable decoding)

const invert = (o) => Object.fromEntries(Object.entries(o).map(([k, v]) => [v, k]));
const PT_NAMES = invert(PT);
const ADDR_NAMES = invert(Addr);
const MODE_NAMES = invert(Mode);
const NACK_NAMES = invert(NackReason);
const POSE_NAMES = invert(ArmPose);

const hex2 = (v) => '0x' + (v < 16 ? '0' : '') + v.toString(16).toUpperCase();

export const typeName = (t) => PT_NAMES[t] ?? `TYPE_${hex2(t)}`;
export const addrName = (a) => ADDR_NAMES[a] ?? hex2(a);
export const modeName = (m) => MODE_NAMES[m] ?? `MODE_${m}`;
export const nackReasonName = (r) => NACK_NAMES[r] ?? `REASON_${r}`;
export const FAULT_LABELS = [
  [Fault.ESTOP, 'ESTOP', 'E-stop latched'],
  [Fault.CMD_TIMEOUT, 'CMD_TIMEOUT', 'Command timeout'],
  [Fault.LOW_BATTERY, 'LOW_BATTERY', 'Low battery (< 10.8 V)'],
  [Fault.ENCODER, 'ENCODER', 'Encoder fault'],
  [Fault.SAFE_MODE, 'SAFE_LINK', 'HC-12 safe-mode link'],
];
export function faultNames(flags) {
  return FAULT_LABELS.filter(([bit]) => flags & bit).map(([, n]) => n);
}

/** Normalised telemetry sample in engineering units (shared with gs_cli.py CSV). */
export function normalizeTlm(tlm, t = Date.now()) {
  const s = { t, tlm_version: null, mode: null, fault_flags: null, battery_V: null, uptime_s: null,
    temperature_C: null, humidity_pct: null, pressure_hPa: null, roll_deg: null, pitch_deg: null,
    yaw_deg: null, ax_mps2: null, ay_mps2: null, az_mps2: null, gx_radps: null, gy_radps: null,
    gz_radps: null, mq2_raw: null, mq4_raw: null, soil_raw: null, rain_raw: null, proximity: null,
    x_m: null, y_m: null, heading_deg: null, range_min_m: null, ir: null, gas_raw: null,
    us1_cm: null, us2_cm: null };
  if (tlm instanceof TlmV2) {
    Object.assign(s, {
      tlm_version: tlm.tlm_version, mode: tlm.mode, fault_flags: tlm.fault_flags,
      battery_V: tlm.battery_mV / 1000, uptime_s: tlm.uptime_s,
      temperature_C: tlm.temperature_cC / 100, humidity_pct: tlm.humidity_cP / 100,
      pressure_hPa: tlm.pressure_Pa / 100, roll_deg: tlm.roll_cdeg / 100,
      pitch_deg: tlm.pitch_cdeg / 100, yaw_deg: tlm.yaw_cdeg / 100,
      ax_mps2: tlm.ax / 1000, ay_mps2: tlm.ay / 1000, az_mps2: tlm.az / 1000,
      gx_radps: tlm.gx / 1000, gy_radps: tlm.gy / 1000, gz_radps: tlm.gz / 1000,
      mq2_raw: tlm.mq2_raw, mq4_raw: tlm.mq4_raw, soil_raw: tlm.soil_raw, rain_raw: tlm.rain_raw,
      proximity: tlm.proximity, x_m: tlm.x_mm / 1000, y_m: tlm.y_mm / 1000,
      heading_deg: tlm.heading_cdeg / 100,
      range_min_m: tlm.range_min_mm === NO_ECHO ? null : tlm.range_min_mm / 1000,
    });
  } else if (tlm instanceof TlmLegacy) {
    const us = [tlm.us1_cm, tlm.us2_cm].filter((v) => v > 0);
    Object.assign(s, {
      tlm_version: 1, temperature_C: tlm.temp_c, humidity_pct: tlm.rh_pct,
      ir: tlm.ir, gas_raw: tlm.gas_raw, us1_cm: tlm.us1_cm, us2_cm: tlm.us2_cm,
      range_min_m: us.length ? Math.min(...us) / 100 : null,
    });
  }
  return s;
}

/** CSV column order (identical to gs_cli.py CSV_COLUMNS). */
export const CSV_COLUMNS = ['time_iso', 'tlm_version', 'mode', 'fault_flags', 'battery_V', 'uptime_s',
  'temperature_C', 'humidity_pct', 'pressure_hPa', 'roll_deg', 'pitch_deg', 'yaw_deg',
  'ax_mps2', 'ay_mps2', 'az_mps2', 'gx_radps', 'gy_radps', 'gz_radps',
  'mq2_raw', 'mq4_raw', 'soil_raw', 'rain_raw', 'proximity', 'x_m', 'y_m', 'heading_deg',
  'range_min_m', 'ir', 'gas_raw', 'us1_cm', 'us2_cm'];

export function samplesToCsv(samples) {
  const lines = [CSV_COLUMNS.join(',')];
  for (const s of samples) {
    lines.push(CSV_COLUMNS.map((c) => {
      if (c === 'time_iso') return new Date(s.t).toISOString();
      const v = s[c];
      if (v == null) return '';
      return Number.isInteger(v) ? String(v) : String(Math.round(v * 1e6) / 1e6);
    }).join(','));
  }
  return lines.join('\r\n') + '\r\n';
}

const f = (v, d = 1) => (v == null ? '-' : Number(v).toFixed(d));

function describeTlm(data) {
  try {
    const t = decodeTlm(data);
    if (t instanceof TlmLegacy) {
      return `TLM legacy: IR=${t.ir} gas=${t.gas_raw} US1=${t.us1_cm}cm US2=${t.us2_cm}cm ` +
        `T=${t.temp_c}°C RH=${t.rh_pct}%`;
    }
    const faults = faultNames(t.fault_flags);
    return `TLM v2: ${modeName(t.mode)} batt=${f(t.battery_mV / 1000, 2)}V ` +
      `T=${f(t.temperature_cC / 100)}°C RH=${f(t.humidity_cP / 100)}% p=${f(t.pressure_Pa / 100)}hPa ` +
      `pos=(${f(t.x_mm / 1000, 2)},${f(t.y_mm / 1000, 2)})m hdg=${f(t.heading_cdeg / 100)}° ` +
      `rmin=${t.range_min_mm === NO_ECHO ? 'none' : f(t.range_min_mm / 1000, 2) + 'm'}` +
      (faults.length ? ` faults=${faults.join('|')}` : '');
  } catch (e) {
    return `TLM ${data.length} B (unknown layout)`;
  }
}

/** One-line human-readable meaning of a frame (for logs). */
export function describeFrame(fr) {
  const d = fr.data;
  const name = typeName(fr.type);
  try {
    switch (fr.type) {
      case PT.PING: return 'PING';
      case PT.ACK: return d.length ? `ACK of ${typeName(d[0])}` : 'ACK';
      case PT.NACK: {
        if (d.length !== 2) break;
        const n = Nack.unpack(d);
        return `NACK of ${typeName(n.rejected_type)}: ${nackReasonName(n.reason)}`;
      }
      case PT.GET_IMAGE:
        if (d.length === 0) return 'GET_IMAGE (legacy 100x100 gray)';
        if (d.length === 1) return `GET_IMAGE v2 quality=${d[0]}`;
        return `legacy image chunk (${d.length} B)`;
      case PT.IMAGE_CHUNK: {
        if (d.length < 9) break;
        const h = ImageChunkHeader.unpack(d);
        return `IMAGE_CHUNK ${h.seq + 1}/${h.total} ${h.width}x${h.height} ` +
          `${h.fmt === 1 ? 'jpeg' : 'gray8'} (${d.length - 9} B)`;
      }
      case PT.GET_TLM: return d.length ? describeTlm(d) : 'GET_TLM';
      case PT.SET_MODE: return d.length === 1 ? `SET_MODE ${modeName(d[0])}` : name;
      case PT.DRIVE: {
        if (d.length !== 4) break;
        const v = Drive.unpack(d);
        return `DRIVE v=${v.v_pct}% w=${v.w_pct}% for ${v.duration_ms} ms`;
      }
      case PT.SET_VELOCITY: {
        if (d.length !== 6) break;
        const v = SetVelocity.unpack(d);
        return `SET_VELOCITY v=${v.v_mm_s}mm/s w=${v.w_mrad_s}mrad/s timeout=${v.timeout_ms}ms`;
      }
      case PT.HEARTBEAT: {
        if (d.length !== 6) break;
        const h = Heartbeat.unpack(d);
        return `HEARTBEAT up=${(h.uptime_ms / 1000).toFixed(1)}s ${modeName(h.mode)} faults=${hex2(h.fault_flags)}`;
      }
      case PT.ARM_SET_POSE: return d.length === 1 ? `ARM_SET_POSE ${POSE_NAMES[d[0]] ?? d[0]}` : name;
      default: break;
    }
  } catch (e) { /* fall through */ }
  return d.length ? `${name} (${d.length} B)` : name;
}

/** Frame bytes as hex (re-exported convenience for UI code). */
export const hexOf = (bytes, sep = ' ') => toHex(bytes, sep);
