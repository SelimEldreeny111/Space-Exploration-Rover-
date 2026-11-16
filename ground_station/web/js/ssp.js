// MISSION X Simple Serial Protocol (SSP) framing -- ICD v2.0 §2-§3.
//
// JavaScript mirror of ros2_ws/src/rover_core/rover_core/{crc,ssp}.py. The
// receiver state machine, statistics and error handling are line-for-line
// equivalent to the Python reference; test/crosscheck.test.mjs feeds the same
// corrupted streams to both and requires identical output.
//
// Wire format (transparency OFF, wire-compatible with the 2023 LabVIEW GS):
//
//     C0 | DEST | SRC | TYPE | LEN | DATA[LEN] | CRC_L | CRC_H | C0
//
// CRC-16/XMODEM over DEST..DATA, transmitted low byte first.

export const FEND = 0xC0;
export const FESC = 0xDB;
export const TFEND = 0xDC;
export const TFESC = 0xDD;
export const MAX_DATA = 255;
export const HEADER_LEN = 4; // DEST SRC TYPE LEN

export class SSPError extends Error {
  constructor(message) {
    super(message);
    this.name = 'SSPError';
  }
}

// ------------------------------------------------------------------ helpers

/** Convert Uint8Array / ArrayBuffer / array of ints to a fresh Uint8Array. */
export function toBytes(x) {
  if (x == null) return new Uint8Array(0);
  if (x instanceof Uint8Array) return new Uint8Array(x);
  if (x instanceof ArrayBuffer) return new Uint8Array(x.slice(0));
  if (ArrayBuffer.isView(x)) return new Uint8Array(x.buffer.slice(x.byteOffset, x.byteOffset + x.byteLength));
  if (typeof x === 'string') throw new TypeError('toBytes: strings are not bytes (use fromHex or TextEncoder)');
  const arr = Array.from(x);
  for (const v of arr) {
    if (!Number.isInteger(v) || v < 0 || v > 255) throw new RangeError(`byte must be in range(0, 256), got ${v}`);
  }
  return Uint8Array.from(arr);
}

/** "C0 01 0B" / "c0010b" -> Uint8Array */
export function fromHex(s) {
  const clean = String(s).replace(/[\s:,-]/g, '');
  if (clean.length % 2) throw new SSPError(`odd-length hex string: ${s}`);
  const out = new Uint8Array(clean.length / 2);
  for (let i = 0; i < out.length; i++) {
    const v = parseInt(clean.substr(2 * i, 2), 16);
    if (Number.isNaN(v)) throw new SSPError(`bad hex: ${s}`);
    out[i] = v;
  }
  return out;
}

/** Uint8Array -> "C0 01 0B" (upper-case, separator configurable) */
export function toHex(bytes, sep = ' ') {
  let s = '';
  for (let i = 0; i < bytes.length; i++) {
    if (i && sep) s += sep;
    s += (bytes[i] < 16 ? '0' : '') + bytes[i].toString(16).toUpperCase();
  }
  return s;
}

export function bytesEqual(a, b) {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}

// ---------------------------------------------------------------------- CRC
// CRC-16/XMODEM: poly 0x1021, init 0x0000, no reflection, xorout 0x0000.

function makeTable() {
  const table = new Uint16Array(256);
  for (let i = 0; i < 256; i++) {
    let crc = i << 8;
    for (let k = 0; k < 8; k++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
      crc &= 0xFFFF;
    }
    table[i] = crc;
  }
  return table;
}

export const CRC16_TABLE = makeTable();

/** CRC-16/XMODEM of `data` (optionally continuing from `crc`). */
export function crc16Xmodem(data, crc = 0x0000) {
  const tbl = CRC16_TABLE;
  for (let i = 0; i < data.length; i++) {
    crc = ((crc << 8) & 0xFFFF) ^ tbl[((crc >> 8) ^ data[i]) & 0xFF];
  }
  return crc;
}

// -------------------------------------------------------------------- Frame

function checkByte(name, v) {
  if (!Number.isInteger(v) || v < 0 || v > 0xFF) throw new SSPError(`${name}=${v} out of range 0..255`);
}

export class Frame {
  constructor(dest, src, type, data = new Uint8Array(0)) {
    checkByte('dest', dest);
    checkByte('src', src);
    checkByte('type', type);
    const d = toBytes(data);
    if (d.length > MAX_DATA) throw new SSPError(`DATA length ${d.length} > ${MAX_DATA}`);
    this.dest = dest;
    this.src = src;
    this.type = type;
    this.data = d;
    Object.freeze(this);
  }

  /** DEST..DATA (the CRC-protected part). */
  get body() {
    const b = new Uint8Array(HEADER_LEN + this.data.length);
    b[0] = this.dest; b[1] = this.src; b[2] = this.type; b[3] = this.data.length;
    b.set(this.data, HEADER_LEN);
    return b;
  }

  get crc() { return crc16Xmodem(this.body); }

  encode(transparency = false) {
    return encode(this.dest, this.src, this.type, this.data, transparency);
  }

  /** Build a response with swapped addresses (ICD §4). */
  reply(ptype, data = new Uint8Array(0), ownAddr = null) {
    const src = ownAddr == null ? this.dest : ownAddr;
    return new Frame(this.src, src, ptype, data);
  }

  equals(other) {
    return other instanceof Frame && other.dest === this.dest && other.src === this.src &&
      other.type === this.type && bytesEqual(other.data, this.data);
  }

  toJSON() {
    return { dest: this.dest, src: this.src, type: this.type, data: toHex(this.data, '') };
  }

  static fromJSON(o) {
    return new Frame(o.dest, o.src, o.type, fromHex(o.data || ''));
  }

  toString() {
    const h2 = (v) => '0x' + (v < 16 ? '0' : '') + v.toString(16).toUpperCase();
    return `Frame(dest=${h2(this.dest)}, src=${h2(this.src)}, type=${h2(this.type)}, ` +
      `len=${this.data.length}, data=${toHex(this.data).toLowerCase()})`;
  }
}

function escapeBytes(payload) {
  const out = [];
  for (const b of payload) {
    if (b === FEND) out.push(FESC, TFEND);
    else if (b === FESC) out.push(FESC, TFESC);
    else out.push(b);
  }
  return Uint8Array.from(out);
}

/** Encode one SSP frame (with both FEND flags). */
export function encode(dest, src, ptype, data = new Uint8Array(0), transparency = false) {
  const d = toBytes(data);
  if (d.length > MAX_DATA) throw new SSPError(`DATA length ${d.length} > ${MAX_DATA}`);
  for (const [name, v] of [['dest', dest], ['src', src], ['type', ptype]]) {
    if (!Number.isInteger(v) || v < 0 || v > 0xFF) throw new SSPError(`${name}=${v} out of range`);
  }
  const n = d.length;
  let inner = new Uint8Array(HEADER_LEN + n + 2);
  inner[0] = dest; inner[1] = src; inner[2] = ptype; inner[3] = n;
  inner.set(d, HEADER_LEN);
  const crc = crc16Xmodem(inner.subarray(0, HEADER_LEN + n));
  inner[HEADER_LEN + n] = crc & 0xFF;
  inner[HEADER_LEN + n + 1] = (crc >> 8) & 0xFF;
  if (transparency) inner = escapeBytes(inner);
  const out = new Uint8Array(inner.length + 2);
  out[0] = FEND;
  out.set(inner, 1);
  out[out.length - 1] = FEND;
  return out;
}

// ------------------------------------------------------------------ decoder

export class DecoderStats {
  constructor() {
    this.frames_ok = 0;
    this.crc_errors = 0;
    this.framing_errors = 0;
    this.escape_errors = 0;
    this.timeouts = 0;
    this.filtered = 0;        // valid frames not addressed to us
    this.bytes_in = 0;
    this.bytes_discarded = 0; // bytes seen while hunting for a flag
  }

  asDict() { return { ...this }; }
}

const defaultClock = () => ((typeof performance !== 'undefined' && performance.now)
  ? performance.now() / 1000 : Date.now() / 1000);

/**
 * Incremental SSP receiver (mirror of rover_core.ssp.StreamDecoder).
 *
 * Options:
 *   ownAddr      if given, frames whose DEST is neither ownAddr nor 0xFF are
 *                counted in stats.filtered and not returned (unless promiscuous)
 *   transparency SLIP escaping on/off (must match the sender)
 *   timeoutS     inter-byte timeout inside a frame in seconds (ICD §3 step 6),
 *                null disables it
 *   clock        monotonic clock in seconds, injectable for tests
 *   lengthCheck  optional f(type, len) -> bool; an implausible LEN for a known
 *                TYPE is a framing error as soon as the header is complete
 *   onError      optional f(kind, bytes) diagnostic hook, called whenever an
 *                error counter is incremented (kind: 'framing' | 'crc' |
 *                'timeout' | 'escape'); it does not influence decoding
 */
export class StreamDecoder {
  constructor({ ownAddr = null, transparency = false, timeoutS = 0.05, promiscuous = false,
    clock = defaultClock, lengthCheck = null, onError = null } = {}) {
    this.ownAddr = ownAddr;
    this.transparency = transparency;
    this.timeoutS = timeoutS;
    this.promiscuous = promiscuous;
    this.lengthCheck = lengthCheck;
    this.onError = onError;
    this._clock = clock;
    this.stats = new DecoderStats();
    this._inFrame = false;
    this._buf = [];
    this._esc = false;
    this._lastT = null;
    this._queue = [];
    this._qhead = 0;
  }

  // ---------------------------------------------------------------- public
  reset() {
    this._inFrame = false;
    this._buf = [];
    this._esc = false;
    this._queue = [];
    this._qhead = 0;
  }

  /** Feed received bytes; return the array of complete, CRC-valid frames. */
  feed(data, now = null) {
    const t = now == null ? this._clock() : now;
    if (this.timeoutS != null && this._lastT != null && this._inFrame &&
        this._buf.length && (t - this._lastT) > this.timeoutS) {
      this.stats.timeouts += 1;
      this._emit('timeout', this._buf);
      this._inFrame = false;
      this._buf = [];
      this._esc = false;
    }
    const bytes = (data instanceof Uint8Array) ? data : toBytes(data);
    if (bytes.length) this._lastT = t;
    this.stats.bytes_in += bytes.length;
    for (let i = 0; i < bytes.length; i++) this._queue.push(bytes[i]);
    const out = [];
    const slip = this.transparency;
    while (this._qhead < this._queue.length) {
      const b = this._queue[this._qhead++];
      const fr = slip ? this._stepSlip(b) : this._stepLen(b);
      if (fr !== null) out.push(fr);
    }
    this._queue = [];
    this._qhead = 0;
    return out;
  }

  // ------------------------------------------------------------- internals
  _emit(kind, bytes) {
    if (this.onError) {
      try { this.onError(kind, Uint8Array.from(bytes)); } catch (e) { /* diagnostics only */ }
    }
  }

  /** Validate DEST..CRC. Returns [frame, status] with status in
   *  {'ok', 'filtered', 'framing', 'crc'} and updates the statistics. */
  _check(body) {
    if (body.length < HEADER_LEN + 2 || body.length !== HEADER_LEN + body[3] + 2) {
      this.stats.framing_errors += 1;
      this._emit('framing', body);
      return [null, 'framing'];
    }
    const crcRx = body[body.length - 2] | (body[body.length - 1] << 8);
    if (crc16Xmodem(body.subarray(0, body.length - 2)) !== crcRx) {
      this.stats.crc_errors += 1;
      this._emit('crc', body);
      return [null, 'crc'];
    }
    const fr = new Frame(body[0], body[1], body[2], body.subarray(HEADER_LEN, body.length - 2));
    if (this.ownAddr != null && !this.promiscuous && fr.dest !== this.ownAddr && fr.dest !== 0xFF) {
      this.stats.filtered += 1;
      return [null, 'filtered'];
    }
    this.stats.frames_ok += 1;
    return [fr, 'ok'];
  }

  _accept(body) {
    return this._check(body)[0];
  }

  /** Push back the bytes after the failed opening flag (ICD §3 step 5). */
  _resync() {
    const raw = this._buf;
    this._buf = [];
    const idx = raw.indexOf(FEND);
    if (idx < 0) {
      this._inFrame = false;
      this.stats.bytes_discarded += raw.length;
      return;
    }
    this.stats.bytes_discarded += idx;
    this._inFrame = true;
    // re-process everything after that flag before the remaining input
    const rest = raw.slice(idx + 1);
    if (rest.length) this._queue.splice(this._qhead, 0, ...rest);
  }

  _stepLen(b) {
    if (!this._inFrame) {
      if (b === FEND) {
        this._inFrame = true;
        this._buf = [];
      } else {
        this.stats.bytes_discarded += 1;
      }
      return null;
    }
    if (b === FEND && this._buf.length < 3) {
      // A flag where DEST/SRC/TYPE is expected: idle flag, closing flag of
      // the previous frame, or a truncated header -> restart the header.
      // (LEN may legitimately be 0xC0 = 192, so position 3 is exempt.)
      if (this._buf.length) {
        this.stats.framing_errors += 1;
        this._emit('framing', this._buf);
        this._buf = [];
      }
      return null;
    }
    this._buf.push(b);
    const n = this._buf.length;
    if (n < HEADER_LEN) return null;
    if (n === HEADER_LEN && this.lengthCheck && !this.lengthCheck(this._buf[2], this._buf[3])) {
      this.stats.framing_errors += 1;
      this._emit('framing', this._buf);
      this._resync();
      return null;
    }
    const need = HEADER_LEN + this._buf[3] + 2;
    if (n <= need) return null;
    // n === need + 1 -> this byte must be the closing flag
    if (b !== FEND) {
      this.stats.framing_errors += 1;
      this._emit('framing', this._buf);
      this._resync();
      return null;
    }
    const [fr, status] = this._check(Uint8Array.from(this._buf.slice(0, -1)));
    if (status === 'crc') {
      // The opening flag may have been a 0xC0 inside an earlier frame's
      // payload: rescan the buffered bytes (closing flag included).
      this._resync();
      return null;
    }
    // ok / filtered: the closing flag doubles as the next opening flag
    this._buf = [];
    this._inFrame = true;
    return fr;
  }

  _stepSlip(b) {
    if (b === FEND) {
      let fr = null;
      if (this._inFrame && this._buf.length && !this._esc) {
        fr = this._accept(Uint8Array.from(this._buf));
      } else if (this._esc) {
        this.stats.escape_errors += 1;
        this._emit('escape', this._buf);
      }
      this._inFrame = true;
      this._buf = [];
      this._esc = false;
      return fr;
    }
    if (!this._inFrame) {
      this.stats.bytes_discarded += 1;
      return null;
    }
    if (this._esc) {
      this._esc = false;
      if (b === TFEND) {
        this._buf.push(FEND);
      } else if (b === TFESC) {
        this._buf.push(FESC);
      } else {
        this.stats.escape_errors += 1;
        this._emit('escape', this._buf);
        this._inFrame = false;
        this._buf = [];
      }
      return null;
    }
    if (b === FESC) {
      this._esc = true;
      return null;
    }
    this._buf.push(b);
    if (this._buf.length > HEADER_LEN + MAX_DATA + 2) {
      this.stats.framing_errors += 1;
      this._emit('framing', this._buf);
      this._inFrame = false;
      this._buf = [];
    }
    return null;
  }
}
