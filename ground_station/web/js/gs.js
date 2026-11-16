// Ground-station core (no DOM): link handling, SSP decoding, request queue,
// reply matching, telemetry history, image assembly, frame log, session I/O.
// Shared by the browser UI (app.js) and the Node tests.

import { Frame, StreamDecoder, toBytes, toHex } from './ssp.js';
import {
  Addr, Drive, ECHO_ACK_TYPES, ImageChunkHeader, IMAGE_CHUNK_PAYLOAD, LEGACY_IMAGE, Nack, PT, decodeTlm,
  describeFrame, normalizeTlm, plausibleLength, samplesToCsv, typeName,
} from './protocol.js';

/** wire size of one legacy image chunk frame: FEND + 4 + 200 + CRC 2 + FEND */
export const LEGACY_CHUNK_WIRE = 1 + 4 + LEGACY_IMAGE.chunkSize + 2 + 1;

/** 'done' if `fr` completes request `req`, 'nack' if it rejects it, else null.
 *  (One request is outstanding at a time -- half-duplex radio -- so replies
 *  are matched on type; same rule as gs_cli.classify.) */
export function classify(req, fr) {
  const rt = req.type; const d = fr.data;
  if (fr.type === PT.NACK && d.length === 2 && d[0] === rt) return 'nack';
  if (rt === PT.GET_TLM) return fr.type === PT.GET_TLM && d.length ? 'done' : null;
  if (ECHO_ACK_TYPES.has(rt)) return fr.type === rt && !d.length ? 'done' : null;
  if (fr.type === PT.ACK && (!d.length || d[0] === rt)) return 'done';
  return null;
}

// ----------------------------------------------------------------- images
/**
 * Collects legacy (type 0x08, 200 B) or v2 IMAGE_CHUNK frames.
 * Legacy chunks carry no sequence number; every legacy chunk frame is exactly
 * 208 bytes on the wire, so the bytes the decoder consumed between two valid
 * chunks tell how many chunks were lost in between (same logic as gs_cli.py).
 */
export class ImageAssembler {
  constructor(bytesAtRequest = 0, t0 = 0) {
    this.legacy = null;
    this.width = 0; this.height = 0; this.total = 0; this.fmt = 0;
    this.chunks = new Map();
    this.skipped = new Set();
    this._slot = 0;
    this._mark = bytesAtRequest;
    this.t0 = t0;
    this.bytes = 0;
  }

  noteOther(fr) { this._mark += fr.data.length + 8; }

  add(fr, bytesIn) {
    const d = fr.data;
    if (fr.type === PT.GET_IMAGE && d.length === LEGACY_IMAGE.chunkSize) {
      if (this.legacy !== true) {
        this.legacy = true; this.width = LEGACY_IMAGE.width; this.height = LEGACY_IMAGE.height;
        this.total = LEGACY_IMAGE.chunks; this.fmt = 0;
      }
      const gap = bytesIn - this._mark - LEGACY_CHUNK_WIRE;
      const lost = gap > LEGACY_CHUNK_WIRE / 2 ? Math.round(gap / LEGACY_CHUNK_WIRE) : 0;
      for (let k = 0; k < lost; k++) this.skipped.add(this._slot + k);
      this._slot += lost;
      if (this._slot < this.total) { this.chunks.set(this._slot, Uint8Array.from(d)); this.bytes += d.length; }
      this._slot += 1;
      this._mark = bytesIn;
      return true;
    }
    if (fr.type === PT.IMAGE_CHUNK && d.length >= ImageChunkHeader.size()) {
      const h = ImageChunkHeader.unpack(d);
      if (this.legacy !== false || h.total !== this.total || h.width !== this.width ||
          h.height !== this.height || h.fmt !== this.fmt) {
        this.legacy = false; this.total = h.total; this.width = h.width; this.height = h.height; this.fmt = h.fmt;
        this.chunks = new Map(); this.bytes = 0;
      }
      if (h.seq < h.total && !this.chunks.has(h.seq)) {
        const payload = d.slice(ImageChunkHeader.size());
        this.chunks.set(h.seq, payload);
        this.bytes += payload.length;
        this._slot = Math.max(this._slot, h.seq + 1);
      }
      this._mark = bytesIn;
      return true;
    }
    return false;
  }

  get complete() { return this.total > 0 && this.chunks.size >= this.total; }

  missing() {
    const out = [];
    for (let i = 0; i < this.total; i++) if (!this.chunks.has(i)) out.push(i);
    return out;
  }

  /** per-chunk state for the progress map: 'ok' | 'missing' | 'pending' */
  states(final = false) {
    return Array.from({ length: this.total }, (_, i) => {
      if (this.chunks.has(i)) return 'ok';
      if (final || i < this._slot || this.skipped.has(i)) return 'missing';
      return 'pending';
    });
  }

  result(t1 = 0, reason = 'complete') {
    const size = this.legacy ? LEGACY_IMAGE.chunkSize : IMAGE_CHUNK_PAYLOAD;
    let data;
    if (this.fmt === 0) {
      data = new Uint8Array(this.width * this.height);
      for (const [i, c] of this.chunks) {
        const off = i * size;
        if (off < data.length) data.set(c.subarray(0, Math.max(0, data.length - off)), off);
      }
    } else {
      const parts = [];
      for (let i = 0; i < this.total; i++) parts.push(this.chunks.get(i) || new Uint8Array(size));
      data = new Uint8Array(parts.reduce((s, p) => s + p.length, 0));
      let o = 0;
      for (const p of parts) { data.set(p, o); o += p.length; }
    }
    return {
      t: Date.now(), width: this.width, height: this.height, fmt: this.fmt, legacy: !!this.legacy,
      total: this.total, received: [...this.chunks.keys()].sort((a, b) => a - b), missing: this.missing(),
      complete: this.complete, data, elapsed: Math.max(0, t1 - this.t0), reason,
    };
  }
}

// ------------------------------------------------------------ base64 helpers
export function bytesToB64(bytes) {
  let s = '';
  for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
  return btoa(s);
}
export function b64ToBytes(b64) {
  const s = atob(b64);
  const out = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) out[i] = s.charCodeAt(i);
  return out;
}

// =========================================================== ground station
export class GroundStation {
  constructor(opts = {}) {
    this.opts = {
      ownAddr: Addr.GROUND, dest: Addr.ROVER, timeoutMs: 2000, ibtS: 0.1, transparency: false,
      historyMax: 7200, logMax: 3000, imageFirstMs: 5000, imageIdleMs: 2500, imagesMax: 12, ...opts,
    };
    this.clock = opts.clock || (() => performance.now() / 1000);
    this.listeners = new Map();
    this.link = null;
    this.history = [];
    this.log = [];
    this.images = [];
    this.lastTlm = null;
    this.lastTlmAt = null;
    this._pollTimer = null;
    this.pollIntervalS = 0;
    this._resetLinkState();
  }

  // ----------------------------------------------------------- events
  on(evt, fn) {
    if (!this.listeners.has(evt)) this.listeners.set(evt, new Set());
    this.listeners.get(evt).add(fn);
    return () => this.listeners.get(evt).delete(fn);
  }

  emit(evt, data) {
    const set = this.listeners.get(evt);
    if (!set) return;
    for (const fn of [...set]) {
      try { fn(data); } catch (e) { console.error(`listener for ${evt} failed`, e); }
    }
  }

  // ------------------------------------------------------------- link
  _resetLinkState() {
    this.decoder = new StreamDecoder({
      ownAddr: this.opts.ownAddr, transparency: this.opts.transparency, timeoutS: this.opts.ibtS,
      lengthCheck: (t, n) => plausibleLength(t, n), clock: this.clock,
      onError: (kind, bytes) => this._onDecoderError(kind, bytes),
    });
    this.counters = { tx_frames: 0, tx_bytes: 0, requests: 0, replies: 0, req_timeouts: 0, nacks: 0, retries: 0 };
    this.rtt = { last: null, min: null, max: null, sum: 0, n: 0 };
    this.queue = [];
    this.inflight = [];
    this.image = null;
  }

  get connected() { return !!(this.link && this.link.isOpen); }

  configure(opts) {
    Object.assign(this.opts, opts);
    this.decoder.transparency = this.opts.transparency;
    this.decoder.timeoutS = this.opts.ibtS;
  }

  async connect(link) {
    if (this.link) await this.disconnect();
    this._resetLinkState();
    this.link = link;
    link.onData = (bytes) => this._onBytes(bytes);
    link.onClose = (reason) => this._onClosed(reason);
    try {
      await link.open();
    } catch (e) {
      this.link = null;
      throw e;
    }
    this.connectedAt = Date.now();
    this.sys(`Connected to ${link.name}${link.baud ? ` @ ${link.baud} baud` : ''}`);
    this.emit('link', { connected: true, link });
    if (this.pollIntervalS) this.setAutoPoll(this.pollIntervalS);
  }

  async disconnect() {
    const link = this.link;
    if (!link) return;
    this.link = null;
    this._failAll('closed');
    try { await link.close(); } catch (e) { /* already gone */ }
    this.sys('Disconnected');
    this.emit('link', { connected: false });
  }

  _onClosed(reason) {
    if (!this.link) return;
    this.link = null;
    this._failAll('closed');
    this.sys(`Link closed: ${reason}`, 'warn');
    this.emit('link', { connected: false, reason });
  }

  _failAll(error) {
    for (const r of [...this.inflight, ...this.queue]) {
      clearTimeout(r.timer);
      r.resolve({ ok: false, error, type: r.type });
    }
    this.inflight = [];
    this.queue = [];
    if (this.image) this._finishImage('aborted');
  }

  // --------------------------------------------------------------- rx
  _onBytes(bytes) {
    const now = this.clock();
    for (let i = 0; i < bytes.length; i++) { // byte-wise: each frame is tagged with its stream position
      const frames = this.decoder.feed(bytes.subarray(i, i + 1), now);
      for (const fr of frames) this._onFrame(fr, now, this.decoder.stats.bytes_in);
    }
    this.emit('rx', bytes.length);
  }

  _onFrame(fr, t, bytesIn) {
    this._logFrame('RX', fr);
    const isChunk = (fr.type === PT.GET_IMAGE && fr.data.length === LEGACY_IMAGE.chunkSize) ||
      (fr.type === PT.IMAGE_CHUNK && fr.data.length >= ImageChunkHeader.size());
    if (isChunk) {
      if (!this.image) this._startImage(null);      // unsolicited image stream
      this.image.add(fr, bytesIn);
      this._armImageTimer(this.opts.imageIdleMs);
      this.emit('image-progress', this._imageProgress());
      if (this.image.complete) this._finishImage('complete');
      return;
    }
    if (this.image) this.image.noteOther(fr);
    if (fr.type === PT.GET_TLM && fr.data.length) this._onTlm(fr);
    for (const req of this.inflight) {
      if (req.kind === 'image') {
        if (fr.type === PT.NACK && fr.data.length === 2 && fr.data[0] === PT.GET_IMAGE) {
          this.counters.nacks += 1;
          this._finishImage('nack', Nack.unpack(fr.data));
          return;
        }
        continue;
      }
      const verdict = classify(req.frame, fr);
      if (verdict) { this._resolve(req, verdict, fr, t); return; }
    }
    this.emit('unsolicited', fr);
  }

  _onTlm(fr) {
    let tlm;
    try { tlm = decodeTlm(fr.data); } catch (e) {
      this.sys(`GET_TLM reply with unknown layout (${fr.data.length} B)`, 'warn');
      return;
    }
    const sample = normalizeTlm(tlm, Date.now());
    this.lastTlm = tlm;
    this.lastTlmAt = Date.now();
    this.history.push(sample);
    if (this.history.length > this.opts.historyMax) this.history.splice(0, this.history.length - this.opts.historyMax);
    this.emit('tlm', { tlm, sample });
  }

  _onDecoderError(kind, bytes) {
    this._push({ t: Date.now(), dir: 'ERR', name: kind.toUpperCase(), len: bytes.length, hex: toHex(bytes),
      text: { crc: 'CRC mismatch, frame dropped', framing: 'framing error, resynchronising',
        timeout: 'inter-byte timeout, partial frame dropped', escape: 'bad SLIP escape' }[kind] || kind });
  }

  // --------------------------------------------------------------- tx
  _write(frame) {
    const wire = frame.encode(this.opts.transparency);
    this.counters.tx_frames += 1;
    this.counters.tx_bytes += wire.length;
    this._logFrame('TX', frame);
    const p = this.link.write(wire);
    if (p && p.catch) p.catch((e) => this.sys(`write failed: ${e.message || e}`, 'error'));
  }

  /** Send a frame without waiting for anything. */
  sendRaw(type, data = [], dest = this.opts.dest) {
    if (!this.connected) throw new Error('not connected');
    const fr = new Frame(dest, this.opts.ownAddr, type, data);
    this._write(fr);
    return fr;
  }

  /**
   * Queue a request; resolves with {ok, error, frame, rtt, nack, type}.
   * opts: dest, timeoutMs, retries, priority (bypass the queue), coalesce
   * (replace a queued request with the same key), kind ('cmd' | 'image').
   */
  request(type, data = [], opts = {}) {
    if (!this.connected) return Promise.resolve({ ok: false, error: 'offline', type });
    return new Promise((resolve) => {
      const req = {
        type, data: toBytes(data), dest: opts.dest ?? this.opts.dest, kind: opts.kind || 'cmd',
        timeoutMs: opts.timeoutMs ?? this.opts.timeoutMs, retries: opts.retries ?? 0, attempts: 0,
        priority: !!opts.priority, coalesce: opts.coalesce || null, label: opts.label || typeName(type), resolve,
      };
      if (req.priority) { this._send(req); return; }
      if (req.coalesce) {
        const q = this.queue.find((r) => r.coalesce === req.coalesce);
        if (q) {
          q.data = req.data;
          const prev = q.resolve;
          q.resolve = (res) => { prev({ ...res, superseded: true }); resolve(res); };
          return;
        }
      }
      this.queue.push(req);
      this._pump();
    });
  }

  _pump() {
    if (!this.connected) return;
    if (this.inflight.some((r) => !r.priority)) return;
    const req = this.queue.shift();
    if (req) this._send(req);
  }

  _send(req) {
    req.attempts += 1;
    req.frame = new Frame(req.dest, this.opts.ownAddr, req.type, req.data);
    if (!this.inflight.includes(req)) this.inflight.push(req);
    if (req.attempts === 1) this.counters.requests += 1; else this.counters.retries += 1;
    if (req.kind === 'image') this._startImage(req);
    req.t0 = this.clock();
    this._write(req.frame);
    clearTimeout(req.timer);
    if (req.kind === 'image') this._armImageTimer(this.opts.imageFirstMs);
    else req.timer = setTimeout(() => this._timeout(req), req.timeoutMs);
    this.emit('queue', this.queueState());
  }

  _timeout(req) {
    if (!this.inflight.includes(req)) return;
    if (req.attempts <= req.retries && this.connected) {
      this.sys(`${req.label}: no reply, retrying (${req.attempts}/${req.retries})`, 'warn');
      this._send(req);
      return;
    }
    this.inflight = this.inflight.filter((r) => r !== req);
    this.counters.req_timeouts += 1;
    this.sys(`${req.label}: no reply after ${req.timeoutMs} ms`, 'warn');
    const res = { ok: false, error: 'timeout', type: req.type, attempts: req.attempts };
    req.resolve(res);
    this.emit('reply', { req, res });
    this._pump();
  }

  _resolve(req, verdict, fr, t) {
    clearTimeout(req.timer);
    this.inflight = this.inflight.filter((r) => r !== req);
    const rtt = Math.max(0, t - req.t0);
    const res = { ok: verdict === 'done', error: verdict === 'nack' ? 'nack' : null, frame: fr, rtt,
      type: req.type, attempts: req.attempts };
    if (verdict === 'nack') {
      res.nack = Nack.unpack(fr.data);
      this.counters.nacks += 1;
    } else {
      this.counters.replies += 1;
    }
    if (req.type === PT.PING && verdict === 'done') {
      const ms = rtt * 1000;
      const r = this.rtt;
      r.last = ms; r.n += 1; r.sum += ms;
      r.min = r.min == null ? ms : Math.min(r.min, ms);
      r.max = r.max == null ? ms : Math.max(r.max, ms);
    }
    req.resolve(res);
    this.emit('reply', { req, res });
    this._pump();
  }

  queueState() {
    return { queued: this.queue.length, inflight: this.inflight.map((r) => r.label), image: !!this.image };
  }

  // ------------------------------------------------------------ images
  _startImage(req) {
    if (this.image && this.image.req && this.image.req !== req) this._finishImage('superseded');
    this.image = new ImageAssembler(this.decoder.stats.bytes_in, this.clock());
    this.image.req = req;
    this.emit('image-progress', this._imageProgress());
  }

  _armImageTimer(ms) {
    const img = this.image;
    if (!img) return;
    clearTimeout(img.timer);
    img.timer = setTimeout(() => { if (this.image === img) this._finishImage(img.total ? 'idle timeout' : 'timeout'); }, ms);
  }

  _imageProgress() {
    const a = this.image;
    if (!a) return null;
    return { active: true, legacy: a.legacy, total: a.total, received: a.chunks.size, width: a.width,
      height: a.height, fmt: a.fmt, bytes: a.bytes, states: a.states(false), elapsed: this.clock() - a.t0 };
  }

  _finishImage(reason, nack = null) {
    const a = this.image;
    if (!a) return;
    clearTimeout(a.timer);
    this.image = null;
    const result = a.total ? a.result(this.clock(), reason) : null;
    if (result) {
      result.states = a.states(true);
      this.images.push(result);
      if (this.images.length > this.opts.imagesMax) this.images.shift();
      this.emit('image', result);
      const miss = result.missing.length ? `, missing chunks ${result.missing.join(', ')}` : '';
      this.sys(`Image ${result.width}x${result.height} ${result.legacy ? 'legacy' : (result.fmt ? 'JPEG' : 'gray8')}: ` +
        `${result.received.length}/${result.total} chunks in ${result.elapsed.toFixed(1)} s${miss}`,
      result.complete ? 'info' : 'warn');
    }
    const req = a.req;
    if (req) {
      this.inflight = this.inflight.filter((r) => r !== req);
      let res;
      if (nack) res = { ok: false, error: 'nack', nack, type: PT.GET_IMAGE };
      else if (!result) {
        this.counters.req_timeouts += 1;
        res = { ok: false, error: reason === 'aborted' ? 'closed' : 'timeout', type: PT.GET_IMAGE };
      } else {
        this.counters.replies += 1;
        res = { ok: result.complete, error: result.complete ? null : 'incomplete', image: result, type: PT.GET_IMAGE };
      }
      req.resolve(res);
      this.emit('reply', { req, res });
    }
    this.emit('image-progress', null);
    this._pump();
  }

  // ---------------------------------------------------------- commands
  ping(opts) { return this.request(PT.PING, [], opts); }
  getTlm(opts) { return this.request(PT.GET_TLM, [], opts); }
  capImage(opts = {}) { return this.request(PT.CAP_IMAGE, [], { timeoutMs: Math.max(this.opts.timeoutMs, 3000), ...opts }); }
  startScan(opts) { return this.request(PT.START_SCAN, [], opts); }
  stopScan(opts) { return this.request(PT.STOP_SCAN, [], opts); }
  collectObject(opts) { return this.request(PT.COLLECT_OBJECT, [], opts); }
  clearEstop(opts) { return this.request(PT.CLEAR_ESTOP, [], opts); }
  setMode(mode, opts) { return this.request(PT.SET_MODE, [mode], opts); }
  getImage({ v2 = false, quality = 0 } = {}) {
    return this.request(PT.GET_IMAGE, v2 ? [quality] : [], { kind: 'image', label: v2 ? `GET_IMAGE q=${quality}` : 'GET_IMAGE' });
  }

  drive(vPct, wPct, durationMs = 600) {
    const data = new Drive({ v_pct: vPct, w_pct: wPct, duration_ms: durationMs }).pack();
    return this.request(PT.DRIVE, data, { coalesce: 'drive', timeoutMs: 1000, label: 'DRIVE' });
  }

  /** E-STOP: bypasses the queue, cancels queued motion commands, retried until acknowledged. */
  estop() {
    for (const r of [...this.queue]) {
      if ([PT.DRIVE, PT.START_SCAN, PT.COLLECT_OBJECT].includes(r.type)) {
        this.queue = this.queue.filter((x) => x !== r);
        r.resolve({ ok: false, error: 'cancelled by E-STOP', type: r.type });
      }
    }
    this.sys('E-STOP sent', 'warn');
    return this.request(PT.ESTOP, [], { priority: true, retries: 3, timeoutMs: 700, label: 'E-STOP' });
  }

  // ----------------------------------------------------------- polling
  setAutoPoll(intervalS) {
    clearInterval(this._pollTimer);
    this._pollTimer = null;
    this.pollIntervalS = intervalS;
    if (intervalS > 0) this._pollTimer = setInterval(() => this._pollTick(), intervalS * 1000);
  }

  _pollTick() {
    if (!this.connected || this.image) return; // never talk over an image downlink (half-duplex)
    if ([...this.queue, ...this.inflight].some((r) => r.type === PT.GET_TLM)) return;
    this.request(PT.GET_TLM, [], { label: 'GET_TLM (auto)' });
  }

  // --------------------------------------------------------------- log
  _logFrame(dir, fr) {
    this._push({ t: Date.now(), dir, type: fr.type, name: typeName(fr.type), len: fr.data.length,
      hex: toHex(fr.encode(this.opts.transparency)), text: describeFrame(fr), src: fr.src, dest: fr.dest });
  }

  sys(text, level = 'info') {
    this._push({ t: Date.now(), dir: 'SYS', name: level.toUpperCase(), len: 0, hex: '', text, level });
  }

  _push(entry) {
    this.log.push(entry);
    if (this.log.length > this.opts.logMax) this.log.splice(0, this.log.length - this.opts.logMax);
    this.emit('log', entry);
  }

  // ------------------------------------------------------------- stats
  stats() {
    const r = this.rtt;
    const d = this.decoder.stats;
    const bad = d.crc_errors + d.framing_errors;
    return {
      ...d.asDict(), ...this.counters,
      rtt_last_ms: r.last, rtt_avg_ms: r.n ? r.sum / r.n : null, rtt_min_ms: r.min, rtt_max_ms: r.max, pings: r.n,
      frame_quality: d.frames_ok + bad ? d.frames_ok / (d.frames_ok + d.crc_errors) : null,
      queued: this.queue.length, inflight: this.inflight.length,
      bad_events: bad,
    };
  }

  // ------------------------------------------------------- data export
  telemetryCsv() { return samplesToCsv(this.history); }

  exportSession(settings = {}) {
    return {
      format: 'missionx-gs-session', version: 1, saved: new Date().toISOString(), settings,
      stats: this.stats(), history: this.history, log: this.log.slice(-1500),
      images: this.images.map((im) => ({ ...im, data: bytesToB64(im.data) })),
    };
  }

  importSession(obj) {
    if (!obj || obj.format !== 'missionx-gs-session') throw new Error('not a MISSION X GS session file');
    this.history = Array.isArray(obj.history) ? obj.history : [];
    this.log = Array.isArray(obj.log) ? obj.log : [];
    this.images = (obj.images || []).map((im) => ({ ...im, data: b64ToBytes(im.data) }));
    this.sys(`Session loaded (${this.history.length} telemetry samples, ${this.images.length} images, saved ${obj.saved})`);
    this.emit('session', obj);
    return obj.settings || {};
  }
}

