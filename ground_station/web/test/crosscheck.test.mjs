// Cross-check of the JavaScript codec against the Python reference
// implementation (rover_core.ssp / rover_core.protocol).
//
// test_vectors.json is produced by ground_station/tools/gen_vectors.py; every
// stream (corrupted with dropped bytes, bit flips, inserted noise, late start,
// timeouts, SLIP) must decode to exactly the frames and statistics the Python
// StreamDecoder produced.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

import { Frame, StreamDecoder, crc16Xmodem, encode, fromHex, toHex } from '../js/ssp.js';
import * as P from '../js/protocol.js';

const V = JSON.parse(readFileSync(new URL('./test_vectors.json', import.meta.url), 'utf8'));
const hx = (b) => toHex(b, '');

const LENGTH_CHECKS = {
  null: null,
  plausible: (t, n) => P.plausibleLength(t, n),
  strict: (t, n) => P.plausibleLength(t, n, true),
};

test('vectors file is present and complete', () => {
  assert.ok(V.crc.length > 10 && V.encode.length > 50 && V.streams.length > 10 && V.payloads.length > 50);
});

test('CRC vectors', () => {
  for (const v of V.crc) assert.equal(crc16Xmodem(fromHex(v.data)), v.crc, v.data);
});

test('ICD reference vectors', () => {
  for (const v of V.reference) {
    assert.equal(hx(Frame.fromJSON(v.frame).encode()), v.wire, v.name);
  }
});

test('encodings (transparency off and on)', () => {
  for (const v of V.encode) {
    const f = v.frame;
    assert.equal(hx(encode(f.dest, f.src, f.type, fromHex(f.data), v.transparency)), v.wire);
  }
});

for (const sc of V.streams) {
  test(`stream decodes identically to Python: ${sc.name}`, () => {
    const c = sc.config;
    const dec = new StreamDecoder({
      ownAddr: c.own_addr, transparency: c.transparency, timeoutS: c.timeout_s,
      promiscuous: c.promiscuous, clock: () => 0, lengthCheck: LENGTH_CHECKS[String(c.length_check)],
    });
    const frames = [];
    for (const [now, data] of sc.chunks) frames.push(...dec.feed(fromHex(data), now ?? 0));
    assert.deepEqual(frames.map((f) => f.toJSON()), sc.expected_frames);
    assert.deepEqual(dec.stats.asDict(), sc.expected_stats);
  });
}

test('stream decoding is independent of chunking (byte-by-byte feed)', () => {
  for (const sc of V.streams.filter((s) => s.config.timeout_s === null)) {
    const c = sc.config;
    const dec = new StreamDecoder({ ownAddr: c.own_addr, transparency: c.transparency, timeoutS: null,
      promiscuous: c.promiscuous, lengthCheck: LENGTH_CHECKS[String(c.length_check)] });
    const frames = [];
    for (const [, data] of sc.chunks) for (const b of fromHex(data)) frames.push(...dec.feed([b]));
    assert.deepEqual(frames.map((f) => f.toJSON()), sc.expected_frames, sc.name);
  }
});

test('payload structs pack/unpack identically', () => {
  for (const v of V.payloads) {
    const cls = P.PAYLOAD_CLASSES[v.cls];
    assert.ok(cls, v.cls);
    const obj = cls.unpack(fromHex(v.hex));
    assert.deepEqual(obj.asDict(), v.fields, v.cls);
    assert.equal(hx(new cls(v.fields).pack()), v.hex, v.cls);
  }
});

test('struct sizes', () => {
  for (const [name, size] of Object.entries(V.sizes)) assert.equal(P.PAYLOAD_CLASSES[name].size(), size, name);
});

test('SetVelocity.fromSI rounds like Python', () => {
  for (const v of V.set_velocity_from_si) {
    assert.deepEqual(P.SetVelocity.fromSI(v.v, v.w, v.timeout_s).asDict(), v.fields, JSON.stringify(v));
  }
});

test('decodeTlm selects the same layout', () => {
  for (const v of V.decode_tlm) {
    let res;
    try { res = P.decodeTlm(fromHex(v.hex)).constructor.name; } catch (e) { res = 'error'; }
    assert.equal(res, v.result, v.hex);
  }
});

test('plausibleLength table identical for all 256 types (strict and non-strict)', () => {
  for (let t = 0; t < 256; t++) {
    for (const strict of [false, true]) {
      const ok = [];
      for (let n = 0; n < 256; n++) if (P.plausibleLength(t, n, strict)) ok.push(n);
      const exp = V.plausible_length[`${t}:${strict ? 1 : 0}`];
      assert.deepEqual(ok.length === 256 ? 'any' : ok, exp, `type ${t} strict=${strict}`);
    }
  }
});
