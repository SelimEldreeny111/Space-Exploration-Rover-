// Unit tests for web/js/ssp.js -- mirror of ros2_ws/src/rover_core/test/test_ssp.py.
// Run: node --test   (from ground_station/web)
import test from 'node:test';
import assert from 'node:assert/strict';

import { FEND, Frame, StreamDecoder, crc16Xmodem, encode, fromHex, toHex } from '../js/ssp.js';
import { Addr, PT, plausibleLength } from '../js/protocol.js';

const h = (s) => fromHex(s);
const ascii = (s) => new TextEncoder().encode(s);

/** Small deterministic PRNG (mulberry32). */
function rng(seed) {
  let a = seed >>> 0;
  const next = () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  return { random: next, randrange: (n) => Math.floor(next() * n) };
}

const concat = (...parts) => {
  const n = parts.reduce((s, p) => s + p.length, 0);
  const out = new Uint8Array(n);
  let o = 0;
  for (const p of parts) { out.set(p, o); o += p.length; }
  return out;
};

const sameFrames = (a, b) => a.length === b.length && a.every((f, i) => f.equals(b[i]));
const contains = (list, fr) => list.some((f) => f.equals(fr));

// --------------------------------------------------------------------- CRC
test('CRC-16/XMODEM check value 0x31C3', () => {
  assert.equal(crc16Xmodem(ascii('123456789')), 0x31C3);
});

test('CRC matches the 2023 LabVIEW capture', () => {
  assert.equal(crc16Xmodem(Uint8Array.of(0x01, 0x0B, 0x0F, 0x00)), 0x967B);
});

// --------------------------------------------------------- ICD §2.1 vectors
const VECTORS = [
  ['GS->Rover PING', new Frame(Addr.ROVER, Addr.GROUND, PT.PING), 'C0 01 0B 00 00 45 86 C0'],
  ['GS->Rover GET_TLM', new Frame(Addr.ROVER, Addr.GROUND, PT.GET_TLM), 'C0 01 0B 0F 00 7B 96 C0'],
  ['Rover->GS ACK', new Frame(Addr.GROUND, Addr.ROVER, PT.ACK), 'C0 0B 01 01 00 1E 1A C0'],
  ['Rover->GS legacy TLM', new Frame(Addr.GROUND, Addr.ROVER, PT.GET_TLM, h('00 87 00 46 2E 08 16')),
    'C0 0B 01 0F 07 00 87 00 46 2E 08 16 80 D9 C0'],
];

for (const [name, frame, wire] of VECTORS) {
  test(`ICD §2.1 vector: ${name}`, () => {
    assert.equal(toHex(frame.encode()), wire);
    const dec = new StreamDecoder({ timeoutS: null });
    const out = dec.feed(h(wire));
    assert.equal(out.length, 1);
    assert.ok(out[0].equals(frame));
  });
}

// ----------------------------------------------------------- round trips
for (const transparency of [false, true]) {
  test(`random round trip (transparency=${transparency})`, () => {
    const r = rng(1);
    let frames = [];
    for (let i = 0; i < 300; i++) {
      const data = Uint8Array.from({ length: r.randrange(256) }, () => r.randrange(256));
      frames.push(new Frame(r.randrange(256), r.randrange(256), r.randrange(256), data));
    }
    // avoid DEST/SRC/TYPE == 0xC0 when transparency is off (cannot occur in practice)
    if (!transparency) frames = frames.filter((f) => ![f.dest, f.src, f.type].includes(FEND));
    const stream = concat(...frames.map((f) => f.encode(transparency)));
    const dec = new StreamDecoder({ transparency, timeoutS: null });
    const out = [];
    let i = 0;
    while (i < stream.length) {
      const n = 1 + r.randrange(39);
      out.push(...dec.feed(stream.subarray(i, i + n)));
      i += n;
    }
    assert.ok(sameFrames(out, frames));
    assert.equal(dec.stats.crc_errors, 0);
    assert.equal(dec.stats.framing_errors, 0);
  });
}

test('payload containing flags without transparency', () => {
  const f = new Frame(0x0B, 0x01, PT.GET_TLM, [0xC0, 0xC0, 0xDB, 0x00, 0xC0]);
  const dec = new StreamDecoder({ timeoutS: null });
  assert.ok(sameFrames(dec.feed(concat(Uint8Array.of(0x00, 0x13), f.encode(), f.encode())), [f, f]));
});

test('shared flags between frames', () => {
  const a = new Frame(1, 11, PT.PING);
  const b = new Frame(1, 11, PT.GET_TLM);
  const wire = concat(a.encode(), b.encode().subarray(1)); // C0 a C0 b C0
  assert.ok(sameFrames(new StreamDecoder({ timeoutS: null }).feed(wire), [a, b]));
});

test('CRC error then recovery', () => {
  const a = new Frame(1, 11, PT.START_SCAN);
  const bad = a.encode();
  bad[3] ^= 0x04; // corrupt TYPE
  const dec = new StreamDecoder({ timeoutS: null });
  const out = dec.feed(concat(bad, a.encode()));
  assert.ok(sameFrames(out, [a]));
  assert.equal(dec.stats.crc_errors, 1);
});

const TRAFFIC = concat(...Array.from({ length: 40 }, () => new Frame(1, 11, PT.PING).encode())); // > 257 B

for (const check of [null, plausibleLength]) {
  test(`dropped byte resynchronises (lengthCheck=${check ? 'plausible' : 'none'})`, () => {
    const frames = [PT.PING, PT.GET_TLM, PT.START_SCAN, PT.STOP_SCAN].map((t) => new Frame(1, 11, t));
    const full = concat(...frames.map((f) => f.encode()));
    const wire = concat(full.subarray(0, 9), full.subarray(10)); // drop a byte inside the 2nd frame
    const dec = new StreamDecoder({ timeoutS: null, lengthCheck: check });
    const out = dec.feed(wire);
    // without the check a false LEN may hold the decision until LEN+2 bytes have
    // arrived, but no valid frame is lost once more traffic arrives
    if (!check) out.push(...dec.feed(TRAFFIC));
    assert.ok(contains(out, frames[0]) && contains(out, frames[2]) && contains(out, frames[3]));
    assert.ok(!contains(out, frames[1]));
  });

  test(`false sync inside payload recovers (lengthCheck=${check ? 'plausible' : 'none'})`, () => {
    const inner = new Frame(1, 11, PT.PING).encode();
    const carrier = new Frame(0x0B, 0x01, PT.GET_TLM, concat(inner, Uint8Array.of(1, 2)));
    const nxt = new Frame(1, 11, PT.STOP_SCAN);
    const wire = carrier.encode();
    // receiver switched on late, in the middle of the carrier frame
    const dec = new StreamDecoder({ timeoutS: null, lengthCheck: check });
    const out = [...dec.feed(concat(wire.subarray(6), nxt.encode())), ...dec.feed(TRAFFIC)];
    assert.ok(contains(out, nxt));
  });
}

test('length check rejects implausible LEN at the header', () => {
  // GET_TLM-looking garbage with LEN 200 would hold the receiver for 202 bytes;
  // PING with LEN 5 is implausible and rejected immediately.
  const ping = new Frame(1, 11, PT.PING);
  const dec = new StreamDecoder({ timeoutS: null, lengthCheck: plausibleLength });
  const out = dec.feed(concat(Uint8Array.of(0xC0, 0x01, 0x0B, 0x00, 0x05), ping.encode()));
  assert.ok(sameFrames(out, [ping]));
  assert.equal(dec.stats.framing_errors, 1);
});

test('timeout drops partial frame', () => {
  const clock = [0.0];
  const dec = new StreamDecoder({ timeoutS: 0.05, clock: () => clock[0] });
  const f = new Frame(1, 11, PT.PING).encode();
  dec.feed(f.subarray(0, 4));
  clock[0] = 1.0;
  const out = dec.feed(f);
  assert.ok(sameFrames(out, [new Frame(1, 11, PT.PING)]));
  assert.equal(dec.stats.timeouts, 1);
});

test('address filter', () => {
  const dec = new StreamDecoder({ ownAddr: Addr.MOBILITY, timeoutS: null });
  const a = new Frame(Addr.MOBILITY, Addr.ROVER, PT.PING);
  const b = new Frame(Addr.PAYLOAD, Addr.ROVER, PT.PING);
  const c = new Frame(Addr.BROADCAST, Addr.ROVER, PT.PING);
  assert.ok(sameFrames(dec.feed(concat(a.encode(), b.encode(), c.encode())), [a, c]));
  assert.equal(dec.stats.filtered, 1);
});

test('reply swaps addresses', () => {
  const req = new Frame(Addr.ROVER, Addr.GROUND, PT.PING);
  const rsp = req.reply(PT.ACK);
  assert.deepEqual([rsp.dest, rsp.src, rsp.type], [Addr.GROUND, Addr.ROVER, PT.ACK]);
});

test('transparency escapes flags', () => {
  const f = new Frame(1, 11, PT.GET_TLM, [0xC0, 0xDB]);
  const wire = f.encode(true);
  assert.equal(wire.filter((b) => b === FEND).length, 2);
  assert.ok(sameFrames(new StreamDecoder({ transparency: true, timeoutS: null }).feed(wire), [f]));
});

test('frame validation errors', () => {
  assert.throws(() => new Frame(256, 0, 0), /out of range/);
  assert.throws(() => new Frame(0, 0, 0, new Uint8Array(256)), /> 255/);
  assert.throws(() => encode(1, 2, -1), /out of range/);
});

test('onError hook reports CRC and framing errors without changing decoding', () => {
  const events = [];
  const a = new Frame(1, 11, PT.START_SCAN);
  const bad = a.encode();
  bad[5] ^= 0xFF; // corrupt CRC
  const dec = new StreamDecoder({ timeoutS: null, onError: (k) => events.push(k) });
  assert.ok(sameFrames(dec.feed(concat(bad, a.encode())), [a]));
  assert.deepEqual(events, ['crc']);
});
