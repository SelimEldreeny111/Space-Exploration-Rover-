// End-to-end: GroundStation (gs.js) <-> VirtualLink/RoverModel (sim.js), no DOM.
import test from 'node:test';
import assert from 'node:assert/strict';

import { GroundStation, ImageAssembler, LEGACY_CHUNK_WIRE, classify } from '../js/gs.js';
import { VirtualLink, BitErrors, mulberry32 } from '../js/sim.js';
import { Frame } from '../js/ssp.js';
import { Addr, Fault, Mode, NackReason, PT, TlmLegacy, TlmV2, CSV_COLUMNS } from '../js/protocol.js';

/** fast link: 1 Mbaud, no air latency, processing delays x0.01 */
async function setup(opts = {}) {
  const link = new VirtualLink({ baud: 1_000_000, latencyMs: 0, jitterMs: 0, timeScale: 0.01, chunkMs: 2, ...opts });
  const gs = new GroundStation({ timeoutMs: 400, imageFirstMs: 1500, imageIdleMs: 300 });
  await gs.connect(link);
  return { gs, link, model: link.model };
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

test('ping, telemetry, commands and NACKs through the virtual link', async () => {
  const { gs, model } = await setup();
  try {
    const p = await gs.ping();
    assert.ok(p.ok && p.frame.type === PT.ACK && p.rtt >= 0);
    assert.equal(gs.stats().pings, 1);

    const t = await gs.getTlm();
    assert.ok(t.ok);
    assert.ok(gs.lastTlm instanceof TlmV2);
    assert.equal(gs.history.length, 1);
    assert.equal(gs.history[0].tlm_version, 2);

    assert.ok((await gs.capImage()).ok);
    assert.ok((await gs.startScan()).ok);
    assert.equal(model.mode, Mode.AUTONOMOUS);
    const busy = await gs.drive(40, 0, 500);
    assert.equal(busy.error, 'nack');
    assert.equal(busy.nack.reason, NackReason.BUSY);
    assert.ok((await gs.stopScan()).ok);

    const e = await gs.estop();
    assert.ok(e.ok && e.frame.data[0] === PT.ESTOP);
    await gs.getTlm();
    assert.ok(gs.lastTlm.fault_flags & Fault.ESTOP);
    const rej = await gs.drive(40, 0, 500);
    assert.equal(rej.nack.reason, NackReason.NOT_PERMITTED);
    assert.ok((await gs.clearEstop()).ok);
    assert.ok((await gs.setMode(Mode.TELEOP)).ok);
    assert.ok((await gs.drive(60, 0, 300)).ok);
    assert.ok((await gs.collectObject()).ok);
    const unk = await gs.request(0x77);
    assert.equal(unk.nack.reason, NackReason.UNKNOWN_TYPE);
    const csv = gs.telemetryCsv().trim().split('\r\n');
    assert.equal(csv[0], CSV_COLUMNS.join(','));
  } finally {
    await gs.disconnect();
  }
});

test('legacy and v2 image downloads are exact', async () => {
  const { gs, model } = await setup();
  try {
    const r = await gs.getImage();
    assert.ok(r.ok, JSON.stringify(r.error));
    assert.deepEqual([r.image.width, r.image.height, r.image.total, r.image.legacy], [100, 100, 50, true]);
    assert.deepEqual(r.image.data, model.scene.render(100, 100));
    const v2 = await gs.getImage({ v2: true, quality: 0 });
    assert.ok(v2.ok && !v2.image.legacy && v2.image.fmt === 0);
    assert.deepEqual(v2.image.data, model.scene.render(100, 100));
    const jpg = await gs.getImage({ v2: true, quality: 60 }); // no OffscreenCanvas in Node -> gray8 fallback
    assert.ok(jpg.ok && jpg.image.total === 50);
    assert.equal(gs.images.length, 3);
  } finally {
    await gs.disconnect();
  }
});

test('bit errors: legacy image chunks stay in place, missing chunks reported', async () => {
  for (const seed of [3, 4, 5]) {
    const { gs, model } = await setup({ ber: 1e-4, seed });
    try {
      const r = await gs.getImage();
      const im = r.image;
      const truth = model.scene.render(100, 100);
      assert.ok(im && im.missing.length > 0, `seed ${seed}: expected corrupted chunks`);
      assert.equal(im.received.length + im.missing.length, 50);
      for (const i of im.received) {
        assert.deepEqual(im.data.subarray(i * 200, (i + 1) * 200), truth.subarray(i * 200, (i + 1) * 200),
          `seed ${seed}: chunk ${i} misplaced`);
      }
      const st = gs.stats();
      assert.ok(st.crc_errors + st.framing_errors > 0);
    } finally {
      await gs.disconnect();
    }
  }
});

test('legacy rover: 7-byte TLM, v2 commands time out', async () => {
  const { gs } = await setup({ legacy: true });
  try {
    assert.ok((await gs.getTlm()).ok);
    assert.ok(gs.lastTlm instanceof TlmLegacy);
    assert.equal(gs.history.at(-1).tlm_version, 1);
    const r = await gs.setMode(1);
    assert.equal(r.error, 'timeout');
    assert.ok(gs.stats().req_timeouts >= 1);
  } finally {
    await gs.disconnect();
  }
});

test('auto-poll fills the history and drive requests coalesce', async () => {
  const { gs } = await setup();
  try {
    gs.setAutoPoll(0.05);
    await sleep(400);
    gs.setAutoPoll(0);
    assert.ok(gs.history.length >= 3, `history ${gs.history.length}`);
    const a = gs.drive(10, 0, 300); // in flight
    const b = gs.drive(20, 0, 300); // queued
    const c = gs.drive(30, 0, 300); // replaces b
    const [ra, rb, rc] = await Promise.all([a, b, c]);
    assert.ok(ra.ok && rc.ok && rb.superseded);
  } finally {
    await gs.disconnect();
  }
});

test('session export/import round trip', async () => {
  const { gs } = await setup();
  try {
    await gs.getTlm();
    await gs.getImage();
    const s = JSON.parse(JSON.stringify(gs.exportSession({ baud: 9600 })));
    const gs2 = new GroundStation();
    const settings = gs2.importSession(s);
    assert.equal(settings.baud, 9600);
    assert.equal(gs2.history.length, gs.history.length);
    assert.deepEqual(gs2.images[0].data, gs.images[0].data);
  } finally {
    await gs.disconnect();
  }
});

test('reply classification rules', () => {
  const req = (t) => new Frame(1, 11, t);
  assert.equal(classify(req(PT.PING), new Frame(11, 1, PT.ACK)), 'done');
  assert.equal(classify(req(PT.ESTOP), new Frame(11, 1, PT.ACK, [PT.ESTOP])), 'done');
  assert.equal(classify(req(PT.ESTOP), new Frame(11, 1, PT.ACK, [PT.DRIVE])), null);
  assert.equal(classify(req(PT.START_SCAN), new Frame(11, 1, PT.START_SCAN)), 'done');
  assert.equal(classify(req(PT.GET_TLM), new Frame(11, 1, PT.GET_TLM, [1, 2, 3, 4, 5, 6, 7])), 'done');
  assert.equal(classify(req(PT.DRIVE), new Frame(11, 1, PT.NACK, [PT.DRIVE, 4])), 'nack');
  assert.equal(classify(req(PT.DRIVE), new Frame(11, 1, PT.NACK, [PT.PING, 4])), null);
});

test('assembler infers lost legacy chunks from the byte count', () => {
  const asm = new ImageAssembler(1000, 0);
  let pos = 1000;
  for (let i = 0; i < 50; i++) {
    pos += LEGACY_CHUNK_WIRE;
    if ([0, 7, 8, 49].includes(i)) continue;
    asm.add(new Frame(Addr.GROUND, Addr.ROVER, PT.GET_IMAGE, new Uint8Array(200).fill(i)), pos);
  }
  const res = asm.result(1);
  assert.deepEqual(res.missing, [0, 7, 8, 49]);
  for (const i of res.received) assert.equal(res.data[i * 200], i);
});

test('bit error injector rate', () => {
  const inj = new BitErrors(1e-3, mulberry32(1));
  const out = inj.apply(new Uint8Array(100000));
  let flips = 0;
  for (const b of out) for (let k = 0; k < 8; k++) flips += (b >> k) & 1;
  assert.equal(flips, inj.flipped);
  assert.ok(flips > 600 && flips < 1000, `${flips}`);
});
