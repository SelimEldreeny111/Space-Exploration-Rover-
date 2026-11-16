// Tests for web/js/protocol.js (payload layouts, ICD §6-§7).
import test from 'node:test';
import assert from 'node:assert/strict';

import { fromHex, toHex } from '../js/ssp.js';
import {
  ArmSetJoints, ArmState, Drive, EnvTlm, Heartbeat, ImageChunkHeader, Imu, Nack, PT, Ranges,
  SetVelocity, TlmLegacy, TlmV2, WheelOdom, decodeTlm, describeFrame, faultNames, normalizeTlm,
  plausibleLength, samplesToCsv, CSV_COLUMNS, pyRound,
} from '../js/protocol.js';
import { Frame } from '../js/ssp.js';

test('payload sizes (ICD §6.2, §7.1)', () => {
  assert.equal(TlmV2.size(), 56);
  assert.equal(EnvTlm.size(), 20);
  assert.equal(Imu.size(), 20);
  assert.equal(WheelOdom.size(), 28);
  assert.equal(SetVelocity.size(), 6);
  assert.equal(ArmSetJoints.size(), 8);
  assert.equal(ArmState.size(), 7);
  assert.equal(Heartbeat.size(), 6);
  assert.equal(TlmLegacy.size(), 7);
  assert.equal(Drive.size(), 4);
  assert.equal(Nack.size(), 2);
  assert.equal(ImageChunkHeader.size(), 9);
});

test('payload round trips', () => {
  const t = new TlmV2({ mode: 2, battery_mV: 12450, temperature_cC: 2510, pressure_Pa: 101325,
    x_mm: -1234, y_mm: 5678, heading_cdeg: -9000, range_min_mm: 812 });
  assert.deepEqual(TlmV2.unpack(t.pack()).asDict(), t.asDict());
  assert.ok(decodeTlm(t.pack()) instanceof TlmV2);
  assert.ok(decodeTlm(fromHex('00 87 00 46 2E 08 16')) instanceof TlmLegacy);
  const w = new WheelOdom({ t_ms: 123456, ticks: [1, -2, 3, -4, 5, 2 ** 31 - 1] });
  assert.deepEqual(WheelOdom.unpack(w.pack()).asDict(), w.asDict());
  const r = new Ranges({ range_mm: [100, 2000, 0xFFFF, 4, 5, 6] });
  assert.deepEqual(Ranges.unpack(r.pack()).asDict(), r.asDict());
  assert.equal(r.meters()[2], Infinity);
  const sv = SetVelocity.fromSI(0.3, -0.5, 0.5);
  assert.deepEqual([sv.v_mm_s, sv.w_mrad_s, sv.timeout_ms], [300, -500, 500]);
});

test('legacy TLM fields', () => {
  const t = TlmLegacy.unpack(fromHex('00 87 00 46 2E 08 16'));
  assert.equal(t.gas_raw, 0x8700);
  assert.equal(t.us1_cm, 0x46);
  assert.equal(t.temp_c, 8);
  assert.equal(t.rh_pct, 22);
});

test('struct range errors like Python struct.error', () => {
  assert.throws(() => new Drive({ v_pct: 200 }).pack(), RangeError);
  assert.throws(() => new Nack({ rejected_type: -1 }).pack(), RangeError);
  assert.throws(() => TlmV2.unpack(new Uint8Array(55)), /expected 56 bytes/);
  assert.throws(() => decodeTlm(new Uint8Array(17)), /unknown telemetry layout/);
});

test('little-endian layout of DRIVE and IMAGE_CHUNK header', () => {
  assert.equal(toHex(new Drive({ v_pct: -1, w_pct: 50, duration_ms: 0x1234 }).pack()), 'FF 32 34 12');
  const h = ImageChunkHeader.unpack(fromHex('0100 3200 6400 6400 00 AABB'));
  assert.deepEqual(h.asDict(), { seq: 1, total: 50, width: 100, height: 100, fmt: 0 });
});

test('Python round-half-even', () => {
  assert.deepEqual([0.5, 1.5, 2.5, -0.5, -1.5, -2.5, 2.4, 2.6].map(pyRound), [0, 2, 2, 0, -2, -2, 2, 3]);
});

test('plausible length examples from ICD §3 step 5a', () => {
  assert.ok(plausibleLength(PT.PING, 0));
  assert.ok(!plausibleLength(PT.PING, 1));
  assert.ok(plausibleLength(PT.SET_VELOCITY, 6));
  assert.ok(!plausibleLength(PT.SET_VELOCITY, 7));
  assert.ok(plausibleLength(PT.GET_TLM, 68));
  assert.ok(plausibleLength(0x77, 13));
  assert.ok(!plausibleLength(0x77, 13, true));
});

test('describeFrame and faults', () => {
  assert.equal(describeFrame(new Frame(1, 11, PT.PING)), 'PING');
  assert.equal(describeFrame(new Frame(11, 1, PT.ACK, [PT.ESTOP])), 'ACK of ESTOP');
  assert.equal(describeFrame(new Frame(11, 1, PT.NACK, [PT.DRIVE, 4])), 'NACK of DRIVE: NOT_PERMITTED');
  assert.match(describeFrame(new Frame(11, 1, PT.GET_TLM, new TlmV2({ mode: 2, battery_mV: 12000 }).pack())),
    /TLM v2: AUTONOMOUS batt=12.00V/);
  assert.deepEqual(faultNames(0b10101), ['ESTOP', 'LOW_BATTERY', 'SAFE_LINK']);
});

test('normalised samples and CSV', () => {
  const s = normalizeTlm(new TlmV2({ battery_mV: 12345, range_min_mm: 0xFFFF, x_mm: 1500 }), 0);
  assert.equal(s.battery_V, 12.345);
  assert.equal(s.range_min_m, null);
  assert.equal(s.x_m, 1.5);
  const l = normalizeTlm(TlmLegacy.unpack(fromHex('01 00 10 46 2E 19 28')), 0);
  assert.equal(l.tlm_version, 1);
  assert.equal(l.range_min_m, 0.46);
  const csv = samplesToCsv([s, l]).trim().split('\r\n');
  assert.equal(csv.length, 3);
  assert.equal(csv[0], CSV_COLUMNS.join(','));
  assert.equal(csv[1].split(',').length, CSV_COLUMNS.length);
});
