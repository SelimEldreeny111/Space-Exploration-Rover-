// MISSION X ground station -- browser UI (DOM wiring only; logic lives in gs.js).

import { GroundStation } from './gs.js';
import { VirtualLink } from './sim.js';
import { WebSerialLink } from './serial.js';
import { ArcGauge, AttitudeIndicator, TimeChart, TrackPlot, css } from './widgets.js';
import { FAULT_LABELS, Fault, PT, describeFrame, modeName, nackReasonName, typeName } from './protocol.js';

const $ = (id) => document.getElementById(id);
const store = {
  get(k, d) { try { const v = localStorage.getItem(`mxgs.${k}`); return v == null ? d : JSON.parse(v); } catch (e) { return d; } },
  set(k, v) { try { localStorage.setItem(`mxgs.${k}`, JSON.stringify(v)); } catch (e) { /* private mode */ } },
};

const gs = new GroundStation({ timeoutMs: 2000, ibtS: 0.1 });
const ui = {
  source: store.get('source', 'virtual'),
  windowS: store.get('win', 300),
  view: 'charts',
  logPaused: false,
  trackFrom: 0,
  imageIdx: -1,
  lastRx: 0,
  lostLink: false,
  charts: [],
};

// ================================================================ helpers
const fmt = (v, d = 1) => (v == null || !Number.isFinite(v) ? '—' : v.toFixed(d));
const ms = (s) => (s == null ? '' : `${(s * 1000).toFixed(0)} ms`);
const pad2 = (n) => String(n).padStart(2, '0');
const hhmmss = (d) => `${pad2(d.getUTCHours())}:${pad2(d.getUTCMinutes())}:${pad2(d.getUTCSeconds())}`;
const hhmmssms = (t) => { const d = new Date(t); return `${hhmmss(d)}.${String(d.getUTCMilliseconds()).padStart(3, '0')}`; };
const dur = (s) => {
  if (s == null) return '—';
  const h = Math.floor(s / 3600); const m = Math.floor((s % 3600) / 60); const sec = Math.floor(s % 60);
  return h ? `${h}h ${pad2(m)}m` : `${m}m ${pad2(sec)}s`;
};
const escapeHtml = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

function toast(msg, level = 'info', timeout = 5000) {
  const el = document.createElement('div');
  el.className = `toast ${level}`;
  el.textContent = msg;
  $('toasts').appendChild(el);
  setTimeout(() => el.remove(), timeout);
}

function download(blob, name) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = name;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 2000);
}

const stamp = () => new Date().toISOString().replace(/[:]/g, '').replace(/\..*/, '').replace('T', '_');

const rafPending = new Set();
/** Run fn once on the next animation frame -- or after 250 ms when frames are
 *  throttled (background tab, hidden pane) so counters and logs never stall. */
function schedule(fn) {
  if (rafPending.has(fn)) return;
  rafPending.add(fn);
  let done = false;
  const go = () => { if (done) return; done = true; rafPending.delete(fn); fn(); };
  requestAnimationFrame(go);
  setTimeout(go, 250);
}

// =========================================================== link panel
function setSource(src) {
  ui.source = src;
  store.set('source', src);
  for (const b of $('srcSeg').querySelectorAll('button')) b.setAttribute('aria-checked', String(b.dataset.src === src));
  for (const el of document.querySelectorAll('.virt')) el.hidden = src !== 'virtual';
  const note = $('serialNote');
  if (src === 'serial' && !WebSerialLink.supported()) {
    note.hidden = false;
    note.textContent = 'Web Serial is not available here. Use Chrome or Edge (desktop, or a Windows tablet) and open ' +
      'the page from file:// or http://localhost. The virtual rover works everywhere.';
  } else if (src === 'serial') {
    note.hidden = false;
    note.textContent = 'Connect asks you to pick the USB-UART adapter of the HC-12 (CH340 / CP210x / FTDI). ' +
      'Radio settings per ICD: channel 001, FU3, 9600 8N1.';
  } else {
    note.hidden = true;
  }
}

function applyLinkSettings() {
  gs.configure({
    dest: Number($('destSel').value), timeoutMs: Number($('timeoutSel').value),
    ibtS: Number($('ibtSel').value), transparency: $('slipChk').checked,
  });
}

async function toggleConnect() {
  const btn = $('btnConnect');
  if (gs.link) { await gs.disconnect(); return; }
  applyLinkSettings();
  const baud = Number($('baud').value);
  let link;
  if (ui.source === 'virtual') {
    const profile = $('vProfile').value;
    link = new VirtualLink({ baud, ber: Number($('vBer').value), legacy: profile === 'legacy',
      safeLink: profile === 'safe', seed: 1, transparency: $('slipChk').checked });
  } else {
    link = new WebSerialLink({ baud });
    link.onError = (e) => gs.sys(`serial: ${e.name || 'error'} ${e.message || ''}`, 'warn');
  }
  btn.disabled = true;
  btn.textContent = 'Connecting…';
  try {
    await gs.connect(link);
    ui.lostLink = false;
  } catch (e) {
    if (e && e.name === 'NotFoundError') toast('No serial port selected.', 'warn');
    else toast(`Could not connect: ${e.message || e}`, 'error');
  } finally {
    btn.disabled = false;
    updateLinkUi();
  }
}

function updateLinkUi() {
  const on = gs.connected;
  const btn = $('btnConnect');
  btn.textContent = on ? 'Disconnect' : 'Connect';
  btn.classList.toggle('connected', on);
  document.body.classList.toggle('offline', !on);
  for (const el of document.querySelectorAll('[data-cmd], .pad button, #autoPoll')) el.disabled = !on;
  for (const id of ['baud', 'vProfile', 'slipChk', 'destSel']) $(id).disabled = on;
  for (const b of $('srcSeg').querySelectorAll('button')) b.disabled = on;
  const link = gs.link;
  $('linkMeta').textContent = on ? `${link.kind === 'virtual' ? 'simulated' : 'serial'} · ${link.baud} Bd` : 'offline';
  updatePill();
  if (!on) $('cmdResult').textContent = ui.lostLink ? 'Link lost.' : 'Not connected.';
  else if ($('cmdResult').textContent.startsWith('Not connected') || $('cmdResult').textContent === 'Link lost.') {
    $('cmdResult').textContent = 'Ready.';
    $('cmdResult').dataset.state = '';
  }
}

function updatePill() {
  const pill = $('linkPill');
  const link = gs.link;
  if (!gs.connected) {
    pill.dataset.state = ui.lostLink ? 'lost' : 'off';
    $('linkText').textContent = ui.lostLink ? 'Link lost' : 'Offline';
    return;
  }
  const quiet = Date.now() - ui.lastRx > 6000;
  pill.dataset.state = quiet ? 'busy' : 'on';
  const what = link.kind === 'virtual' ? 'Virtual rover' : link.name;
  $('linkText').textContent = `${what} · ${link.baud} Bd${quiet ? ' · quiet' : ''}`;
}

function renderStats() {
  const s = gs.stats();
  $('sOk').textContent = s.frames_ok;
  $('sCrc').textContent = s.crc_errors;
  $('sCrc').classList.toggle('bad', s.crc_errors > 0);
  $('sFrm').textContent = s.framing_errors;
  $('sFrm').classList.toggle('bad', s.framing_errors > 0);
  $('sTo').textContent = s.timeouts;
  $('sBytes').textContent = `${s.bytes_in} / ${s.tx_bytes}`;
  $('sReq').textContent = `${s.req_timeouts} / ${s.nacks}`;
  $('sRtt').textContent = s.rtt_last_ms == null ? '—' : `${s.rtt_last_ms.toFixed(0)} ms`;
  $('sRttAvg').textContent = s.rtt_avg_ms == null ? '—'
    : `${s.rtt_avg_ms.toFixed(0)} (${s.rtt_min_ms.toFixed(0)}–${s.rtt_max_ms.toFixed(0)}) ms`;
  const q = gs.queueState();
  $('queueMeta').textContent = q.image ? 'image downlink…' : q.inflight.length
    ? `waiting: ${q.inflight.join(', ')}${q.queued ? ` (+${q.queued})` : ''}` : 'idle';
}

// ============================================================= commands
function setResult(text, state = '') {
  const el = $('cmdResult');
  el.textContent = text;
  el.dataset.state = state;
}

function showResult(label, res) {
  if (res.ok) {
    const what = res.frame ? describeFrame(res.frame) : (res.image ? 'image complete' : 'OK');
    setResult(`${what}${res.rtt != null ? ` · ${ms(res.rtt)}` : ''}`, 'ok');
  } else if (res.error === 'nack') {
    setResult(`NACK ${typeName(res.nack.rejected_type)}: ${nackReasonName(res.nack.reason)}`, 'nack');
  } else if (res.error === 'timeout') {
    setResult(`${label}: no reply${res.attempts > 1 ? ` after ${res.attempts} attempts` : ''}`, 'timeout');
  } else if (res.error === 'incomplete') {
    setResult(`${label}: ${res.image.missing.length} chunk(s) missing`, 'timeout');
  } else if (!res.superseded) {
    setResult(`${label}: ${res.error}`, 'timeout');
  }
}

function requestImage() {
  const q = $('imgQuality').value;
  return q === 'legacy' ? gs.getImage() : gs.getImage({ v2: true, quality: Number(q) });
}

async function run(cmd) {
  if (!gs.connected) { setResult('Not connected.', 'timeout'); return null; }
  const table = {
    ping: ['PING', () => gs.ping()],
    tlm: ['GET_TLM', () => gs.getTlm()],
    cap: ['CAP_IMAGE', () => gs.capImage()],
    img: ['GET_IMAGE', () => requestImage()],
    scan: ['START_SCAN', () => gs.startScan()],
    stop: ['STOP_SCAN', () => gs.stopScan()],
    collect: ['COLLECT_OBJECT', () => gs.collectObject()],
    clear: ['CLEAR_ESTOP', () => gs.clearEstop()],
    mode: ['SET_MODE', () => gs.setMode(Number($('modeSel').value))],
  };
  const entry = table[cmd];
  if (!entry) return null;
  const [label, fn] = entry;
  const btn = document.querySelector(`[data-cmd="${cmd}"]`);
  if (btn) btn.classList.add('busy');
  setResult(`${label} …`);
  const res = await fn();
  if (btn) btn.classList.remove('busy');
  showResult(label, res);
  return res;
}

async function estop() {
  const b = $('estopBtn');
  b.classList.remove('flash'); void b.offsetWidth; b.classList.add('flash');
  stopDriving(false);
  if (!gs.connected) { toast('E-STOP not sent: no link to the rover!', 'error'); return; }
  setResult('E-STOP sent …');
  const res = await gs.estop();
  showResult('E-STOP', res);
  if (!res.ok) toast('E-STOP NOT ACKNOWLEDGED — check the link / use the hardware stop.', 'error', 9000);
}

function setAutoPoll(on) {
  $('autoPoll').checked = on;
  gs.setAutoPoll(on ? Number($('pollInt').value) : 0);
  store.set('poll', on);
}

// ================================================================ drive
const drive = { keys: new Set(), pad: null, timer: null, last: '' };
const KEYMAP = { KeyW: 'f', ArrowUp: 'f', KeyS: 'b', ArrowDown: 'b', KeyA: 'l', ArrowLeft: 'l', KeyD: 'r', ArrowRight: 'r' };

function driveVector() {
  if (drive.pad) return drive.pad;
  let v = 0; let w = 0;
  if (drive.keys.has('f')) v += 1;
  if (drive.keys.has('b')) v -= 1;
  if (drive.keys.has('l')) w += 1;
  if (drive.keys.has('r')) w -= 1;
  return { v, w };
}

function sendDrive() {
  const { v, w } = driveVector();
  const vp = Math.round(v * Number($('vMax').value));
  const wp = Math.round(w * Number($('wMax').value));
  $('driveMeta').textContent = `DRIVE v ${vp}% · w ${wp}%`;
  gs.drive(vp, wp, 600).then((res) => {
    if (!res.ok && res.error === 'nack') showResult('DRIVE', res);
  });
}

function driveUpdate() {
  const { v, w } = driveVector();
  for (const b of document.querySelectorAll('.pad button')) {
    b.classList.toggle('active', !!(v || w) && Number(b.dataset.v) === v && Number(b.dataset.w) === w);
  }
  if (v || w) {
    if (!gs.connected) return;
    sendDrive();
    if (!drive.timer) drive.timer = setInterval(sendDrive, 250);
  } else {
    stopDriving(true);
  }
}

function stopDriving(sendStop) {
  const wasDriving = !!drive.timer;
  clearInterval(drive.timer);
  drive.timer = null;
  drive.keys.clear();
  drive.pad = null;
  for (const b of document.querySelectorAll('.pad button')) b.classList.remove('active');
  $('driveMeta').textContent = 'stopped';
  if (sendStop && wasDriving && gs.connected) gs.drive(0, 0, 0);
}

function bindPad() {
  for (const b of document.querySelectorAll('.pad button')) {
    const v = Number(b.dataset.v); const w = Number(b.dataset.w);
    b.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      b.setPointerCapture(e.pointerId);
      drive.pad = (v || w) ? { v, w } : null;
      driveUpdate();
    });
    const up = () => { if (drive.pad) { drive.pad = null; driveUpdate(); } };
    b.addEventListener('pointerup', up);
    b.addEventListener('pointercancel', up);
    b.addEventListener('lostpointercapture', up);
    b.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') { e.preventDefault(); drive.pad = (v || w) ? { v, w } : null; driveUpdate(); }
    });
    b.addEventListener('keyup', (e) => { if (e.key === 'Enter') up(); });
  }
  for (const [id, out] of [['vMax', 'vOut'], ['wMax', 'wOut']]) {
    $(id).addEventListener('input', () => { $(out).textContent = `${$(id).value} %`; store.set(id, $(id).value); });
    $(id).value = store.get(id, $(id).value);
    $(out).textContent = `${$(id).value} %`;
  }
}

// ============================================================ telemetry
const gauges = {};
const FAULT_SEV = { [Fault.ESTOP]: 'critical', [Fault.CMD_TIMEOUT]: 'warning', [Fault.LOW_BATTERY]: 'critical',
  [Fault.ENCODER]: 'serious', [Fault.SAFE_MODE]: 'warning' };
const ICON = {
  critical: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M5 1h6l4 4v6l-4 4H5l-4-4V5z" fill="currentColor"/></svg>',
  warning: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M8 1l7 14H1z" fill="currentColor"/></svg>',
  serious: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M8 1l7 14H1z" fill="currentColor"/></svg>',
  ok: '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M3 8.5l3 3 7-7" stroke="currentColor" stroke-width="2" fill="none"/></svg>',
};

function setGauge(id, value, decimals, status = 'normal') {
  const g = gauges[id];
  g.widget.set(value, status);
  g.el.querySelector('.v').textContent = fmt(value, decimals);
  if (value != null) g.el.setAttribute('aria-valuenow', value.toFixed(decimals));
  else g.el.removeAttribute('aria-valuenow');
}

function setState(el, state, text) {
  el.dataset.state = state;
  el.textContent = text;
}

function renderFaults(tlm) {
  const box = $('faultPills');
  if (!tlm || tlm.fault_flags == null) { box.innerHTML = ''; return; }
  const active = FAULT_LABELS.filter(([bit]) => tlm.fault_flags & bit);
  box.innerHTML = active.length
    ? active.map(([bit, , desc]) => `<span class="fault" data-sev="${FAULT_SEV[bit]}" title="${desc}">${ICON[FAULT_SEV[bit]]}${desc.split(' (')[0].toUpperCase()}</span>`).join('')
    : `<span class="fault" data-sev="ok">${ICON.ok}No faults</span>`;
}

function onTlm({ tlm, sample: s }) {
  ui.lastRx = Date.now();
  setGauge('gTemp', s.temperature_C, 1);
  setGauge('gHum', s.humidity_pct, 1);
  setGauge('gPres', s.pressure_hPa, 1);
  const low = s.battery_V != null && (s.battery_V < 10.8 || (s.fault_flags & Fault.LOW_BATTERY));
  setGauge('gBatt', s.battery_V, 2, low ? 'critical' : 'normal');
  $('battStatus').textContent = low ? 'LOW' : '';
  $('tlmVer').textContent = s.tlm_version === 2 ? 'TLM v2 · 56 B' : 'legacy TLM · 7 B';
  $('uptime').textContent = dur(s.uptime_s);
  $('modeText').textContent = s.mode == null ? '—' : modeName(s.mode);
  $('modePill').dataset.mode = s.mode == null ? '' : modeName(s.mode);
  renderFaults(s);
  // environment meters (legacy gas -> MQ-2 row)
  const meterVals = { mq2_raw: s.mq2_raw ?? s.gas_raw, mq4_raw: s.mq4_raw, soil_raw: s.soil_raw, rain_raw: s.rain_raw };
  for (const row of document.querySelectorAll('#meters .meter')) {
    const v = meterVals[row.dataset.key];
    const bar = row.querySelector('.m-bar');
    row.querySelector('i').style.width = v == null ? '0' : `${Math.min(100, (v / 1023) * 100).toFixed(1)}%`;
    row.querySelector('.m-val').textContent = v == null ? '—' : String(v);
    if (v != null) bar.setAttribute('aria-valuenow', String(v)); else bar.removeAttribute('aria-valuenow');
  }
  const prox = s.proximity ?? s.ir;
  if (prox == null) setState($('proxV'), 'none', '—');
  else if (prox) setState($('proxV'), 'warning', s.ir != null ? 'IR DETECT' : 'METAL DETECTED');
  else setState($('proxV'), 'good', 'clear');
  const r = s.range_min_m;
  $('rangeV').textContent = r == null ? '—' : `${r.toFixed(2)} m`;
  if (r == null) setState($('rangeState'), 'none', 'no echo');
  else if (r < 0.3) setState($('rangeState'), 'critical', 'TOO CLOSE');
  else if (r < 0.6) setState($('rangeState'), 'warning', 'CLOSE');
  else setState($('rangeState'), 'good', 'clear');
  // attitude + pose
  ui.att.set(s.roll_deg, s.pitch_deg);
  $('rollV').textContent = s.roll_deg == null ? '—' : `${fmt(s.roll_deg)}°`;
  $('pitchV').textContent = s.pitch_deg == null ? '—' : `${fmt(s.pitch_deg)}°`;
  $('yawV').textContent = s.yaw_deg == null ? '—' : `${fmt(s.yaw_deg)}°`;
  $('xV').textContent = s.x_m == null ? '—' : `${fmt(s.x_m, 2)} m`;
  $('yV').textContent = s.y_m == null ? '—' : `${fmt(s.y_m, 2)} m`;
  $('hdgV').textContent = s.heading_deg == null ? '—' : `${fmt(s.heading_deg)}°`;
  schedule(renderTrack);
  schedule(renderCharts);
  if (ui.view === 'table') schedule(renderTable);
}

function renderTrack() {
  const pts = gs.history.filter((s) => s.x_m != null && s.t >= ui.trackFrom).map((s) => ({ t: s.t, x: s.x_m, y: s.y_m }));
  const last = [...gs.history].reverse().find((s) => s.x_m != null);
  ui.track.set(pts, last ? { x: last.x_m, y: last.y_m, heading: last.heading_deg || 0 } : null);
}

function tickAge() {
  $('utcClock').textContent = hhmmss(new Date());
  if (gs.lastTlmAt) {
    const age = (Date.now() - gs.lastTlmAt) / 1000;
    $('tlmAge').textContent = age < 1.5 ? 'live' : `${age.toFixed(0)} s ago`;
    const stale = age > Math.max(10, 3 * (gs.pollIntervalS || 0));
    $('tlmAge').style.color = stale ? css('--warning') : '';
  }
  updatePill();
}

// =============================================================== charts
const CHARTS = [
  { title: 'Temperature', unit: '°C', decimals: 1, minSpan: 1, series: [['temperature_C', 'Temperature', '--series-1']] },
  { title: 'Humidity', unit: '%RH', decimals: 1, minSpan: 2, series: [['humidity_pct', 'Humidity', '--series-1']] },
  { title: 'Battery', unit: 'V', decimals: 2, minSpan: 0.2, series: [['battery_V', 'Battery', '--series-1']] },
  { title: 'Gas', unit: 'ADC', decimals: 0, minSpan: 20, yMin: 0,
    series: [['mq2_raw', 'MQ-2', '--series-1'], ['mq4_raw', 'MQ-4', '--series-2'], ['gas_raw', 'Gas (legacy)', '--series-3']] },
  { title: 'Roll / pitch', unit: '°', decimals: 1, minSpan: 2,
    series: [['roll_deg', 'Roll', '--series-1'], ['pitch_deg', 'Pitch', '--series-2']] },
  { title: 'Closest obstacle', unit: 'm', decimals: 2, minSpan: 0.5, yMin: 0, series: [['range_min_m', 'Range', '--series-1']] },
];

function buildCharts() {
  const box = $('charts');
  box.innerHTML = '';
  ui.charts = CHARTS.map((c, i) => {
    const card = document.createElement('div');
    card.className = 'chart-card';
    card.innerHTML = `<h3>${c.title} <span class="unit">${c.unit}</span><span class="legend"></span></h3>` +
      `<div class="chart-wrap"><canvas aria-label="${c.title} history chart; focus and use arrow keys to read values"></canvas>` +
      '<div class="tip" hidden></div></div>';
    box.appendChild(card);
    const series = c.series.map(([key, label, color]) => ({ key, label, color: css(color) }));
    const chart = new TimeChart(card.querySelector('canvas'), { series, decimals: c.decimals, unit: c.unit,
      minSpan: c.minSpan, yMin: c.yMin ?? null, tip: card.querySelector('.tip') });
    chart.windowS = ui.windowS;
    return { chart, legend: card.querySelector('.legend'), series, id: i, legendKey: '' };
  });
}

function renderCharts() {
  if (ui.view !== 'charts') return;
  for (const c of ui.charts) {
    const present = c.series.filter((s) => gs.history.some((h) => h[s.key] != null));
    const key = present.map((s) => s.key).join();
    if (key !== c.legendKey) {
      c.legendKey = key;
      c.legend.innerHTML = present.length > 1
        ? present.map((s) => `<span><i class="sw" style="background:${s.color}"></i>${s.label}</span>`).join('') : '';
    }
    c.chart.setData(gs.history);
  }
}

function setWindow(s) {
  ui.windowS = s;
  store.set('win', s);
  for (const b of $('winSeg').querySelectorAll('button')) b.setAttribute('aria-checked', String(Number(b.dataset.win) === s));
  for (const c of ui.charts) c.chart.setWindow(s);
  if (ui.view === 'table') renderTable();
}

function setView(v) {
  ui.view = v;
  for (const b of $('viewSeg').querySelectorAll('button')) b.setAttribute('aria-checked', String(b.dataset.view === v));
  $('charts').hidden = v !== 'charts';
  $('tableView').hidden = v !== 'table';
  if (v === 'table') renderTable(); else renderCharts();
}

const TABLE_COLS = [['Time (UTC)', (s) => hhmmss(new Date(s.t))], ['Mode', (s) => (s.mode == null ? '—' : modeName(s.mode))],
  ['Batt V', (s) => fmt(s.battery_V, 2)], ['T °C', (s) => fmt(s.temperature_C)], ['RH %', (s) => fmt(s.humidity_pct)],
  ['p hPa', (s) => fmt(s.pressure_hPa)], ['Roll °', (s) => fmt(s.roll_deg)], ['Pitch °', (s) => fmt(s.pitch_deg)],
  ['MQ-2', (s) => (s.mq2_raw ?? s.gas_raw ?? '—')], ['MQ-4', (s) => (s.mq4_raw ?? '—')], ['Soil', (s) => (s.soil_raw ?? '—')],
  ['Rain', (s) => (s.rain_raw ?? '—')], ['x m', (s) => fmt(s.x_m, 2)], ['y m', (s) => fmt(s.y_m, 2)],
  ['Hdg °', (s) => fmt(s.heading_deg)], ['Range m', (s) => fmt(s.range_min_m, 2)]];

function renderTable() {
  const t = $('tableView').querySelector('table');
  t.tHead.innerHTML = `<tr>${TABLE_COLS.map(([h]) => `<th scope="col">${h}</th>`).join('')}</tr>`;
  const from = ui.windowS ? Date.now() - ui.windowS * 1000 : 0;
  const rows = gs.history.filter((s) => s.t >= from).slice(-200).reverse();
  t.tBodies[0].innerHTML = rows.map((s) => `<tr>${TABLE_COLS.map(([, f]) => `<td>${f(s)}</td>`).join('')}</tr>`).join('') ||
    `<tr><td colspan="${TABLE_COLS.length}">No telemetry in this window.</td></tr>`;
}

// ================================================================ image
function renderChunkMap(states) {
  const map = $('chunkMap');
  if (!states) return;
  if (map.children.length !== states.length) {
    map.innerHTML = states.map(() => '<i></i>').join('');
    map.style.gridTemplateColumns = `repeat(${Math.min(25, Math.max(10, states.length))}, minmax(0, 1fr))`;
  }
  states.forEach((st, i) => { map.children[i].className = st === 'ok' ? 'ok' : st === 'missing' ? 'missing' : ''; });
  const ok = states.filter((s) => s === 'ok').length; const miss = states.filter((s) => s === 'missing').length;
  map.setAttribute('aria-label', `Image chunks: ${ok} received, ${miss} missing, ${states.length - ok - miss} pending`);
}

function drawGray(canvas, im, highlightMissing = true) {
  canvas.width = im.width; canvas.height = im.height;
  const ctx = canvas.getContext('2d');
  const img = ctx.createImageData(im.width, im.height);
  const size = 200;
  const missing = new Set(highlightMissing ? im.missing : []);
  for (let i = 0; i < im.data.length && i < im.width * im.height; i++) {
    const v = im.data[i];
    const bad = missing.has(Math.floor(i / size));
    img.data[4 * i] = bad ? 110 : v;
    img.data[4 * i + 1] = bad ? 24 : v;
    img.data[4 * i + 2] = bad ? 28 : v;
    img.data[4 * i + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

async function drawImage(canvas, im) {
  if (im.fmt === 0) { drawGray(canvas, im); canvas.classList.remove('smooth'); return true; }
  try {
    const bmp = await createImageBitmap(new Blob([im.data], { type: 'image/jpeg' }));
    canvas.width = bmp.width; canvas.height = bmp.height;
    canvas.getContext('2d').drawImage(bmp, 0, 0);
    canvas.classList.add('smooth');
    return true;
  } catch (e) {
    return false;
  }
}

function onImageProgress(p) {
  if (!p) return;
  $('imgEmpty').hidden = true;
  const pct = p.total ? (100 * p.received) / p.total : 0;
  $('imgBar').style.width = `${pct.toFixed(1)}%`;
  const kind = p.legacy == null ? 'waiting for first chunk' : p.legacy ? 'legacy' : (p.fmt ? 'JPEG' : 'gray8');
  $('imgMeta').textContent = p.total ? `${kind} · ${p.received}/${p.total} · ${(p.bytes / 1024).toFixed(1)} kB · ${p.elapsed.toFixed(1)} s`
    : kind;
  renderChunkMap(p.states);
  if (gs.image && gs.image.fmt === 0 && p.total) drawGray($('imgCanvas'), gs.image.result(0, 'partial'), false);
}

async function showImage(idx) {
  const im = gs.images[idx];
  if (!im) return;
  ui.imageIdx = idx;
  $('imgEmpty').hidden = true;
  const ok = await drawImage($('imgCanvas'), im);
  const kind = im.legacy ? 'legacy' : (im.fmt ? 'JPEG' : 'gray8');
  $('imgMeta').textContent = `${kind} ${im.width}×${im.height} · ${im.received.length}/${im.total}` +
    `${im.missing.length ? ` · ${im.missing.length} missing` : ''} · ${(im.elapsed || 0).toFixed(1)} s`;
  if (!ok) $('imgMeta').textContent += ' · JPEG not decodable';
  $('imgBar').style.width = `${((100 * im.received.length) / Math.max(1, im.total)).toFixed(1)}%`;
  renderChunkMap(im.states || Array.from({ length: im.total }, (_, i) => (im.missing.includes(i) ? 'missing' : 'ok')));
  $('btnSaveImg').disabled = false;
  renderThumbs();
}

function renderThumbs() {
  const box = $('thumbs');
  box.innerHTML = '';
  gs.images.slice(-8).forEach((im, k, arr) => {
    const idx = gs.images.length - arr.length + k;
    const b = document.createElement('button');
    b.type = 'button';
    b.title = `${new Date(im.t).toISOString().slice(11, 19)} UTC · ${im.width}×${im.height}`;
    b.setAttribute('aria-label', `Show image ${idx + 1}`);
    b.setAttribute('aria-current', String(idx === ui.imageIdx));
    const c = document.createElement('canvas');
    b.appendChild(c);
    drawImage(c, im);
    b.addEventListener('click', () => showImage(idx));
    box.appendChild(b);
  });
}

// ================================================================== log
const LOG_DOM_MAX = 400;
let logQueue = [];

function logMatches(e) {
  if (!$(`f${e.dir}`).checked) return false;
  if ($('fPoll').checked && e.type === PT.GET_TLM) return false;
  const q = $('logFilter').value.trim().toLowerCase();
  if (!q) return true;
  return `${e.name} ${e.hex} ${e.text}`.toLowerCase().includes(q);
}

function hexDump(hex) {
  const b = hex.split(' ').filter(Boolean);
  const lines = [];
  for (let i = 0; i < b.length; i += 16) lines.push(`${i.toString(16).padStart(4, '0')}  ${b.slice(i, i + 16).join(' ')}`);
  return lines.join('\n');
}

function logRow(e) {
  const row = document.createElement('div');
  row.className = `log-row${e.dir === 'ERR' || e.level === 'error' ? ' err' : ''}${e.level === 'warn' ? ' warn' : ''}`;
  row.innerHTML = `<span class="lt">${hhmmssms(e.t).slice(0, 12)}</span><span class="ld ${e.dir}">${e.dir}</span>` +
    `<span class="lx"><span class="lm">${escapeHtml(e.text)}</span>${e.hex ? `<span class="lh">${e.hex}</span>` : ''}</span>`;
  if (e.hex) {
    row.title = 'Click for a hex dump';
    row.addEventListener('click', () => {
      const open = row.classList.toggle('open');
      row.querySelector('.lh').textContent = open ? hexDump(e.hex) : e.hex;
    });
  }
  return row;
}

function flushLog() {
  const list = $('logList');
  const atBottom = list.scrollHeight - list.scrollTop - list.clientHeight < 30;
  const frag = document.createDocumentFragment();
  for (const e of logQueue) if (logMatches(e)) frag.appendChild(logRow(e));
  logQueue = [];
  list.appendChild(frag);
  while (list.children.length > LOG_DOM_MAX) list.firstChild.remove();
  if (atBottom) list.scrollTop = list.scrollHeight;
  $('logCount').textContent = `${gs.log.length} entries`;
}

function rerenderLog() {
  $('logList').innerHTML = '';
  logQueue = gs.log.filter(logMatches).slice(-LOG_DOM_MAX);
  flushLog();
  $('logList').scrollTop = $('logList').scrollHeight;
}

// ================================================================= data
function exportCsv() {
  if (!gs.history.length) { toast('No telemetry to export yet.', 'warn'); return; }
  download(new Blob([gs.telemetryCsv()], { type: 'text/csv' }), `missionx_tlm_${stamp()}.csv`);
}

function settings() {
  return { source: ui.source, baud: Number($('baud').value), profile: $('vProfile').value, ber: $('vBer').value,
    poll: Number($('pollInt').value), window: ui.windowS };
}

function saveSession() {
  const blob = new Blob([JSON.stringify(gs.exportSession(settings()))], { type: 'application/json' });
  download(blob, `missionx_session_${stamp()}.json`);
}

async function loadSession(file) {
  try {
    const obj = JSON.parse(await file.text());
    const st = gs.importSession(obj);
    if (st.window != null) setWindow(st.window);
    if (st.baud) $('baud').value = String(st.baud);
    const last = [...gs.history].reverse()[0];
    if (last) onTlm({ tlm: null, sample: last });
    gs.lastTlmAt = last ? last.t : null;
    renderCharts(); renderTrack(); rerenderLog(); renderThumbs();
    if (gs.images.length) showImage(gs.images.length - 1);
    toast(`Session loaded: ${gs.history.length} samples, ${gs.images.length} images.`);
  } catch (e) {
    toast(`Cannot load session: ${e.message}`, 'error');
  }
}

// ============================================================ keyboard
function isTyping(t) {
  return t && (t.tagName === 'INPUT' && !['checkbox', 'radio', 'range'].includes(t.type)) || (t && (t.tagName === 'TEXTAREA' || t.isContentEditable));
}

function onKeyDown(e) {
  if ($('helpDlg').open && e.key !== ' ') return;
  if (isTyping(e.target)) { if (e.key === 'Escape') e.target.blur(); return; }
  if (e.code === 'Space') { e.preventDefault(); if (!e.repeat) estop(); return; }
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  if (KEYMAP[e.code] && !(e.target && e.target.tagName === 'CANVAS' && e.code.startsWith('Arrow')) &&
      !(e.target && e.target.tagName === 'SELECT' && e.code.startsWith('Arrow'))) {
    e.preventDefault();
    if (!e.repeat) { drive.keys.add(KEYMAP[e.code]); driveUpdate(); }
    return;
  }
  if (e.repeat) return;
  const k = e.key;
  const map = { p: 'ping', t: 'tlm', c: 'cap', i: 'img', g: 'scan', h: 'stop', o: 'collect' };
  if (k === 'C' && e.shiftKey) { run('clear'); e.preventDefault(); return; }
  if (map[k]) { run(map[k]); e.preventDefault(); return; }
  if (k === 'r') { setAutoPoll(!$('autoPoll').checked); e.preventDefault(); return; }
  if (k === 'm') { $('modeSel').focus(); e.preventDefault(); return; }
  if (k === '?' || (e.code === 'Slash' && e.shiftKey)) { $('helpDlg').showModal(); e.preventDefault(); return; }
  if (k === '/') { $('logFilter').focus(); e.preventDefault(); }
}

function onKeyUp(e) {
  if (KEYMAP[e.code] && drive.keys.has(KEYMAP[e.code])) {
    drive.keys.delete(KEYMAP[e.code]);
    driveUpdate();
  }
}

// ================================================================ init
function init() {
  // gauges
  for (const [id, opts] of [['gTemp', { min: 0, max: 50 }], ['gHum', { min: 0, max: 100 }],
    ['gPres', { min: 950, max: 1050 }], ['gBatt', { min: 10, max: 12.6, threshold: 10.8 }]]) {
    const el = $(id);
    gauges[id] = { el, widget: new ArcGauge(el.querySelector('canvas'), opts) };
    gauges[id].widget.draw();
  }
  ui.att = new AttitudeIndicator($('attCanvas'));
  ui.att.draw();
  ui.track = new TrackPlot($('trackCanvas'), { tip: document.querySelector('.track-wrap .tip') });
  ui.track.draw();
  buildCharts();
  setWindow(ui.windowS);

  // link panel
  for (const b of $('srcSeg').querySelectorAll('button')) b.addEventListener('click', () => setSource(b.dataset.src));
  setSource(ui.source);
  $('btnConnect').addEventListener('click', toggleConnect);
  $('baud').value = String(store.get('baud', 9600));
  $('baud').addEventListener('change', () => store.set('baud', Number($('baud').value)));
  $('vBer').value = store.get('ber', '1e-5');
  $('vBer').addEventListener('change', () => {
    store.set('ber', $('vBer').value);
    if (gs.link && gs.link.setBer) { gs.link.setBer(Number($('vBer').value)); gs.sys(`virtual link BER set to ${$('vBer').value}`); }
  });
  $('vProfile').value = store.get('profile', 'v2');
  $('vProfile').addEventListener('change', () => store.set('profile', $('vProfile').value));
  for (const id of ['timeoutSel', 'ibtSel']) $(id).addEventListener('change', applyLinkSettings);

  // commands
  for (const b of document.querySelectorAll('[data-cmd]')) b.addEventListener('click', () => run(b.dataset.cmd));
  $('estopBtn').addEventListener('click', estop);
  $('autoPoll').addEventListener('change', () => setAutoPoll($('autoPoll').checked));
  $('pollInt').value = String(store.get('pollInt', 1));
  $('pollInt').addEventListener('change', () => {
    store.set('pollInt', Number($('pollInt').value));
    if ($('autoPoll').checked) gs.setAutoPoll(Number($('pollInt').value));
  });
  bindPad();

  // charts / table
  for (const b of $('winSeg').querySelectorAll('button')) b.addEventListener('click', () => setWindow(Number(b.dataset.win)));
  for (const b of $('viewSeg').querySelectorAll('button')) b.addEventListener('click', () => setView(b.dataset.view));
  $('btnClearTrack').addEventListener('click', () => { ui.trackFrom = Date.now(); renderTrack(); });

  // image
  $('btnSaveImg').addEventListener('click', () => {
    const im = gs.images[ui.imageIdx];
    if (!im) return;
    $('imgCanvas').toBlob((b) => download(b, `missionx_image_${stamp()}.png`), 'image/png');
  });

  // log
  for (const id of ['fTX', 'fRX', 'fERR', 'fSYS', 'fPoll']) $(id).addEventListener('change', rerenderLog);
  $('logFilter').addEventListener('input', rerenderLog);
  $('btnPause').addEventListener('click', () => {
    ui.logPaused = !ui.logPaused;
    $('btnPause').textContent = ui.logPaused ? 'Resume' : 'Pause';
    $('btnPause').setAttribute('aria-pressed', String(ui.logPaused));
    if (!ui.logPaused) rerenderLog();
  });
  $('btnClearLog').addEventListener('click', () => { gs.log = []; rerenderLog(); });
  $('btnCsv').addEventListener('click', exportCsv);
  $('btnSaveSession').addEventListener('click', saveSession);
  $('btnLoadSession').addEventListener('click', () => $('fileSession').click());
  $('fileSession').addEventListener('change', (e) => { if (e.target.files[0]) loadSession(e.target.files[0]); e.target.value = ''; });

  // help + keyboard
  $('helpBtn').addEventListener('click', () => $('helpDlg').showModal());
  window.addEventListener('keydown', onKeyDown, true);
  window.addEventListener('keyup', onKeyUp, true);
  window.addEventListener('blur', () => stopDriving(true));
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopDriving(true); });

  // ground-station events
  gs.on('tlm', onTlm);
  gs.on('log', (e) => { if (!ui.logPaused) { logQueue.push(e); schedule(flushLog); } });
  gs.on('rx', () => { ui.lastRx = Date.now(); schedule(renderStats); });
  gs.on('reply', () => schedule(renderStats));
  gs.on('queue', () => schedule(renderStats));
  gs.on('image-progress', (p) => onImageProgress(p));
  gs.on('image', () => showImage(gs.images.length - 1));
  gs.on('link', (ev) => {
    if (!ev.connected && ev.reason && ev.reason !== 'closed') {
      ui.lostLink = true;
      toast(`Link lost: ${ev.reason}`, 'error');
    }
    stopDriving(false);
    updateLinkUi();
    renderStats();
  });

  updateLinkUi();
  renderStats();
  setInterval(tickAge, 500);
  tickAge();
  if (store.get('poll', false)) $('autoPoll').checked = true;

  const params = new URLSearchParams(location.search);
  if (params.has('demo')) demo(params);
  window.missionx = { gs, ui, run, estop, setAutoPoll, setSource, toggleConnect, setView, setWindow };
}

/** ?demo -> connect the virtual rover and run a short scripted mission. */
async function demo(params) {
  setSource('virtual');
  if (params.get('ber')) $('vBer').value = params.get('ber');
  if (params.get('win')) setWindow(Number(params.get('win')));
  await toggleConnect();
  $('pollInt').value = params.get('poll') || '1';
  setAutoPoll(true);
  const wait = (s) => new Promise((r) => setTimeout(r, s * 1000));
  await run('ping');
  await wait(1);
  await run('scan');
  await wait(Number(params.get('explore') || 20));
  await run('cap');
  await run('img');
  await run('ping');
}

init();
