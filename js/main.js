// Dronee — F450 simulator with Pixhawk hardware-in-the-loop and a
// QGroundControl-style ground station, running entirely in the browser.

import { GeoOrigin } from './core/geo.js';
import { DEG, qFromEuler, qToEuler } from './core/math.js';
import { Quadcopter } from './sim/quadcopter.js';
import { Autopilot, MODES, MANUAL_MODES } from './sim/autopilot.js';
import { SensorSuite } from './sim/sensors.js';
import { InputManager } from './input/input.js';
import { SerialTransport, WebSocketTransport } from './link/transports.js';
import { MavlinkVehicle } from './link/mavlinkVehicle.js';
import { SceneView, GEAR_HEIGHT } from './render/scene.js';
import { GroundStation } from './ui/gcs.js';
import { drawAttitude, drawCompass } from './ui/instruments.js';

const $ = (id) => document.getElementById(id);
const SETTINGS_KEY = 'dronee.settings.v1';
const KFUPM = { lat: 26.30706, lon: 50.14527, alt: 60 };
const DT = 0.002; // physics step (500 Hz)

// ------------------------------------------------------------------ settings
const defaults = {
  quality: matchMedia('(pointer: coarse)').matches ? 'medium' : 'high',
  home: KFUPM, hour: 10.5, wind: 0, windDir: 45, gust: 0, noise: 1, trail: true, sticks: true, helpSeen: false,
};
let settings = { ...defaults };
try { settings = { ...defaults, ...JSON.parse(localStorage.getItem(SETTINGS_KEY) || '{}') }; } catch { /* ignore */ }
const saveSettings = () => { try { localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings)); } catch { /* ignore */ } };

// ------------------------------------------------------------------ internal vehicle adapter
class InternalVehicle {
  constructor(ap) { this.ap = ap; }
  get modes() { return MODES; }
  telemetry() { return this.ap.telemetry(); }
  arm() { return Promise.resolve(this.ap.arm()); }
  disarm() { this.ap.disarm(); return Promise.resolve(true); }
  setMode(m) { return Promise.resolve(this.ap.setMode(m)); }
  takeoff(alt) { return Promise.resolve(this.ap.takeoff(alt)); }
  land() { return Promise.resolve(this.ap.land()); }
  rtl() { return Promise.resolve(this.ap.rtl()); }
  startMission() { return Promise.resolve(this.ap.startMission()); }
  gotoLocation(lat, lon, alt) { return Promise.resolve(this.ap.gotoLocation(lat, lon, alt)); }
  uploadMission(items, onProgress) {
    this.ap.setMission(items);
    onProgress?.(1);
    return Promise.resolve(true);
  }
  downloadMission() { return Promise.resolve(this.ap.mission.map((i) => ({ ...i }))); }
}

// ------------------------------------------------------------------ app
const app = {
  view: 'sim',
  linkMode: 'sim', // sim | rc | hitl | monitor
  link: null,
  settings,
  log(text, severity = 6) {
    const box = $('messages');
    const d = document.createElement('div');
    d.className = severity <= 3 ? 'bad' : severity === 4 ? 'warn' : '';
    d.textContent = text;
    box.appendChild(d);
    setTimeout(() => d.remove(), 6000);
    while (box.children.length > 4) box.firstChild.remove();
    app.gcs?.log(text, severity);
  },
  homeGeo() {
    const t = app.vehicle?.telemetry();
    if (t?.home && t.home.lat) return t.home;
    return { lat: app.geo.lat, lon: app.geo.lon, alt: app.geo.alt };
  },
};
window.dronee = app; // handy for debugging from the console

const geo = new GeoOrigin(settings.home.lat, settings.home.lon, settings.home.alt);
app.geo = geo;

const view = new SceneView($('view3d'), settings.quality);
app.scene = view;
view.world.setTimeOfDay(settings.hour);
view.showTrail = settings.trail;

const quad = new Quadcopter();
const groundAt = (n, e) => {
  const h = view.terrainHeightNed(n, e);
  return h;
};
const resetVehicle = () => {
  quad.reset([0, 0, -(groundAt(0, 0) + GEAR_HEIGHT)], 0);
  quad.groundHeight = groundAt(0, 0) + GEAR_HEIGHT;
};
resetVehicle();
const ap = new Autopilot(quad, geo);
ap.home = [...quad.pos];
const sensors = new SensorSuite(geo, { noise: settings.noise });
const input = new InputManager();
app.input = input;
app.internal = new InternalVehicle(ap);
app.vehicle = app.internal;
ap.on('text', (t, s) => { if (app.vehicle === app.internal) app.log(t, s); });
ap.on('mode', () => refreshModeSelect(true));

if (matchMedia('(pointer: coarse)').matches) document.body.classList.add('touch');
input.attachTouchSticks($('touchL'), $('touchR'));

const gcs = new GroundStation(app);
app.gcs = gcs;
app.onPlanChanged = (plan) => {
  const pts = [];
  let idx = 0;
  for (const it of plan) {
    idx++;
    if (it.lat) {
      const ned = geo.toNed(it.lat, it.lon, geo.alt + (it.alt || 0));
      ned[2] -= GEAR_HEIGHT;
      pts.push({ ned, label: idx, color: it.command === 21 ? 0xff453a : 0x2e9df7 });
    }
  }
  view.setMissionMarkers(pts);
};

// ------------------------------------------------------------------ environment settings
const windNED = () => {
  const from = settings.windDir * DEG;
  return [-Math.cos(from) * settings.wind, -Math.sin(from) * settings.wind, 0];
};

// ------------------------------------------------------------------ simulation loop
let simTime = 0;
let lastTick = performance.now();
let acc = 0;
let hilSensorAcc = 0;
let hilGpsAcc = 0;
let manualAcc = 0;
let lastActSeq = -1;
let actStale = 0;
let crashedShown = false;
let gestureT = 0;
let lastModeSwitch = null;
let lastArmSwitch = null;
let padPrev = [];

function physicsTick() {
  const now = performance.now();
  const elapsed = Math.min((now - lastTick) / 1000, 0.25);
  lastTick = now;
  acc += elapsed;
  const sticks = input.update(elapsed);
  handlePilotCommands(sticks, elapsed);

  quad.wind = windNED();
  quad.windGust = settings.gust;
  let steps = 0;
  while (acc >= DT && steps < 150) {
    acc -= DT;
    steps++;
    simTime += DT;
    quad.groundHeight = Math.max(groundAt(quad.pos[0], quad.pos[1]), -1.2) + GEAR_HEIGHT;
    if (app.linkMode === 'monitor') continue;
    if (app.linkMode === 'hitl') {
      // motor outputs from the flight controller; cut them if it stops answering
      // for 0.5 s of *simulated* time (robust against browser stalls)
      if (app.link.actuatorsSeq !== lastActSeq) { lastActSeq = app.link.actuatorsSeq; actStale = 0; } else actStale += DT;
      quad.setMotors(actStale < 0.5 ? app.link.actuators : [0, 0, 0, 0]);
      quad.step(DT);
      hilSensorAcc += DT;
      hilGpsAcc += DT;
      if (hilSensorAcc >= 0.004) {
        hilSensorAcc -= 0.004;
        const us = Math.round(simTime * 1e6);
        app.link.sendHilSensor(us, sensors.imu(quad), sensors.baro(quad, 0.004));
      }
      if (hilGpsAcc >= 0.1) {
        hilGpsAcc -= 0.1;
        app.link.sendHilGps(Math.round(simTime * 1e6), sensors.gps(quad, 0.1));
      }
    } else {
      ap.update(DT, sticks);
      quad.step(DT);
    }
  }
  if (steps >= 150) acc = 0;

  // HITL without an RC receiver: forward keyboard / gamepad as MANUAL_CONTROL
  if (app.linkMode === 'hitl' && input.source !== 'rc') {
    manualAcc += elapsed;
    if (manualAcc > 0.04) {
      manualAcc = 0;
      app.link.send('MANUAL_CONTROL', {
        target: app.link.target.sys,
        x: Math.round(-sticks.pitch * 1000), y: Math.round(sticks.roll * 1000),
        z: Math.round(sticks.throttle * 1000), r: Math.round(sticks.yaw * 1000), buttons: 0,
      });
    }
  }

  if (quad.crashed && !crashedShown) {
    crashedShown = true;
    if (app.linkMode !== 'hitl') ap.disarm('crash');
    $('bigStatus').innerHTML = 'CRASHED<small>Press Backspace or the Reset button to try again</small>';
    app.log('Vehicle crashed!', 2);
  }
}
setInterval(physicsTick, 4);

function handlePilotCommands(sticks, dt) {
  if (app.linkMode === 'hitl' || app.linkMode === 'monitor') return;
  // stick-gesture arming: throttle low + yaw right / left for 1 s
  const low = sticks.throttle < 0.06;
  if (low && Math.abs(sticks.yaw) > 0.85 && (input.source === 'rc' || input.source === 'gamepad')) {
    gestureT += dt;
    if (gestureT > 1) {
      gestureT = -2;
      if (sticks.yaw > 0 && !ap.armed) ap.arm(true);
      else if (sticks.yaw < 0 && ap.armed && quad.onGround) ap.disarm('stick gesture');
    }
  } else if (gestureT > 0) gestureT = 0;
  else if (gestureT < 0) gestureT = Math.min(0, gestureT + dt);

  // RC mode switch (3 position) and arm switch
  if (input.source === 'rc') {
    if (input.modeSwitch !== null && input.modeSwitch !== lastModeSwitch) {
      lastModeSwitch = input.modeSwitch;
      ap.setMode(['Stabilized', 'Altitude', 'Position'][input.modeSwitch]);
    }
    if (input.armSwitch !== null && input.armSwitch !== lastArmSwitch) {
      if (lastArmSwitch !== null) { if (input.armSwitch) ap.arm(); else ap.disarm('switch'); }
      lastArmSwitch = input.armSwitch;
    }
  }
  // gamepad buttons (standard mapping): A arm/disarm, B land, X takeoff, Y return
  if (input.source === 'gamepad' && input.padButtons) {
    const b = input.padButtons;
    const edge = (i) => b[i] && !padPrev[i];
    if (input.gamepad()?.mapping === 'standard') {
      if (edge(0)) (ap.armed ? ap.disarm() : ap.arm());
      if (edge(1)) ap.land();
      if (edge(2)) ap.takeoff();
      if (edge(3)) ap.rtl();
    }
    padPrev = [...b];
  }
  // keyboard throttle behaviour follows the flight mode
  const centered = !['Stabilized', 'Acro'].includes(ap.mode);
  if (centered !== input.centeredThrottle) {
    input.setThrottleCentered(centered);
    if (!centered && !quad.onGround) { input.kbThrottle = ap.throttleOut / 0.9; input.sticks.throttle = input.kbThrottle; }
  }
}

// ------------------------------------------------------------------ render loop
let lastFrame = performance.now();
let uiAcc = 0;
function frame() {
  const now = performance.now();
  const dt = Math.min((now - lastFrame) / 1000, 0.1);
  lastFrame = now;
  let state = {
    pos: quad.pos, q: quad.q, motorOut: quad.motorOut, propAngle: quad.propAngle,
    armed: ap.armed, wind: windNED(),
  };
  if (app.linkMode === 'hitl') state.armed = app.link.state.armed;
  if (app.linkMode === 'monitor' && app.link) {
    const t = app.link.state;
    state = { ...state, pos: [0, 0, -(1.3 + GEAR_HEIGHT)], q: qFromEuler(t.roll, t.pitch, t.yaw), motorOut: t.motors, armed: t.armed };
    for (let i = 0; i < 4; i++) quad.propAngle[i] += (t.motors[i] || 0) * dt * 300 * (i < 2 ? -1 : 1);
  }
  view.updateVehicle(state, dt);
  // the 3D view is hidden behind the plan / analyze / setup panels
  if (!['plan', 'analyze', 'setup'].includes(app.view)) view.render();
  uiAcc += dt;
  if (uiAcc > 0.1) { uiAcc = 0; updateUi(); }
  requestAnimationFrame(frame);
}

// ------------------------------------------------------------------ UI updates
const fmtTime = (s) => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(Math.floor(s % 60)).padStart(2, '0')}`;
let flightStart = null;

function telemetry() {
  const t = app.vehicle.telemetry();
  if (app.linkMode === 'hitl') {
    t.motors = app.link.hilActuators;
    if (t.armed && !t.landed) { flightStart ??= performance.now(); t.flightTime = (performance.now() - flightStart) / 1000; } else if (!t.armed) flightStart = null;
  }
  return t;
}

function updateUi() {
  const t = telemetry();
  // toolbar
  const pill = $('readyPill');
  const notReady = quad.crashed || (app.linkMode !== 'sim' && app.linkMode !== 'rc' && !app.link?.connected);
  pill.textContent = t.armed ? (t.landed ? 'Armed' : 'Flying') : notReady ? 'Not Ready' : 'Ready To Fly';
  pill.className = `pill ${t.armed ? 'armed' : notReady ? 'warn' : ''}`;
  if ($('modeSelect').value !== t.mode) refreshModeSelect();
  $('indGps').querySelector('b').textContent = t.gps ? `${t.gps.sats}` : '—';
  const pct = t.battery?.remaining;
  $('indBat').querySelector('b').textContent = pct >= 0 ? `${Math.round(pct * 100)}%` : `${(t.battery?.voltage || 0).toFixed(1)}V`;
  const srcNames = { keyboard: 'KB', gamepad: 'PAD', rc: 'RC', touch: 'TOUCH' };
  $('indInput').querySelector('b').textContent = srcNames[input.source] || input.source;
  $('indLink').querySelector('b').textContent = { sim: 'SIM', rc: 'PIXHAWK RC', hitl: 'HITL', monitor: 'TELEM' }[app.linkMode];

  // instruments
  if (app.view === 'sim' || app.view === 'fly') {
    drawAttitude($('attCanvas'), t.roll, t.pitch);
    const home = t.home;
    let homeBearing = null;
    if (home && t.distHome > 2) {
      const dN = (home.lat - t.lat) * 111320, dE = (home.lon - t.lon) * 111320 * Math.cos(t.lat * DEG);
      homeBearing = ((Math.atan2(dE, dN) / DEG) + 360) % 360;
    }
    drawCompass($('compassCanvas'), t.heading, { homeBearing });
    $('vAlt').textContent = t.altRel.toFixed(1);
    $('vClimb').textContent = t.climb.toFixed(1);
    $('vSpd').textContent = t.groundSpeed.toFixed(1);
    $('vHome').textContent = (t.distHome || 0).toFixed(0);
    $('vTime').textContent = fmtTime(t.flightTime || 0);
    $('vBat').textContent = (t.battery?.voltage || 0).toFixed(1);
    $('vHdg').textContent = Math.round(t.heading);
    $('vThr').textContent = Math.round((t.throttle || 0) * 100);
  }
  // sticks monitor
  const s = input.sticks;
  $('stickL').style.left = `${50 + s.yaw * 42}%`;
  $('stickL').style.top = `${50 - (s.throttle - 0.5) * 84}%`;
  $('stickR').style.left = `${50 + s.roll * 42}%`;
  $('stickR').style.top = `${50 + s.pitch * 42}%`;
  $('stickSrc').textContent = { keyboard: 'Keyboard', gamepad: 'Gamepad', rc: 'RC (Pixhawk)', touch: 'Touch' }[input.source];

  // fly tools
  $('toolArm').querySelector('span').textContent = t.armed ? 'Disarm' : 'Arm';
  $('toolArm').classList.toggle('on', t.armed);
  $('toolReset').style.display = app.linkMode === 'monitor' ? 'none' : '';

  gcs.updateMap(t);
  gcs.pushHistory(t);
  if (app.view === 'analyze') gcs.updateAnalyze(t);
  if (app.view === 'setup') gcs.updateSetup();

  if (app.linkMode === 'hitl' && document.hidden) app.log('Keep this tab visible during HITL — browsers slow down background tabs', 4);
}

let modeListKey = '';
function refreshModeSelect(force) {
  const sel = $('modeSelect');
  const modes = app.vehicle.modes;
  const t = app.vehicle.telemetry();
  const key = modes.join();
  if (key !== modeListKey || force) {
    modeListKey = key;
    const list = modes.includes(t.mode) ? modes : [...modes, t.mode];
    sel.innerHTML = list.map((m) => `<option>${m}</option>`).join('');
  }
  if (![...sel.options].some((o) => o.value === t.mode)) sel.add(new Option(t.mode));
  sel.value = t.mode;
}
$('modeSelect').onchange = (e) => { app.vehicle.setMode(e.target.value); };

// ------------------------------------------------------------------ views
function setView(v) {
  app.view = v;
  const el = $('app');
  el.classList.remove('view-sim', 'view-fly', 'view-plan', 'view-analyze', 'view-setup');
  el.classList.add(`view-${v}`);
  el.classList.toggle('map-main', v === 'fly' || v === 'plan');
  document.querySelectorAll('#viewTabs button').forEach((b) => b.classList.toggle('active', b.dataset.view === v));
  gcs.invalidate();
  gcs.renderPlan();
  if (v === 'setup') gcs.renderPadMap();
}
document.querySelectorAll('#viewTabs button').forEach((b) => { b.onclick = () => setView(b.dataset.view); });
$('gcsToggle').onclick = () => setView(app.view === 'sim' ? 'fly' : 'sim');
$('swapBtn').onclick = () => setView(app.view === 'fly' ? 'sim' : 'fly');
$('view3d').addEventListener('click', () => { if (app.view === 'fly') setView('sim'); });

// ------------------------------------------------------------------ fly tools
const isFlying = () => { const t = app.vehicle.telemetry(); return t.armed && !t.landed; };
$('toolArm').onclick = () => {
  const t = app.vehicle.telemetry();
  if (t.armed) {
    if (isFlying() && !confirm('The vehicle is flying. Disarming will stop the motors and it will fall. Disarm anyway?')) return;
    app.vehicle.disarm(isFlying());
  } else app.vehicle.arm();
};
$('toolTakeoff').onclick = () => app.vehicle.takeoff(Number($('apTakeoff').value) || 10);
$('toolLand').onclick = () => app.vehicle.land();
$('toolRtl').onclick = () => app.vehicle.rtl();
$('toolHold').onclick = () => app.vehicle.setMode('Hold');
$('toolMission').onclick = async () => {
  if (gcs.plan.length && (gcs.dirty || app.vehicle !== app.internal)) { if (!(await gcs.upload())) return; }
  app.vehicle.startMission();
};
$('toolReset').onclick = () => resetSim();

function resetSim() {
  if (app.linkMode === 'hitl' && app.link?.state.armed) { app.log('Disarm the Pixhawk before resetting the simulation', 4); return; }
  resetVehicle();
  ap.reset();
  ap.home = [...quad.pos];
  if (ap.mission.length) ap.setMission(ap.mission);
  view.clearTrail();
  gcs.clearTrack();
  crashedShown = false;
  $('bigStatus').innerHTML = '';
  app.log('Simulation reset');
}

// ------------------------------------------------------------------ camera / screenshot / help
$('btnCamera').onclick = () => { view.nextCamera(); $('camLabel').textContent = view.cameraLabel; };
$('btnShot').onclick = () => screenshot();
$('btnHelp').onclick = () => $('helpDlg').showModal();

function screenshot() {
  const r = view.renderer;
  const prev = r.getPixelRatio();
  r.setPixelRatio(Math.min(3, Math.max(2, prev * 2)));
  view.resize();
  const url = view.screenshot();
  r.setPixelRatio(prev);
  view.resize();
  const a = document.createElement('a');
  a.href = url;
  a.download = `dronee-f450-${new Date().toISOString().replace(/[:.]/g, '-')}.png`;
  a.click();
  app.log('Screenshot saved');
}

// ------------------------------------------------------------------ keyboard shortcuts
let lastSpace = 0;
window.addEventListener('keydown', (e) => {
  if (e.target && ['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName)) return;
  if (document.querySelector('dialog[open]')) return;
  const v = app.vehicle;
  switch (e.code) {
    case 'Space': {
      const t = v.telemetry();
      if (!t.armed) v.arm();
      else if (!isFlying()) v.disarm();
      else if (performance.now() - lastSpace < 800) v.disarm(true);
      else app.log('Press Space again to disarm in flight', 4);
      lastSpace = performance.now();
      break;
    }
    case 'KeyT': v.takeoff(Number($('apTakeoff').value) || 10); break;
    case 'KeyG': v.land(); break;
    case 'KeyH': v.rtl(); break;
    case 'KeyO': v.setMode('Hold'); break;
    case 'KeyM': $('toolMission').click(); break;
    case 'Digit1': v.setMode('Stabilized'); break;
    case 'Digit2': v.setMode('Altitude'); break;
    case 'Digit3': v.setMode('Position'); break;
    case 'Digit4': v.setMode('Acro'); break;
    case 'KeyC': $('btnCamera').click(); break;
    case 'KeyQ': $('gcsToggle').click(); break;
    case 'KeyP': screenshot(); break;
    case 'KeyU': $('app').classList.toggle('photo'); setTimeout(() => view.resize(), 50); break;
    case 'Backspace': e.preventDefault(); resetSim(); break;
    case 'F1': e.preventDefault(); $('helpDlg').showModal(); break;
    default: return;
  }
});

// ------------------------------------------------------------------ connect Pixhawk
const dlg = $('connectDlg');
$('cSupport').textContent = SerialTransport.supported
  ? 'Web Serial is available in this browser.'
  : 'This browser has no Web Serial support — use desktop Chrome or Edge, or the WebSocket bridge.';
const updateTransportUi = () => { $('cWsWrap').style.display = $('cTransport').value === 'ws' ? '' : 'none'; };
$('cTransport').onchange = updateTransportUi;
updateTransportUi();

$('btnConnect').onclick = () => {
  if (app.link) { disconnect(); return; }
  $('cErr').textContent = '';
  dlg.showModal();
};

dlg.addEventListener('close', async () => {
  if (dlg.returnValue !== 'default') return;
  const mode = new FormData(dlg.querySelector('form')).get('cmode');
  const transport = $('cTransport').value === 'ws' ? new WebSocketTransport($('cWs').value) : new SerialTransport(Number($('cBaud').value));
  const link = new MavlinkVehicle(transport);
  try {
    app.log('Connecting…');
    await link.connect();
  } catch (err) {
    $('cErr').textContent = err.message;
    try { link.close(); } catch { /* ignore */ }
    dlg.showModal();
    return;
  }
  app.link = link;
  app.linkMode = mode;
  link.on('text', (txt, sev) => app.log(`[FC] ${txt}`, sev));
  link.on('rc', (m) => input.setRcChannels(m));
  link.on('close', () => { if (app.link === link) { app.log('Pixhawk disconnected', 3); disconnect(true); } });
  link.on('armed', (a) => app.log(a ? 'Pixhawk armed' : 'Pixhawk disarmed'));
  link.on('mode', (m) => app.log(`Pixhawk mode: ${m}`));
  const ap2 = link.autopilot === 'px4' ? 'PX4' : link.autopilot === 'ardupilot' ? 'ArduPilot' : 'MAVLink';
  if (mode === 'rc') {
    app.log(`${ap2} connected — your RC transmitter now flies the simulator`);
    if (input.preferred !== 'auto' && input.preferred !== 'rc') app.log('Tip: set Setup › Input source to Automatic or RC', 4);
  } else {
    if (ap.armed) ap.disarm('switching to Pixhawk');
    app.vehicle = link;
    if (mode === 'hitl') {
      link.hil = true;
      resetVehicle();
      view.clearTrail();
      app.log(`${ap2} HITL started — the Pixhawk is flying the simulated F450`);
      if (link.autopilot !== 'px4') app.log('HITL via HIL_SENSOR is supported by PX4. ArduPilot users: use RC controller mode or SITL.', 4);
      setTimeout(() => { if (!link.hilArmed && !link.inspector.has('HIL_ACTUATOR_CONTROLS')) app.log('No HIL_ACTUATOR_CONTROLS yet: is SYS_HITL=1 set on the Pixhawk? (see setup guide)', 4); }, 5000);
    } else app.log(`${ap2} telemetry connected`);
  }
  $('btnConnect').querySelector('span').textContent = 'Disconnect';
  $('btnConnect').classList.remove('primary');
  refreshModeSelect(true);
});

function disconnect(already = false) {
  const link = app.link;
  app.link = null;
  app.linkMode = 'sim';
  app.vehicle = app.internal;
  if (!already) link?.close();
  input.rc = null;
  $('btnConnect').querySelector('span').textContent = 'Connect Pixhawk';
  $('btnConnect').classList.add('primary');
  refreshModeSelect(true);
  resetSim();
}

// ------------------------------------------------------------------ setup panel bindings
function bindRange(id, key, fmt, apply) {
  const el = $(id);
  const out = $(`${id}Val`);
  el.value = settings[key];
  const show = () => { if (out) out.textContent = fmt(Number(el.value)); };
  show();
  el.oninput = () => { settings[key] = Number(el.value); show(); apply?.(settings[key]); saveSettings(); };
}
bindRange('setTime', 'hour', (h) => `${String(Math.floor(h)).padStart(2, '0')}:${String(Math.round((h % 1) * 60)).padStart(2, '0')}`, (h) => view.world.setTimeOfDay(h));
bindRange('setWind', 'wind', (v) => `${v} m/s`);
bindRange('setWindDir', 'windDir', (v) => `${v}°`);
bindRange('setGust', 'gust', (v) => v.toFixed(1));
bindRange('setNoise', 'noise', (v) => `${v.toFixed(1)}×`, (v) => { sensors.noise = v; });
$('setQuality').value = settings.quality;
$('setQuality').onchange = (e) => { settings.quality = e.target.value; saveSettings(); location.reload(); };
$('setTrail').checked = settings.trail;
$('setTrail').onchange = (e) => { settings.trail = e.target.checked; view.showTrail = settings.trail; saveSettings(); };
const applySticks = () => { $('stickMon').style.visibility = settings.sticks ? '' : 'hidden'; };
$('setSticks').checked = settings.sticks;
$('setSticks').onchange = (e) => { settings.sticks = e.target.checked; applySticks(); saveSettings(); };
applySticks();
const showHome = () => { $('homeLat').value = settings.home.lat; $('homeLon').value = settings.home.lon; $('homeAlt').value = settings.home.alt; };
showHome();
$('homeApply').onclick = () => {
  const lat = Number($('homeLat').value), lon = Number($('homeLon').value), alt = Number($('homeAlt').value);
  if (!(Math.abs(lat) <= 85 && Math.abs(lon) <= 180)) { app.log('Invalid home position', 3); return; }
  settings.home = { lat, lon, alt };
  saveSettings();
  geo.set(lat, lon, alt);
  resetSim();
  gcs.map.setView([lat, lon], 18);
  gcs.renderPlan();
  app.log('Home location updated');
};
$('homeKfupm').onclick = () => { settings.home = { ...KFUPM }; showHome(); $('homeApply').click(); };
const apParam = (id, key) => { $(id).onchange = (e) => { ap.params[key] = Number(e.target.value); }; ap.params[key] = Number($(id).value); };
apParam('apTakeoff', 'takeoffAlt');
apParam('apRtl', 'rtlAlt');
apParam('apSpeed', 'maxHorizSpeed');
apParam('apTilt', 'maxTiltDeg');

// ------------------------------------------------------------------ start
refreshModeSelect(true);
setView('sim');
requestAnimationFrame(frame);
setTimeout(() => {
  $('loading').classList.add('done');
  if (!settings.helpSeen) { $('helpDlg').showModal(); settings.helpSeen = true; saveSettings(); }
}, 300);
app.log('Simulation mode — fly with the keyboard, a gamepad, or connect your Pixhawk + RC');

// expose a few internals for debugging / automated tests
Object.assign(app, { quad, ap, sensors, resetSim, setView, qToEuler });
