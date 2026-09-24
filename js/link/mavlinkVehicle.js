// A MAVLink vehicle connection (the Pixhawk). Handles heartbeats, telemetry
// decoding, commands, flight-mode mapping for PX4 and ArduPilot, the mission
// upload/download micro-protocol and hardware-in-the-loop sensor injection.

import {
  MavlinkCodec, MAV_TYPE, MAV_AUTOPILOT, MAV_MODE_FLAG, MAV_CMD, MAV_FRAME, MAV_RESULT, MAV_MISSION_RESULT, IDS,
} from '../mavlink/mavlink.js';

// ---------------------------------------------------------------- flight modes
const PX4_MAIN = { MANUAL: 1, ALTCTL: 2, POSCTL: 3, AUTO: 4, ACRO: 5, OFFBOARD: 6, STABILIZED: 7 };
const PX4_AUTO = { READY: 1, TAKEOFF: 2, LOITER: 3, MISSION: 4, RTL: 5, LAND: 6 };
const PX4_MODES = {
  Manual: [PX4_MAIN.MANUAL, 0],
  Stabilized: [PX4_MAIN.STABILIZED, 0],
  Acro: [PX4_MAIN.ACRO, 0],
  Altitude: [PX4_MAIN.ALTCTL, 0],
  Position: [PX4_MAIN.POSCTL, 0],
  Hold: [PX4_MAIN.AUTO, PX4_AUTO.LOITER],
  Takeoff: [PX4_MAIN.AUTO, PX4_AUTO.TAKEOFF],
  Mission: [PX4_MAIN.AUTO, PX4_AUTO.MISSION],
  Return: [PX4_MAIN.AUTO, PX4_AUTO.RTL],
  Land: [PX4_MAIN.AUTO, PX4_AUTO.LAND],
  Offboard: [PX4_MAIN.OFFBOARD, 0],
};
const APM_MODES = {
  Stabilized: 0, Acro: 1, Altitude: 2, Mission: 3, Guided: 4, Hold: 5, Return: 6, Circle: 7, Land: 9, Drift: 11, Sport: 13, Position: 16, Brake: 17, 'Smart RTL': 21,
};

function px4ModeName(customMode) {
  const main = (customMode >> 16) & 0xff;
  const sub = (customMode >> 24) & 0xff;
  for (const [name, [m, s]] of Object.entries(PX4_MODES)) if (m === main && (s === sub || (m !== PX4_MAIN.AUTO && !s))) return name;
  if (main === PX4_MAIN.AUTO) return sub === PX4_AUTO.READY ? 'Ready' : `Auto(${sub})`;
  return `Mode ${main}`;
}
function apmModeName(customMode) {
  for (const [n, v] of Object.entries(APM_MODES)) if (v === customMode) return n;
  return `Mode ${customMode}`;
}

export class MavlinkVehicle {
  constructor(transport, { sysid = 255, compid = 190 } = {}) {
    this.transport = transport;
    this.codec = new MavlinkCodec({ sysid, compid });
    this.listeners = {};
    this.target = { sys: 1, comp: 1 };
    this.autopilot = null; // 'px4' | 'ardupilot' | 'generic'
    this.connected = false;
    this.lastHeartbeat = 0;
    this.bootTime = performance.now();
    this.hil = false;
    this.actuators = [0, 0, 0, 0];
    this.actuatorsTime = 0;
    this.inspector = new Map(); // name -> {count, last, hz, t}
    this.pendingCmd = new Map();
    this.missionUpload = null;
    this.missionDownload = null;
    this.state = {
      source: 'pixhawk',
      armed: false, mode: '—', roll: 0, pitch: 0, yaw: 0, rates: [0, 0, 0],
      lat: 0, lon: 0, altAmsl: 0, altRel: 0, vel: [0, 0, 0], groundSpeed: 0, climb: 0, heading: 0, throttle: 0,
      battery: { voltage: 0, current: 0, remaining: -1 },
      gps: { fix: 0, sats: 0, hdop: 99 },
      home: null, landed: true, flightTime: 0, missionIndex: 0, missionState: 'idle', distHome: 0,
      motors: [0, 0, 0, 0], rc: null, systemStatus: 0, sensorsHealthy: true,
    };
    transport.onData = (bytes) => this.onBytes(bytes);
    transport.onClose = () => { this.connected = false; clearInterval(this.hbTimer); this.emit('close'); };
  }

  on(evt, fn) { (this.listeners[evt] ||= []).push(fn); }
  emit(evt, ...a) { (this.listeners[evt] || []).forEach((f) => f(...a)); }

  async connect() {
    await this.transport.open();
    this.hbTimer = setInterval(() => this.sendHeartbeat(), 1000);
    this.sendHeartbeat();
    // wait for the autopilot heartbeat
    await new Promise((resolve, reject) => {
      const t = setTimeout(() => reject(new Error('No heartbeat from the flight controller within 6 s. Check the cable / baud rate, and that no other program (QGroundControl, Mission Planner) is using the port.')), 6000);
      this.on('heartbeat', () => { clearTimeout(t); resolve(); });
    });
    this.requestStreams();
  }

  close() {
    clearInterval(this.hbTimer);
    this.transport.close();
  }

  send(name, fields) {
    this.transport.write(this.codec.encode(name, fields));
  }

  sendHeartbeat() {
    this.send('HEARTBEAT', { type: MAV_TYPE.GCS, autopilot: MAV_AUTOPILOT.INVALID, base_mode: 0, custom_mode: 0, system_status: 4, mavlink_version: 3 });
  }

  requestStreams() {
    const t = this.target;
    const rates = { ATTITUDE: 25, GLOBAL_POSITION_INT: 10, RC_CHANNELS: 20, VFR_HUD: 5, SYS_STATUS: 2, GPS_RAW_INT: 2, HOME_POSITION: 0.5, EXTENDED_SYS_STATE: 2, BATTERY_STATUS: 1, SERVO_OUTPUT_RAW: 10, MISSION_CURRENT: 1 };
    for (const [name, hz] of Object.entries(rates)) {
      this.send('COMMAND_LONG', {
        target_system: t.sys, target_component: t.comp, command: MAV_CMD.SET_MESSAGE_INTERVAL,
        param1: IDS[name], param2: Math.round(1e6 / hz),
      });
    }
    if (this.autopilot === 'ardupilot') {
      // legacy stream request (ArduPilot)
      this.send('REQUEST_DATA_STREAM', { target_system: t.sys, target_component: t.comp, req_stream_id: 0, req_message_rate: 10, start_stop: 1 });
    }
    this.send('COMMAND_LONG', { target_system: t.sys, target_component: t.comp, command: MAV_CMD.REQUEST_MESSAGE, param1: IDS.HOME_POSITION });
  }

  // ---------------------------------------------------------------- receive
  onBytes(bytes) {
    for (const msg of this.codec.parse(bytes)) this.handle(msg);
  }

  handle(m) {
    const now = performance.now();
    const ins = this.inspector.get(m._name) || { count: 0, hz: 0, t: now, n: 0 };
    ins.count++; ins.n++; ins.last = m; ins.sys = m._sysid; ins.comp = m._compid;
    if (now - ins.t > 1000) { ins.hz = (ins.n * 1000) / (now - ins.t); ins.t = now; ins.n = 0; }
    this.inspector.set(m._name, ins);

    const s = this.state;
    switch (m._name) {
      case 'HEARTBEAT': {
        if (m.autopilot === MAV_AUTOPILOT.INVALID || m.type === MAV_TYPE.GCS) break; // other GCS / peripherals
        if (!this.connected || m._sysid === this.target.sys) {
          const first = !this.connected;
          this.target = { sys: m._sysid, comp: m._compid };
          this.autopilot = m.autopilot === MAV_AUTOPILOT.PX4 ? 'px4' : m.autopilot === MAV_AUTOPILOT.ARDUPILOTMEGA ? 'ardupilot' : 'generic';
          this.connected = true;
          this.lastHeartbeat = now;
          const armed = !!(m.base_mode & MAV_MODE_FLAG.SAFETY_ARMED);
          if (armed !== s.armed) this.emit('armed', armed);
          s.armed = armed;
          s.hilFlag = !!(m.base_mode & MAV_MODE_FLAG.HIL_ENABLED);
          const mode = this.autopilot === 'px4' ? px4ModeName(m.custom_mode) : this.autopilot === 'ardupilot' ? apmModeName(m.custom_mode) : `Mode ${m.custom_mode}`;
          if (mode !== s.mode) { s.mode = mode; this.emit('mode', mode); }
          s.systemStatus = m.system_status;
          this.emit('heartbeat', m, first);
        }
        break;
      }
      case 'ATTITUDE':
        s.roll = m.roll; s.pitch = m.pitch; s.yaw = m.yaw;
        s.rates = [m.rollspeed, m.pitchspeed, m.yawspeed];
        s.heading = ((m.yaw * 180) / Math.PI + 360) % 360;
        break;
      case 'GLOBAL_POSITION_INT':
        s.lat = m.lat / 1e7; s.lon = m.lon / 1e7;
        s.altAmsl = m.alt / 1000; s.altRel = m.relative_alt / 1000;
        s.vel = [m.vx / 100, m.vy / 100, m.vz / 100];
        s.groundSpeed = Math.hypot(s.vel[0], s.vel[1]);
        s.climb = -s.vel[2];
        if (s.home) s.distHome = distanceM(s.lat, s.lon, s.home.lat, s.home.lon);
        break;
      case 'VFR_HUD':
        s.throttle = m.throttle / 100;
        s.groundSpeed = m.groundspeed;
        s.climb = m.climb;
        break;
      case 'SYS_STATUS':
        s.battery.voltage = m.voltage_battery / 1000;
        s.battery.current = m.current_battery / 100;
        s.battery.remaining = m.battery_remaining >= 0 ? m.battery_remaining / 100 : -1;
        s.sensorsHealthy = (m.onboard_control_sensors_health & m.onboard_control_sensors_enabled) === m.onboard_control_sensors_enabled;
        break;
      case 'GPS_RAW_INT':
        s.gps = { fix: m.fix_type, sats: m.satellites_visible, hdop: m.eph / 100 };
        break;
      case 'HOME_POSITION':
        s.home = { lat: m.latitude / 1e7, lon: m.longitude / 1e7, alt: m.altitude / 1000 };
        this.emit('home', s.home);
        break;
      case 'EXTENDED_SYS_STATE':
        s.landed = m.landed_state === 1;
        break;
      case 'RC_CHANNELS':
        s.rc = m;
        this.emit('rc', m);
        break;
      case 'SERVO_OUTPUT_RAW':
        s.motors = [m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw].map((p) => Math.max(0, Math.min(1, (p - 1000) / 1000)));
        break;
      case 'HIL_ACTUATOR_CONTROLS': {
        const c = m.controls;
        this.actuators = [0, 1, 2, 3].map((i) => (Number.isFinite(c[i]) ? Math.max(0, Math.min(1, c[i])) : 0));
        this.actuatorsTime = now;
        this.actuatorsSeq = (this.actuatorsSeq || 0) + 1;
        this.hilArmed = !!(m.mode & MAV_MODE_FLAG.SAFETY_ARMED);
        break;
      }
      case 'STATUSTEXT':
        this.emit('text', m.text, m.severity);
        break;
      case 'COMMAND_ACK': {
        const p = this.pendingCmd.get(m.command);
        if (p && m.result !== 5) { this.pendingCmd.delete(m.command); p(m.result); }
        break;
      }
      case 'MISSION_CURRENT':
        s.missionIndex = m.seq;
        this.emit('missionCurrent', m.seq);
        break;
      case 'MISSION_ITEM_REACHED':
        this.emit('missionReached', m.seq);
        break;
      case 'MISSION_REQUEST':
      case 'MISSION_REQUEST_INT':
        this.onMissionRequest(m);
        break;
      case 'MISSION_ACK':
        this.onMissionAck(m);
        break;
      case 'MISSION_COUNT':
        this.onMissionCount(m);
        break;
      case 'MISSION_ITEM_INT':
        this.onMissionItem(m);
        break;
    }
  }

  // ---------------------------------------------------------------- commands
  command(cmd, params = [], { timeout = 2500 } = {}) {
    const [p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0, p6 = 0, p7 = 0] = params;
    return new Promise((resolve) => {
      let tries = 0;
      const sendIt = () => this.send('COMMAND_LONG', {
        target_system: this.target.sys, target_component: this.target.comp, command: cmd, confirmation: tries,
        param1: p1, param2: p2, param3: p3, param4: p4, param5: p5, param6: p6, param7: p7,
      });
      const timer = setInterval(() => {
        if (++tries >= 3) { clearInterval(timer); this.pendingCmd.delete(cmd); resolve({ ok: false, result: 'NO RESPONSE' }); return; }
        sendIt();
      }, timeout / 3);
      this.pendingCmd.set(cmd, (result) => { clearInterval(timer); resolve({ ok: result === 0, result: MAV_RESULT[result] || result }); });
      sendIt();
    });
  }

  async report(label, promise) {
    const r = await promise;
    if (!r.ok) this.emit('text', `${label} rejected: ${r.result}`, 4);
    return r.ok;
  }

  get modes() {
    return this.autopilot === 'ardupilot'
      ? ['Stabilized', 'Acro', 'Altitude', 'Position', 'Hold', 'Guided', 'Mission', 'Return', 'Land', 'Brake', 'Smart RTL']
      : ['Manual', 'Stabilized', 'Acro', 'Altitude', 'Position', 'Hold', 'Takeoff', 'Mission', 'Return', 'Land', 'Offboard'];
  }

  arm(force = false) { return this.report('Arm', this.command(MAV_CMD.COMPONENT_ARM_DISARM, [1, force ? 21196 : 0])); }
  disarm(force = false) { return this.report('Disarm', this.command(MAV_CMD.COMPONENT_ARM_DISARM, [0, force ? 21196 : 0])); }

  setMode(name) {
    if (this.autopilot === 'ardupilot') {
      const cm = APM_MODES[name];
      if (cm === undefined) return Promise.resolve(false);
      return this.report(`Mode ${name}`, this.command(MAV_CMD.DO_SET_MODE, [MAV_MODE_FLAG.CUSTOM_MODE_ENABLED, cm]));
    }
    const pm = PX4_MODES[name];
    if (!pm) return Promise.resolve(false);
    return this.report(`Mode ${name}`, this.command(MAV_CMD.DO_SET_MODE, [MAV_MODE_FLAG.CUSTOM_MODE_ENABLED, pm[0], pm[1]]));
  }

  async takeoff(alt = 10) {
    const s = this.state;
    if (this.autopilot === 'ardupilot') {
      await this.setMode('Guided');
      if (!s.armed) await this.arm();
      return this.report('Takeoff', this.command(MAV_CMD.NAV_TAKEOFF, [0, 0, 0, NaN, 0, 0, alt]));
    }
    const amsl = (s.home ? s.home.alt : s.altAmsl - s.altRel) + alt;
    const ok = await this.report('Takeoff', this.command(MAV_CMD.NAV_TAKEOFF, [NaN, 0, 0, NaN, NaN, NaN, amsl]));
    if (ok && !s.armed) await this.arm();
    return ok;
  }

  land() { return this.autopilot === 'ardupilot' ? this.setMode('Land') : this.report('Land', this.command(MAV_CMD.NAV_LAND, [0, 0, 0, NaN, NaN, NaN, NaN])); }
  rtl() { return this.autopilot === 'ardupilot' ? this.setMode('Return') : this.report('RTL', this.command(MAV_CMD.NAV_RETURN_TO_LAUNCH)); }

  async startMission() {
    if (this.autopilot === 'ardupilot') {
      if (!this.state.armed) { await this.setMode('Guided'); await this.arm(); }
      await this.setMode('Mission');
      return this.report('Mission start', this.command(MAV_CMD.MISSION_START, [0, 0]));
    }
    if (!this.state.armed) await this.arm();
    return this.report('Mission start', this.command(MAV_CMD.MISSION_START, [0, 0]));
  }

  gotoLocation(lat, lon, relAlt) {
    const s = this.state;
    const homeAlt = s.home ? s.home.alt : s.altAmsl - s.altRel;
    const altAmsl = relAlt !== undefined ? homeAlt + relAlt : s.altAmsl;
    if (this.autopilot === 'ardupilot') this.setMode('Guided');
    // DO_REPOSITION needs lat/lon as int in COMMAND_INT; COMMAND_LONG with float degrees is accepted by PX4 & ArduPilot
    return this.report('Go to', this.command(MAV_CMD.DO_REPOSITION, [-1, 1, 0, NaN, lat, lon, altAmsl]));
  }

  // ---------------------------------------------------------------- mission protocol
  /** items: [{command, lat, lon, alt(rel), param1..4}] */
  uploadMission(items, onProgress) {
    if (this.missionUpload) return Promise.reject(new Error('Upload already in progress'));
    const s = this.state;
    const home = s.home || { lat: s.lat, lon: s.lon, alt: s.altAmsl };
    let list = items.map((it) => ({ ...it }));
    // takeoff items without coordinates use the current/home position
    list.forEach((it) => { if (it.command === MAV_CMD.NAV_TAKEOFF && !it.lat) { it.lat = home.lat; it.lon = home.lon; } });
    if (this.autopilot === 'ardupilot') list = [{ command: MAV_CMD.NAV_WAYPOINT, lat: home.lat, lon: home.lon, alt: 0, frame: MAV_FRAME.GLOBAL }, ...list];
    return new Promise((resolve, reject) => {
      const up = { list, resolve, reject, sent: new Set(), onProgress };
      this.missionUpload = up;
      const fail = (msg) => { clearTimeout(up.timer); this.missionUpload = null; reject(new Error(msg)); };
      up.fail = fail;
      let tries = 0;
      const sendCount = () => {
        this.send('MISSION_COUNT', { target_system: this.target.sys, target_component: this.target.comp, count: list.length, mission_type: 0 });
      };
      up.kick = () => {
        clearTimeout(up.timer);
        up.timer = setTimeout(() => {
          if (++tries > 4) return fail('Mission upload timed out');
          if (!up.sent.size) sendCount(); else up.kick();
        }, 1500);
      };
      sendCount();
      up.kick();
    });
  }

  onMissionRequest(m) {
    const up = this.missionUpload;
    if (!up) return;
    const it = up.list[m.seq];
    if (!it) return up.fail(`Vehicle requested unknown item ${m.seq}`);
    up.sent.add(m.seq);
    up.kick();
    up.onProgress?.((m.seq + 1) / up.list.length);
    this.send('MISSION_ITEM_INT', {
      target_system: this.target.sys, target_component: this.target.comp,
      seq: m.seq, frame: it.frame ?? MAV_FRAME.GLOBAL_RELATIVE_ALT_INT, command: it.command,
      current: m.seq === 0 ? 1 : 0, autocontinue: 1,
      param1: it.param1 ?? 0, param2: it.param2 ?? 0, param3: it.param3 ?? 0, param4: it.param4 ?? NaN,
      x: Math.round((it.lat || 0) * 1e7), y: Math.round((it.lon || 0) * 1e7), z: it.alt ?? 0, mission_type: 0,
    });
  }

  onMissionAck(m) {
    const up = this.missionUpload;
    if (!up) return;
    clearTimeout(up.timer);
    this.missionUpload = null;
    if (m.type === 0) up.resolve(true);
    else up.reject(new Error(`Mission rejected: ${MAV_MISSION_RESULT[m.type] || m.type}`));
  }

  downloadMission() {
    return new Promise((resolve, reject) => {
      const dl = { items: [], count: -1, resolve, reject };
      this.missionDownload = dl;
      const req = () => this.send('MISSION_REQUEST_LIST', { target_system: this.target.sys, target_component: this.target.comp, mission_type: 0 });
      dl.timer = setTimeout(() => { this.missionDownload = null; reject(new Error('Mission download timed out')); }, 8000);
      req();
    });
  }

  onMissionCount(m) {
    const dl = this.missionDownload;
    if (!dl) return;
    dl.count = m.count;
    if (!m.count) return this.finishDownload();
    this.send('MISSION_REQUEST_INT', { target_system: this.target.sys, target_component: this.target.comp, seq: 0, mission_type: 0 });
  }

  onMissionItem(m) {
    const dl = this.missionDownload;
    if (!dl) return;
    dl.items[m.seq] = { command: m.command, lat: m.x / 1e7, lon: m.y / 1e7, alt: m.z, param1: m.param1, param2: m.param2, param3: m.param3, param4: m.param4, frame: m.frame };
    if (m.seq + 1 < dl.count) this.send('MISSION_REQUEST_INT', { target_system: this.target.sys, target_component: this.target.comp, seq: m.seq + 1, mission_type: 0 });
    else this.finishDownload();
  }

  finishDownload() {
    const dl = this.missionDownload;
    this.missionDownload = null;
    clearTimeout(dl.timer);
    this.send('MISSION_ACK', { target_system: this.target.sys, target_component: this.target.comp, type: 0, mission_type: 0 });
    let items = dl.items.filter(Boolean);
    if (this.autopilot === 'ardupilot') items = items.slice(1); // drop home
    dl.resolve(items);
  }

  // ---------------------------------------------------------------- hardware in the loop
  sendHilSensor(timeUs, imu, baro) {
    this.send('HIL_SENSOR', {
      time_usec: timeUs,
      xacc: imu.acc[0], yacc: imu.acc[1], zacc: imu.acc[2],
      xgyro: imu.gyro[0], ygyro: imu.gyro[1], zgyro: imu.gyro[2],
      xmag: imu.mag[0], ymag: imu.mag[1], zmag: imu.mag[2],
      abs_pressure: baro.pressure, diff_pressure: 0, pressure_alt: baro.alt, temperature: baro.temperature,
      fields_updated: 0x1fff, id: 0,
    });
  }

  sendHilGps(timeUs, gps) {
    this.send('HIL_GPS', {
      time_usec: timeUs, fix_type: gps.fix,
      lat: Math.round(gps.lat * 1e7), lon: Math.round(gps.lon * 1e7), alt: Math.round(gps.alt * 1000),
      eph: Math.round(gps.eph * 100), epv: Math.round(gps.epv * 100),
      vel: Math.round(gps.groundSpeed * 100),
      vn: Math.round(gps.vel[0] * 100), ve: Math.round(gps.vel[1] * 100), vd: Math.round(gps.vel[2] * 100),
      cog: Math.round(gps.cog * 100) % 36000, satellites_visible: gps.sats, id: 0, yaw: 0,
    });
  }

  /** Motor outputs 0..1 received from the flight controller (HIL). */
  get hilActuators() {
    return performance.now() - this.actuatorsTime < 1000 ? this.actuators : [0, 0, 0, 0];
  }

  telemetry() { return this.state; }
}

function distanceM(lat1, lon1, lat2, lon2) {
  const R = 6378137, d = Math.PI / 180;
  const x = (lon2 - lon1) * d * Math.cos(((lat1 + lat2) / 2) * d);
  const y = (lat2 - lat1) * d;
  return Math.hypot(x, y) * R;
}
