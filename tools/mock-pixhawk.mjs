#!/usr/bin/env node
// A tiny fake "PX4 in HITL mode" used to test the simulator's MAVLink link
// without hardware. It serves the same WebSocket as bridge/server.js.
//   cd bridge && npm install && cd .. && node tools/mock-pixhawk.mjs
// Then in the simulator: Connect Pixhawk › HITL › Transport: WebSocket bridge.
// Arm + Takeoff makes the mock climb to 5 m using the simulated barometer.

import { MavlinkCodec } from '../js/mavlink/mavlink.js';
import { createRequire } from 'node:module';
const require = createRequire(new URL('../bridge/package.json', import.meta.url));
const { WebSocketServer } = require('ws');

const port = Number(process.argv[2] || 14560);
const wss = new WebSocketServer({ port });
console.log(`mock PX4 (HITL) on ws://localhost:${port}`);

wss.on('connection', (ws) => {
  const codec = new MavlinkCodec({ sysid: 1, compid: 1 });
  const send = (name, f) => ws.readyState === 1 && ws.send(codec.encode(name, f));
  const st = { armed: false, main: 3, sub: 0, target: 0, alt0: null, alt: 0, climb: 0, lastAlt: null, i: 0, lat: 0, lon: 0, mission: [], rx: {} };
  let up = null;
  const hb = setInterval(() => {
    send('HEARTBEAT', { type: 2, autopilot: 12, base_mode: 1 | 32 | (st.armed ? 128 : 0), custom_mode: (st.main << 16) | (st.sub << 24), system_status: st.armed ? 4 : 3, mavlink_version: 3 });
    send('RC_CHANNELS', { time_boot_ms: 1, chancount: 8, chan1_raw: 1500, chan2_raw: 1500, chan3_raw: 1000, chan4_raw: 1500, chan5_raw: 1000, chan6_raw: 1000, chan7_raw: 1000, chan8_raw: 1000, rssi: 255 });
  }, 1000);
  const tel = setInterval(() => {
    send('GLOBAL_POSITION_INT', { time_boot_ms: 1, lat: Math.round(st.lat * 1e7), lon: Math.round(st.lon * 1e7), alt: Math.round((st.alt) * 1000), relative_alt: Math.round((st.alt - (st.alt0 ?? 0)) * 1000), vx: 0, vy: 0, vz: Math.round(-st.climb * 100), hdg: 0 });
    send('ATTITUDE', { time_boot_ms: 1, roll: 0, pitch: 0, yaw: 0 });
    send('EXTENDED_SYS_STATE', { landed_state: st.alt - (st.alt0 ?? 0) > 0.3 ? 2 : 1 });
  }, 200);
  ws.on('close', () => { clearInterval(hb); clearInterval(tel); console.log('closed', st.rx); });
  ws.on('message', (data) => {
    for (const m of codec.parse(new Uint8Array(data))) {
      st.rx[m._name] = (st.rx[m._name] || 0) + 1;
      if (m._name === 'COMMAND_LONG') {
        let result = 0;
        if (m.command === 400) { st.armed = m.param1 === 1; console.log(st.armed ? 'ARMED' : 'DISARMED'); }
        else if (m.command === 22) { st.main = 4; st.sub = 2; st.target = 5; console.log('TAKEOFF'); }
        else if (m.command === 176) { st.main = m.param2; st.sub = m.param3; }
        else if (m.command === 21) { st.main = 4; st.sub = 6; st.target = -1; }
        else if (m.command === 300) { st.main = 4; st.sub = 4; st.target = 8; console.log('MISSION START'); }
        send('COMMAND_ACK', { command: m.command, result });
      } else if (m._name === 'MISSION_COUNT') {
        up = { count: m.count, items: [] };
        send('MISSION_REQUEST_INT', { seq: 0, target_system: 255, target_component: 190 });
      } else if (m._name === 'MISSION_ITEM_INT' && up) {
        up.items.push(m);
        if (up.items.length < up.count) send('MISSION_REQUEST_INT', { seq: up.items.length, target_system: 255, target_component: 190 });
        else { st.mission = up.items; console.log(`MISSION RECEIVED ${up.count} items: ${up.items.map((i) => i.command).join(',')}`); send('MISSION_ACK', { type: 0, target_system: 255, target_component: 190 }); up = null; }
      } else if (m._name === 'HIL_GPS') {
        st.lat = m.lat / 1e7; st.lon = m.lon / 1e7;
      } else if (m._name === 'HIL_SENSOR') {
        const alt = m.pressure_alt;
        if (st.alt0 === null) st.alt0 = alt;
        if (st.lastAlt !== null) st.climb += ((alt - st.lastAlt) / 0.004 - st.climb) * 0.05;
        st.lastAlt = alt;
        st.alt = alt;
        let thr = 0;
        if (st.armed) {
          const rel = alt - st.alt0;
          const err = st.target - rel;
          st.i = Math.max(-0.2, Math.min(0.2, st.i + err * 0.004 * 0.05));
          thr = Math.max(0.1, Math.min(0.9, 0.37 + err * 0.08 - st.climb * 0.12 + st.i));
          if (st.target < 0 && rel < 0.2) thr = 0.1;
        }
        const c = new Array(16).fill(0);
        c[0] = c[1] = c[2] = c[3] = st.armed ? thr : 0;
        send('HIL_ACTUATOR_CONTROLS', { time_usec: m.time_usec, controls: c, mode: st.armed ? 128 | 32 : 32, flags: 0 });
      }
    }
  });
});
