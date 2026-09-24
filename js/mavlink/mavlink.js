// Minimal MAVLink v1/v2 codec (no signing). Parses v1 and v2 frames,
// encodes v2 frames. Message layouts come from messages.js.

import { MESSAGES, IDS } from './messages.js';

export { IDS };

const SIZES = { u8: 1, i8: 1, char: 1, u16: 2, i16: 2, u32: 4, i32: 4, f32: 4, u64: 8, i64: 8, f64: 8 };

// Pre-compute payload length / field offsets for every message.
for (const def of Object.values(MESSAGES)) {
  let off = 0;
  def.layout = def.fields.map(([name, type, n]) => {
    const f = { name, type, n, off };
    off += SIZES[type] * (n || 1);
    return f;
  });
  def.length = off;
}

export function crcAccumulate(byte, crc) {
  let tmp = byte ^ (crc & 0xff);
  tmp = (tmp ^ (tmp << 4)) & 0xff;
  return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
}

function crcCalc(buf, start, end, extra) {
  let crc = 0xffff;
  for (let i = start; i < end; i++) crc = crcAccumulate(buf[i], crc);
  return crcAccumulate(extra, crc);
}

function readField(dv, f) {
  const le = true;
  const one = (o) => {
    switch (f.type) {
      case 'u8': case 'char': return dv.getUint8(o);
      case 'i8': return dv.getInt8(o);
      case 'u16': return dv.getUint16(o, le);
      case 'i16': return dv.getInt16(o, le);
      case 'u32': return dv.getUint32(o, le);
      case 'i32': return dv.getInt32(o, le);
      case 'f32': return dv.getFloat32(o, le);
      case 'f64': return dv.getFloat64(o, le);
      case 'u64': return Number(dv.getBigUint64(o, le));
      case 'i64': return Number(dv.getBigInt64(o, le));
    }
    return 0;
  };
  if (!f.n) return one(f.off);
  if (f.type === 'char') {
    let s = '';
    for (let i = 0; i < f.n; i++) {
      const c = dv.getUint8(f.off + i);
      if (!c) break;
      s += String.fromCharCode(c);
    }
    return s;
  }
  const arr = [];
  for (let i = 0; i < f.n; i++) arr.push(one(f.off + i * SIZES[f.type]));
  return arr;
}

function writeField(dv, f, value) {
  const le = true;
  const one = (o, v) => {
    v = v ?? 0;
    switch (f.type) {
      case 'u8': case 'char': dv.setUint8(o, v); break;
      case 'i8': dv.setInt8(o, v); break;
      case 'u16': dv.setUint16(o, v, le); break;
      case 'i16': dv.setInt16(o, v, le); break;
      case 'u32': dv.setUint32(o, v >>> 0, le); break;
      case 'i32': dv.setInt32(o, v, le); break;
      case 'f32': dv.setFloat32(o, v, le); break;
      case 'f64': dv.setFloat64(o, v, le); break;
      case 'u64': dv.setBigUint64(o, BigInt(Math.max(0, Math.floor(v))), le); break;
      case 'i64': dv.setBigInt64(o, BigInt(Math.floor(v)), le); break;
    }
  };
  if (!f.n) return one(f.off, value);
  if (f.type === 'char') {
    const s = value || '';
    for (let i = 0; i < f.n; i++) dv.setUint8(f.off + i, i < s.length ? s.charCodeAt(i) & 0x7f : 0);
    return;
  }
  for (let i = 0; i < f.n; i++) one(f.off + i * SIZES[f.type], value ? value[i] : 0);
}

export class MavlinkCodec {
  constructor({ sysid = 255, compid = 190 } = {}) {
    this.sysid = sysid;
    this.compid = compid;
    this.seq = 0;
    this.buf = new Uint8Array(0);
    this.stats = { rx: 0, tx: 0, crcErrors: 0, unknown: 0 };
  }

  /** Encode a message (by name) into a MAVLink v2 frame. */
  encode(name, fields = {}) {
    const id = IDS[name];
    const def = MESSAGES[id];
    if (!def) throw new Error(`Unknown MAVLink message ${name}`);
    const payload = new Uint8Array(def.length);
    const dv = new DataView(payload.buffer);
    for (const f of def.layout) writeField(dv, f, fields[f.name]);
    // v2 payload truncation of trailing zeros (at least one byte kept)
    let len = def.length;
    while (len > 1 && payload[len - 1] === 0) len--;
    const frame = new Uint8Array(10 + len + 2);
    frame[0] = 0xfd;
    frame[1] = len;
    frame[2] = 0; // incompat flags
    frame[3] = 0; // compat flags
    frame[4] = this.seq = (this.seq + 1) & 0xff;
    frame[5] = this.sysid;
    frame[6] = this.compid;
    frame[7] = id & 0xff;
    frame[8] = (id >> 8) & 0xff;
    frame[9] = (id >> 16) & 0xff;
    frame.set(payload.subarray(0, len), 10);
    const crc = crcCalc(frame, 1, 10 + len, def.crcExtra);
    frame[10 + len] = crc & 0xff;
    frame[11 + len] = crc >> 8;
    this.stats.tx++;
    return frame;
  }

  /** Feed raw bytes; returns an array of decoded messages. */
  parse(chunk) {
    const out = [];
    const merged = new Uint8Array(this.buf.length + chunk.length);
    merged.set(this.buf);
    merged.set(chunk, this.buf.length);
    let i = 0;
    const n = merged.length;
    while (i < n) {
      const stx = merged[i];
      if (stx !== 0xfd && stx !== 0xfe) { i++; continue; }
      const v2 = stx === 0xfd;
      const hdr = v2 ? 10 : 6;
      if (i + hdr > n) break;
      const len = merged[i + 1];
      const signed = v2 && (merged[i + 2] & 0x01);
      const total = hdr + len + 2 + (signed ? 13 : 0);
      if (i + total > n) break;
      const msgid = v2 ? merged[i + 7] | (merged[i + 8] << 8) | (merged[i + 9] << 16) : merged[i + 5];
      const def = MESSAGES[msgid];
      if (!def) { this.stats.unknown++; i += total; continue; }
      const crc = crcCalc(merged, i + 1, i + hdr + len, def.crcExtra);
      const rxCrc = merged[i + hdr + len] | (merged[i + hdr + len + 1] << 8);
      if (crc !== rxCrc) { this.stats.crcErrors++; i++; continue; }
      const payload = new Uint8Array(Math.max(def.length, len));
      payload.set(merged.subarray(i + hdr, i + hdr + len));
      const dv = new DataView(payload.buffer);
      const msg = {
        _id: msgid,
        _name: def.name,
        _sysid: merged[i + (v2 ? 5 : 3)],
        _compid: merged[i + (v2 ? 6 : 4)],
        _v2: v2,
      };
      for (const f of def.layout) {
        if (f.off + SIZES[f.type] * (f.n || 1) <= payload.length) msg[f.name] = readField(dv, f);
      }
      this.stats.rx++;
      out.push(msg);
      i += total;
    }
    this.buf = merged.slice(i);
    if (this.buf.length > 4096) this.buf = this.buf.slice(-280);
    return out;
  }
}

// Frequently used enums -------------------------------------------------------
export const MAV_TYPE = { GENERIC: 0, QUADROTOR: 2, GCS: 6 };
export const MAV_AUTOPILOT = { GENERIC: 0, ARDUPILOTMEGA: 3, INVALID: 8, PX4: 12 };
export const MAV_MODE_FLAG = {
  CUSTOM_MODE_ENABLED: 1, TEST_ENABLED: 2, AUTO_ENABLED: 4, GUIDED_ENABLED: 8,
  STABILIZE_ENABLED: 16, HIL_ENABLED: 32, MANUAL_INPUT_ENABLED: 64, SAFETY_ARMED: 128,
};
export const MAV_CMD = {
  NAV_WAYPOINT: 16, NAV_LOITER_UNLIM: 17, NAV_LOITER_TIME: 19, NAV_RETURN_TO_LAUNCH: 20,
  NAV_LAND: 21, NAV_TAKEOFF: 22, DO_SET_MODE: 176, DO_CHANGE_SPEED: 178, DO_SET_HOME: 179,
  DO_REPOSITION: 192, MISSION_START: 300, COMPONENT_ARM_DISARM: 400, SET_MESSAGE_INTERVAL: 511,
  REQUEST_MESSAGE: 512,
};
export const MAV_FRAME = { GLOBAL: 0, MISSION: 2, GLOBAL_RELATIVE_ALT: 3, GLOBAL_RELATIVE_ALT_INT: 6 };
export const MAV_RESULT = ['ACCEPTED', 'TEMPORARILY_REJECTED', 'DENIED', 'UNSUPPORTED', 'FAILED', 'IN_PROGRESS', 'CANCELLED'];
export const MAV_MISSION_RESULT = ['ACCEPTED', 'ERROR', 'UNSUPPORTED_FRAME', 'UNSUPPORTED', 'NO_SPACE', 'INVALID',
  'INVALID_PARAM1', 'INVALID_PARAM2', 'INVALID_PARAM3', 'INVALID_PARAM4', 'INVALID_PARAM5_X', 'INVALID_PARAM6_Y',
  'INVALID_PARAM7', 'INVALID_SEQUENCE', 'DENIED', 'OPERATION_CANCELLED'];
