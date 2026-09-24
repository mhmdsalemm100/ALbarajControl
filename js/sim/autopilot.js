// Internal flight controller used when no Pixhawk is running the flight
// stack (simulation-only mode and "RC via Pixhawk" mode). It mirrors the
// PX4 multicopter control cascade:
//   position -> velocity -> acceleration/attitude -> body rates -> mixer
// and exposes PX4-style flight modes so the ground-control UI works the
// same way against the internal autopilot and a real Pixhawk.

import {
  G, DEG, clamp, wrapPi, deadband, qToEuler, qRotate, PID, norm,
} from '../core/math.js';
import { MAV_CMD } from '../mavlink/mavlink.js';

export const MODES = ['Acro', 'Stabilized', 'Altitude', 'Position', 'Hold', 'Takeoff', 'Land', 'Return', 'Mission'];
export const MANUAL_MODES = ['Acro', 'Stabilized', 'Altitude', 'Position'];

const MIX = [
  // roll, pitch, yaw factors per motor (PX4 quad_x)
  [-1, +1, +1],
  [+1, -1, +1],
  [+1, +1, -1],
  [-1, -1, -1],
];

export class Autopilot {
  constructor(vehicle, geo) {
    this.v = vehicle;
    this.geo = geo;
    this.params = {
      maxTiltDeg: 35,
      acroRateDeg: 220,
      maxYawRateDeg: 150,
      maxHorizSpeed: 10,
      cruiseSpeed: 5,
      maxClimb: 3,
      maxDescent: 1.5,
      landSpeed: 0.7,
      takeoffAlt: 10,
      rtlAlt: 30,
      acceptRadius: 1.5,
    };
    this.rollRate = new PID(0.05, 0.08, 0.0012, 0.15);
    this.pitchRate = new PID(0.05, 0.08, 0.0012, 0.15);
    this.yawRate = new PID(0.25, 0.1, 0, 0.3);
    this.velN = new PID(2.2, 0.4, 0, 2.5);
    this.velE = new PID(2.2, 0.4, 0, 2.5);
    this.velD = new PID(4.0, 1.5, 0, 4.0);
    this.listeners = {};
    this.mission = [];
    this.reset();
  }

  on(evt, fn) { (this.listeners[evt] ||= []).push(fn); }
  emit(evt, ...a) { (this.listeners[evt] || []).forEach((f) => f(...a)); }
  text(msg, severity = 6) { this.emit('text', msg, severity); }

  reset() {
    this.armed = false;
    this.mode = 'Position';
    this.home = [...this.v.pos];
    this.homeSet = false;
    this.posSp = [...this.v.pos];
    this.yawSp = qToEuler(this.v.q)[2];
    this.missionIndex = 0;
    this.missionState = 'idle';
    this.loiterUntil = 0;
    this.gotoTarget = null;
    this.landedTime = 0;
    this.flightTime = 0;
    this.landing = false;
    this.rtlPhase = null;
    this.speedOverride = null;
    [this.rollRate, this.pitchRate, this.yawRate, this.velN, this.velE, this.velD].forEach((p) => p.reset());
    this.throttleOut = 0;
    this.lastSticks = { roll: 0, pitch: 0, yaw: 0, throttle: 0 };
  }

  // ------------------------------------------------------------------ commands
  arm(force = false) {
    if (this.armed) return true;
    if (this.v.crashed) { this.text('Arming denied: vehicle crashed, reset the simulation', 3); return false; }
    if (!force && MANUAL_MODES.includes(this.mode) && this.mode !== 'Position' && this.mode !== 'Altitude' && this.lastSticks.throttle > 0.15) {
      this.text('Arming denied: throttle not low', 4);
      return false;
    }
    this.armed = true;
    this.home = [...this.v.pos];
    this.homeSet = true;
    this.posSp = [...this.v.pos];
    this.yawSp = qToEuler(this.v.q)[2];
    this.landedTime = 0;
    [this.rollRate, this.pitchRate, this.yawRate, this.velN, this.velE, this.velD].forEach((p) => p.reset());
    this.text('Armed');
    this.emit('armed', true);
    return true;
  }

  disarm(reason = '') {
    if (!this.armed) return;
    this.armed = false;
    this.landing = false;
    this.text('Disarmed' + (reason ? ` (${reason})` : ''));
    this.emit('armed', false);
    if (this.mode !== 'Position' && !MANUAL_MODES.includes(this.mode)) this.setMode('Position', true);
  }

  setMode(mode, silent = false) {
    if (!MODES.includes(mode)) return false;
    if (mode === 'Mission' && !this.mission.length) { this.text('No mission uploaded', 4); return false; }
    const prev = this.mode;
    this.mode = mode;
    this.posSp = [...this.v.pos];
    this.yawSp = qToEuler(this.v.q)[2];
    this.gotoTarget = null;
    this.landing = mode === 'Land';
    if (mode === 'Takeoff') {
      this.posSp[2] = this.home[2] - this.params.takeoffAlt;
      if (!this.armed) this.arm(true);
    }
    if (mode === 'Return') this.rtlPhase = 'climb';
    if (mode === 'Mission') {
      if (this.missionState !== 'running' || prev !== 'Mission') {
        if (this.missionState === 'done') this.missionIndex = 0;
        this.missionState = 'running';
        this.itemStarted = false;
      }
      if (!this.armed) this.arm(true);
    }
    if (!silent && prev !== mode) this.text(`Flight mode: ${mode}`);
    this.emit('mode', mode);
    return true;
  }

  takeoff(alt) {
    if (alt) this.params.takeoffAlt = alt;
    return this.setMode('Takeoff');
  }

  land() { return this.setMode('Land'); }
  rtl() { return this.setMode('Return'); }

  gotoLocation(lat, lon, relAlt) {
    if (!this.armed || this.v.onGround) { this.text('Go to location: vehicle must be flying', 4); return false; }
    this.setMode('Hold', true);
    const ned = this.geo.toNed(lat, lon);
    const alt = relAlt ?? (this.home[2] - this.v.pos[2]);
    this.gotoTarget = [ned[0], ned[1], this.home[2] - alt];
    this.text('Go to location');
    return true;
  }

  setMission(items) {
    this.mission = items.map((it) => ({ ...it }));
    this.missionIndex = 0;
    this.missionState = 'idle';
    this.emit('missionCurrent', 0);
  }

  startMission() {
    if (!this.mission.length) { this.text('No mission uploaded', 4); return false; }
    this.missionIndex = 0;
    this.missionState = 'idle';
    return this.setMode('Mission');
  }

  // ------------------------------------------------------------------ update
  update(dt, sticks) {
    this.lastSticks = sticks;
    const v = this.v;
    const [roll, pitch, yaw] = qToEuler(v.q);
    const altRel = this.home[2] - v.pos[2];

    if (!this.armed) {
      this.throttleOut = 0;
      v.setMotors([0, 0, 0, 0]);
      return;
    }
    if (!v.onGround) this.flightTime += dt;

    // Auto-disarm when landed
    const landedNow = v.onGround && (this.landing || sticks.throttle < 0.08 || MANUAL_MODES.indexOf(this.mode) < 0) && this.throttleOut < 0.45;
    if (landedNow && (this.landing || this.mode === 'Land' || (this.mode !== 'Takeoff' && this.mode !== 'Mission' && this.flightTime > 0))) {
      this.landedTime += dt;
      if (this.landedTime > (this.landing ? 1.0 : 3.0)) {
        if (this.mode === 'Mission' && this.missionState === 'running') this.missionState = 'done';
        this.disarm(this.landing ? 'landed' : 'auto disarm');
        return;
      }
    } else this.landedTime = 0;

    let rateSp = null; // body rates
    let attSp = null; // roll, pitch
    let yawRateSp = 0;
    let thrust = 0;
    let velSp = null; // NED velocity setpoint (null for manual modes)
    let posHold = false;
    let climbSp = null;
    const P = this.params;

    const stickYaw = deadband(sticks.yaw, 0.05);
    const stickRoll = deadband(sticks.roll, 0.03);
    const stickPitch = deadband(sticks.pitch, 0.03);
    const thrCentered = deadband((sticks.throttle - 0.5) * 2, 0.1);

    switch (this.mode) {
      case 'Acro':
        rateSp = [stickRoll * P.acroRateDeg * DEG, stickPitch * P.acroRateDeg * DEG, stickYaw * P.maxYawRateDeg * DEG];
        thrust = sticks.throttle * 0.95;
        break;
      case 'Stabilized':
        attSp = [stickRoll * P.maxTiltDeg * DEG, stickPitch * P.maxTiltDeg * DEG];
        yawRateSp = stickYaw * P.maxYawRateDeg * DEG;
        thrust = sticks.throttle * 0.9 / Math.max(0.6, Math.cos(roll) * Math.cos(pitch));
        this.yawSp = yaw;
        break;
      case 'Altitude':
        attSp = [stickRoll * P.maxTiltDeg * DEG, stickPitch * P.maxTiltDeg * DEG];
        yawRateSp = stickYaw * P.maxYawRateDeg * DEG;
        climbSp = thrCentered * (thrCentered > 0 ? P.maxClimb : P.maxDescent * 1.5);
        this.yawSp = yaw;
        break;
      case 'Position': {
        const fwd = stickPitch * -1; // stick forward (pitch negative) = move forward
        const right = stickRoll;
        const sp = P.maxHorizSpeed;
        if (Math.abs(fwd) > 0 || Math.abs(right) > 0) {
          velSp = [
            (Math.cos(yaw) * fwd - Math.sin(yaw) * right) * sp,
            (Math.sin(yaw) * fwd + Math.cos(yaw) * right) * sp,
            0,
          ];
          this.posSp[0] = v.pos[0];
          this.posSp[1] = v.pos[1];
        } else posHold = true;
        climbSp = thrCentered * (thrCentered > 0 ? P.maxClimb : P.maxDescent * 1.5);
        yawRateSp = stickYaw * P.maxYawRateDeg * DEG;
        this.yawSp = yaw;
        if (v.onGround && climbSp <= 0) { thrust = 0.05; climbSp = null; velSp = null; posHold = false; attSp = [0, 0]; }
        break;
      }
      case 'Hold':
        if (this.gotoTarget) {
          const r = this.navigateTo(this.gotoTarget, P.cruiseSpeed);
          velSp = r.vel;
          if (r.reached) { this.posSp = [...this.gotoTarget]; this.gotoTarget = null; this.text('Reached location'); }
        } else posHold = true;
        break;
      case 'Takeoff': {
        const tgt = [this.posSp[0], this.posSp[1], this.home[2] - P.takeoffAlt];
        velSp = this.navigateTo(tgt, P.cruiseSpeed).vel;
        if (Math.abs(altRel - P.takeoffAlt) < 0.4) {
          this.text(`Takeoff complete (${P.takeoffAlt.toFixed(0)} m)`);
          this.setMode('Hold', true);
          this.posSp[2] = this.home[2] - P.takeoffAlt;
          this.emit('mode', 'Hold');
        }
        break;
      }
      case 'Land':
        velSp = [0, 0, altRel > 10 ? P.maxDescent : P.landSpeed];
        this.landing = true;
        velSp[0] = (this.posSp[0] - v.pos[0]) * 0.8;
        velSp[1] = (this.posSp[1] - v.pos[1]) * 0.8;
        break;
      case 'Return':
        velSp = this.updateRtl(altRel);
        break;
      case 'Mission':
        velSp = this.updateMission(dt, altRel);
        break;
    }

    // ---------------- position & velocity loops
    let accSp = null;
    if (posHold || velSp || climbSp !== null) {
      if (posHold) {
        velSp = [
          clamp((this.posSp[0] - v.pos[0]) * 1.0, -P.maxHorizSpeed, P.maxHorizSpeed),
          clamp((this.posSp[1] - v.pos[1]) * 1.0, -P.maxHorizSpeed, P.maxHorizSpeed),
          velSp ? velSp[2] : 0,
        ];
      }
      if (climbSp !== null) {
        if (Math.abs(climbSp) > 0.01) { this.posSp[2] = v.pos[2]; velSp = velSp || [null, null, 0]; velSp[2] = -climbSp; }
        else { velSp = velSp || [null, null, 0]; velSp[2] = clamp((this.posSp[2] - v.pos[2]) * 1.2, -P.maxClimb, P.maxDescent); }
      } else if (posHold && !this.gotoTarget && this.mode !== 'Land') {
        velSp[2] = clamp((this.posSp[2] - v.pos[2]) * 1.2, -P.maxClimb, P.maxDescent);
      }
      const aN = velSp[0] === null ? null : this.velN.update(velSp[0] - v.vel[0], dt);
      const aE = velSp[1] === null ? null : this.velE.update(velSp[1] - v.vel[1], dt);
      const aD = this.velD.update(velSp[2] - v.vel[2], dt);
      accSp = [aN, aE, clamp(aD, -8, 6)];
    }

    if (accSp) {
      const thrustAccZ = G - accSp[2];
      if (accSp[0] !== null) {
        // horizontal acceleration -> tilt, expressed in heading frame
        const maxA = Math.tan(P.maxTiltDeg * DEG) * thrustAccZ;
        let aN = accSp[0], aE = accSp[1];
        const aH = Math.hypot(aN, aE);
        if (aH > maxA) { aN *= maxA / aH; aE *= maxA / aH; }
        const aF = Math.cos(yaw) * aN + Math.sin(yaw) * aE;
        const aR = -Math.sin(yaw) * aN + Math.cos(yaw) * aE;
        const pSp = Math.atan2(-aF, thrustAccZ);
        const rSp = Math.atan2(aR * Math.cos(pSp), thrustAccZ);
        attSp = [rSp, pSp];
        thrust = (this.v.p.mass * Math.hypot(aN, aE, thrustAccZ)) / (4 * this.v.p.maxThrustPerMotor);
      } else {
        thrust = (this.v.p.mass * thrustAccZ) / (4 * this.v.p.maxThrustPerMotor) / Math.max(0.6, Math.cos(roll) * Math.cos(pitch));
      }
    }

    // ---------------- yaw
    if (!rateSp) {
      if (Math.abs(yawRateSp) > 0) this.yawSp = yaw;
      else if (this.mode === 'Stabilized' || this.mode === 'Altitude' || this.mode === 'Position') this.yawSp = this.yawSp ?? yaw;
      const yawErr = wrapPi(this.yawSp - yaw);
      const yawRate = Math.abs(yawRateSp) > 0 ? yawRateSp : clamp(yawErr * 2.5, -P.maxYawRateDeg * DEG, P.maxYawRateDeg * DEG);
      // attitude P loop -> body rates (small-angle, adequate for +-45 deg)
      const kAtt = 7.0;
      const rollRateSp = kAtt * (attSp[0] - roll);
      const pitchRateSp = kAtt * (attSp[1] - pitch);
      rateSp = [
        rollRateSp - Math.sin(pitch) * yawRate,
        pitchRateSp * Math.cos(roll) + Math.sin(roll) * Math.cos(pitch) * yawRate,
        -Math.sin(roll) * pitchRateSp + Math.cos(roll) * Math.cos(pitch) * yawRate,
      ];
    }

    // ---------------- rate loop -> normalised torques
    const w = v.omega;
    const inAir = !v.onGround;
    if (!inAir) { this.rollRate.i = 0; this.pitchRate.i = 0; this.yawRate.i = 0; }
    const tr = this.rollRate.update(rateSp[0] - w[0], dt);
    const tp = this.pitchRate.update(rateSp[1] - w[1], dt);
    const ty = clamp(this.yawRate.update(rateSp[2] - w[2], dt), -0.25, 0.25);

    thrust = clamp(thrust, 0, 1);
    this.throttleOut = thrust;
    const out = this.mix(thrust, tr, tp, ty);
    v.setMotors(out);
  }

  mix(thrust, r, p, y) {
    const raw = MIX.map((m) => m[0] * r + m[1] * p + m[2] * y);
    let lo = Math.min(...raw), hi = Math.max(...raw);
    // airmode-like: keep the attitude authority by shifting collective
    let t = thrust;
    if (t + hi > 1) t = 1 - hi;
    if (t + lo < 0.03 && this.armed) t = Math.min(0.03 - lo, thrust + 0.15);
    const idle = this.armed ? 0.05 : 0;
    return raw.map((x) => clamp(t + x, idle, 1));
  }

  navigateTo(tgt, speed) {
    const v = this.v;
    const d = [tgt[0] - v.pos[0], tgt[1] - v.pos[1], tgt[2] - v.pos[2]];
    const dh = Math.hypot(d[0], d[1]);
    const P = this.params;
    // speed profile: decelerate close to the target (sqrt law)
    const hs = Math.min(speed, Math.sqrt(2 * 1.8 * dh), dh * 1.2);
    const vel = [dh > 0.01 ? (d[0] / dh) * hs : 0, dh > 0.01 ? (d[1] / dh) * hs : 0, clamp(d[2] * 1.2, -P.maxClimb, P.maxDescent)];
    if (dh > 3) this.yawSp = Math.atan2(d[1], d[0]);
    return { vel, reached: dh < P.acceptRadius && Math.abs(d[2]) < 0.8, dist: dh };
  }

  updateRtl(altRel) {
    const P = this.params;
    const v = this.v;
    const rtlAlt = Math.max(P.rtlAlt, altRel);
    if (this.rtlPhase === 'climb') {
      if (altRel >= rtlAlt - 0.5 || altRel >= P.rtlAlt - 0.5) { this.rtlPhase = 'return'; this.rtlZ = v.pos[2]; this.text('RTL: returning home'); }
      return [0, 0, -P.maxClimb];
    }
    if (this.rtlPhase === 'return') {
      const r = this.navigateTo([this.home[0], this.home[1], this.rtlZ], P.cruiseSpeed * 1.6);
      if (r.dist < 1.0) { this.rtlPhase = 'land'; this.posSp = [this.home[0], this.home[1], v.pos[2]]; this.text('RTL: landing'); }
      return r.vel;
    }
    this.landing = true;
    return [(this.home[0] - v.pos[0]) * 0.8, (this.home[1] - v.pos[1]) * 0.8, altRel > 10 ? P.maxDescent : P.landSpeed];
  }

  updateMission(dt, altRel) {
    const v = this.v;
    const P = this.params;
    if (this.missionState !== 'running') { this.posSp = this.posSp || [...v.pos]; return [(this.posSp[0] - v.pos[0]), (this.posSp[1] - v.pos[1]), (this.posSp[2] - v.pos[2])]; }
    const item = this.mission[this.missionIndex];
    if (!item) {
      this.missionState = 'done';
      this.text('Mission finished');
      this.setMode('Hold', true);
      this.emit('mode', 'Hold');
      return [0, 0, 0];
    }
    const next = () => {
      this.emit('missionReached', this.missionIndex);
      this.missionIndex++;
      this.itemStarted = false;
      this.loiterUntil = 0;
      this.emit('missionCurrent', this.missionIndex);
    };
    if (!this.itemStarted) { this.itemStarted = true; this.itemStartPos = [...v.pos]; this.emit('missionCurrent', this.missionIndex); }
    const speed = this.speedOverride || P.cruiseSpeed;
    const tgtFor = (it) => {
      const hasPos = it.lat !== undefined && (it.lat !== 0 || it.lon !== 0);
      const ned = hasPos ? this.geo.toNed(it.lat, it.lon) : [this.itemStartPos[0], this.itemStartPos[1]];
      return [ned[0], ned[1], this.home[2] - (it.alt ?? altRel)];
    };

    switch (item.command) {
      case MAV_CMD.NAV_TAKEOFF: {
        const tgt = [this.itemStartPos[0], this.itemStartPos[1], this.home[2] - (item.alt || P.takeoffAlt)];
        const r = this.navigateTo(tgt, speed);
        if (Math.abs(tgt[2] - v.pos[2]) < 0.5) { this.text(`Mission: takeoff complete`); next(); }
        return r.vel;
      }
      case MAV_CMD.NAV_WAYPOINT:
      case MAV_CMD.NAV_LOITER_TIME: {
        const tgt = tgtFor(item);
        const r = this.navigateTo(tgt, speed);
        if (r.reached) {
          const hold = item.command === MAV_CMD.NAV_LOITER_TIME ? item.param1 : item.param1 || 0;
          if (!this.loiterUntil) { this.loiterUntil = v.time + hold; if (hold > 0) this.text(`Mission: holding ${hold}s at item ${this.missionIndex + 1}`); }
          if (v.time >= this.loiterUntil) { this.text(`Mission: reached item ${this.missionIndex + 1}`); next(); }
          return [(tgt[0] - v.pos[0]) * 1, (tgt[1] - v.pos[1]) * 1, (tgt[2] - v.pos[2]) * 1];
        }
        return r.vel;
      }
      case MAV_CMD.DO_CHANGE_SPEED:
        if (item.param2 > 0) { this.speedOverride = item.param2; this.text(`Mission: speed ${item.param2} m/s`); }
        next();
        return [0, 0, 0];
      case MAV_CMD.NAV_RETURN_TO_LAUNCH:
        this.text('Mission: return to launch');
        this.missionState = 'done';
        this.setMode('Return', true);
        this.emit('mode', 'Return');
        return [0, 0, 0];
      case MAV_CMD.NAV_LAND: {
        const hasPos = item.lat || item.lon;
        const tgt = hasPos ? tgtFor({ ...item, alt: altRel }) : [v.pos[0], v.pos[1], v.pos[2]];
        const d = Math.hypot(tgt[0] - v.pos[0], tgt[1] - v.pos[1]);
        if (d > 1.5) return this.navigateTo(tgt, speed).vel;
        this.landing = true;
        this.posSp = [tgt[0], tgt[1], v.pos[2]];
        return [(tgt[0] - v.pos[0]) * 0.8, (tgt[1] - v.pos[1]) * 0.8, altRel > 10 ? P.maxDescent : P.landSpeed];
      }
      default:
        this.text(`Mission: unsupported command ${item.command}, skipping`, 4);
        next();
        return [0, 0, 0];
    }
  }

  /** Telemetry snapshot for the GCS / HUD. */
  telemetry() {
    const v = this.v;
    const [roll, pitch, yaw] = qToEuler(v.q);
    const g = this.geo.toGeo(v.pos);
    const homeGeo = this.geo.toGeo(this.home);
    return {
      source: 'internal',
      armed: this.armed,
      mode: this.mode,
      roll, pitch, yaw,
      rates: [...v.omega],
      lat: g.lat, lon: g.lon, altAmsl: g.alt,
      altRel: this.home[2] - v.pos[2],
      vel: [...v.vel],
      groundSpeed: Math.hypot(v.vel[0], v.vel[1]),
      climb: -v.vel[2],
      heading: ((yaw * 180) / Math.PI + 360) % 360,
      throttle: this.throttleOut,
      battery: { ...v.battery },
      gps: { fix: 3, sats: 14, hdop: 0.7 },
      home: { lat: homeGeo.lat, lon: homeGeo.lon, alt: homeGeo.alt },
      landed: v.onGround,
      flightTime: this.flightTime,
      missionIndex: this.missionIndex,
      missionState: this.missionState,
      distHome: Math.hypot(v.pos[0] - this.home[0], v.pos[1] - this.home[1]),
      motors: [...v.motorOut],
      thrustDir: qRotate(v.q, [0, 0, -1]),
      speedNorm: norm(v.vel),
    };
  }
}
