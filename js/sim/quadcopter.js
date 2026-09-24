// 6-DOF rigid-body model of a DJI F450-class quadcopter.
//
// Frames: world = NED (north, east, down), body = FRD (forward, right, down).
// Motor layout / numbering follows PX4 & ArduPilot "Quad X":
//
//          front
//      3(CW)   1(CCW)
//          \ /
//          / \
//      2(CCW)  4(CW)
//
// Motor commands are normalised 0..1 (PWM 1000..2000us).

import {
  G, add, scale, cross, qRotate, qRotateInv, qIntegrate, qIdentity, qFromEuler, clamp, norm, randn,
} from '../core/math.js';

export const F450 = {
  name: 'DJI F450',
  mass: 1.25, // kg (frame 282g + 4x 2212 motors + 4x ESC + 3S 2200mAh + Pixhawk + GPS)
  armLength: 0.225, // m (450 mm motor-to-motor diagonal)
  inertia: [0.0118, 0.0118, 0.0215], // kg m^2
  maxThrustPerMotor: 8.4, // N  (2212 920KV + 10x4.5 on 3S, ~860 g)
  motorTau: 0.045, // s  motor/ESC time constant
  yawMomentCoef: 0.016, // Nm of reaction torque per N of thrust
  dragLinear: [0.12, 0.12, 0.25], // N/(m/s) body
  dragQuadratic: [0.045, 0.045, 0.09], // N/(m/s)^2 body
  angularDrag: 0.0025, // Nm/(rad/s)
  maxRpm: 10200,
  propDiameter: 0.254, // 10 inch
  battery: { cells: 3, capacityMah: 2200, cellFull: 4.2, cellEmpty: 3.3, internalR: 0.018 },
};

// Motor positions in body FRD (x fwd, y right) and spin direction (+1 = CCW seen from above)
const S = Math.SQRT1_2;
export const MOTORS = [
  { pos: [S, S, 0], dir: +1 }, // 1 front-right CCW
  { pos: [-S, -S, 0], dir: +1 }, // 2 rear-left CCW
  { pos: [S, -S, 0], dir: -1 }, // 3 front-left CW
  { pos: [-S, S, 0], dir: -1 }, // 4 rear-right CW
];

export class Quadcopter {
  constructor(params = F450) {
    this.p = params;
    this.reset();
    this.wind = [0, 0, 0];
    this.windGust = 0;
  }

  reset(pos = [0, 0, 0], yaw = 0) {
    this.pos = [...pos]; // NED m
    this.vel = [0, 0, 0]; // NED m/s
    this.acc = [0, 0, 0]; // NED m/s^2 (kinematic)
    this.q = yaw ? qFromEuler(0, 0, yaw) : qIdentity();
    this.omega = [0, 0, 0]; // body rad/s
    this.motorCmd = [0, 0, 0, 0];
    this.motorOut = [0, 0, 0, 0]; // 0..1 (thrust fraction)
    this.propAngle = [0, 0, 0, 0];
    this.onGround = true;
    this.crashed = false;
    this.time = 0;
    this.specificForce = [0, 0, -G]; // body, what an accelerometer measures
    this.battery = {
      voltage: this.p.battery.cells * this.p.battery.cellFull,
      current: 0,
      consumedMah: 0,
      remaining: 1,
    };
    this.groundHeight = 0; // terrain height (NED down is -height)
  }

  setMotors(cmds) {
    for (let i = 0; i < 4; i++) this.motorCmd[i] = clamp(cmds[i] || 0, 0, 1);
  }

  step(dt) {
    const p = this.p;
    this.time += dt;

    // --- Battery sag reduces available thrust
    const b = this.battery;
    const sag = clamp((b.voltage / (p.battery.cells * p.battery.cellFull)) ** 2, 0.55, 1);

    // --- Motors (first order lag)
    const a = dt / (p.motorTau + dt);
    let totalThrust = 0;
    let yawTorque = 0;
    const torque = [0, 0, 0];
    for (let i = 0; i < 4; i++) {
      this.motorOut[i] += (this.motorCmd[i] - this.motorOut[i]) * a;
      const thrust = this.motorOut[i] * p.maxThrustPerMotor * sag;
      totalThrust += thrust;
      const m = MOTORS[i];
      const r = [m.pos[0] * p.armLength, m.pos[1] * p.armLength, -0.03];
      const t = cross(r, [0, 0, -thrust]);
      torque[0] += t[0];
      torque[1] += t[1];
      // CCW spinning prop produces CW reaction torque on the body -> negative
      // yaw moment about the down axis is ... in FRD, positive z torque = yaw right (CW from above).
      // A CCW (seen from above) prop pushes the frame CW => +z.
      yawTorque += m.dir * thrust * p.yawMomentCoef;
      // prop visual angle (rad) — CCW from above is negative rotation around FRD z
      const rpm = Math.sqrt(this.motorOut[i]) * p.maxRpm;
      this.propAngle[i] += -m.dir * rpm * (2 * Math.PI / 60) * dt;
    }
    torque[2] += yawTorque;

    // --- Battery model
    const hoverFrac = totalThrust / (4 * p.maxThrustPerMotor);
    b.current = 0.6 + 62 * Math.pow(hoverFrac, 1.5);
    b.consumedMah += (b.current * dt * 1000) / 3600;
    b.remaining = clamp(1 - b.consumedMah / p.battery.capacityMah, 0, 1);
    const cellV = p.battery.cellEmpty + (p.battery.cellFull - p.battery.cellEmpty) * (0.15 + 0.85 * Math.pow(b.remaining, 0.8));
    b.voltage = p.battery.cells * (b.remaining > 0 ? cellV : p.battery.cellEmpty * 0.9) - b.current * p.battery.internalR;

    // --- Aerodynamic drag (body frame, relative to wind)
    const gust = this.windGust > 0 ? scale([randn(), randn(), randn() * 0.3], this.windGust) : [0, 0, 0];
    const airVelWorld = [this.vel[0] - this.wind[0] - gust[0], this.vel[1] - this.wind[1] - gust[1], this.vel[2] - this.wind[2] - gust[2]];
    const vb = qRotateInv(this.q, airVelWorld);
    const drag = [0, 1, 2].map((k) => -p.dragLinear[k] * vb[k] - p.dragQuadratic[k] * vb[k] * Math.abs(vb[k]));

    // --- Forces
    const fBody = [drag[0], drag[1], -totalThrust + drag[2]];
    const fWorld = qRotate(this.q, fBody);
    let accW = [fWorld[0] / p.mass, fWorld[1] / p.mass, fWorld[2] / p.mass + G];

    // --- Rotational dynamics: I*dw = tau - w x (I w) - drag
    const I = p.inertia;
    const w = this.omega;
    const Iw = [I[0] * w[0], I[1] * w[1], I[2] * w[2]];
    const gyro = cross(w, Iw);
    const dw = [
      (torque[0] - gyro[0] - p.angularDrag * w[0]) / I[0],
      (torque[1] - gyro[1] - p.angularDrag * w[1]) / I[1],
      (torque[2] - gyro[2] - p.angularDrag * 2 * w[2]) / I[2],
    ];

    // --- Ground contact
    const groundZ = -this.groundHeight;
    const onGroundNow = this.pos[2] >= groundZ - 0.001 && accW[2] >= 0 && this.vel[2] >= -0.05;
    if (onGroundNow) {
      // resting on landing gear: friction + no penetration
      if (!this.onGround && norm(this.vel) > 4.5) this.crashed = true;
      this.onGround = true;
      accW = [-this.vel[0] * 6, -this.vel[1] * 6, 0];
      this.vel[2] = 0;
      this.pos[2] = groundZ;
      // settle attitude towards level
      this.omega = scale(this.omega, 0.6);
      const up = qRotate(this.q, [0, 0, 1]);
      const tilt = Math.acos(clamp(up[2], -1, 1));
      if (tilt > 0.01) {
        const axisWorld = cross(up, [0, 0, 1]);
        const axisBody = qRotateInv(this.q, axisWorld);
        this.omega = add(this.omega, scale(axisBody, Math.min(tilt, 0.6) * 12 * dt * 10));
      }
      this.omega[2] += dw[2] * dt * 0.2;
    } else {
      this.onGround = false;
      this.omega = add(this.omega, scale(dw, dt));
    }

    this.vel = add(this.vel, scale(accW, dt));
    this.pos = add(this.pos, scale(this.vel, dt));
    if (this.pos[2] > groundZ) {
      if (this.vel[2] > 4.5) this.crashed = true;
      this.pos[2] = groundZ;
      if (this.vel[2] > 0) this.vel[2] = 0;
    }
    this.q = qIntegrate(this.q, this.omega, dt);
    this.acc = accW;

    // Specific force (accelerometer) = R^T (a - g)
    this.specificForce = qRotateInv(this.q, [accW[0], accW[1], accW[2] - G]);
  }

  /** Thrust-to-hover fraction for the current mass (handy for controllers). */
  get hoverThrottle() {
    return (this.p.mass * G) / (4 * this.p.maxThrustPerMotor);
  }
}
