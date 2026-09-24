// Small, allocation-light vector / quaternion helpers used by the physics
// and flight-control code. All vectors are plain [x, y, z] arrays and all
// quaternions are [w, x, y, z] (Hamilton convention, body -> world).

export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;
export const G = 9.80665;

export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
export const lerp = (a, b, t) => a + (b - a) * t;
export const wrapPi = (a) => {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
};
export const deadband = (v, d) => (Math.abs(v) < d ? 0 : (v - Math.sign(v) * d) / (1 - d));

export const v3 = (x = 0, y = 0, z = 0) => [x, y, z];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export const normalize = (a) => {
  const n = norm(a) || 1;
  return [a[0] / n, a[1] / n, a[2] / n];
};

export const qIdentity = () => [1, 0, 0, 0];
export const qMul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
];
export const qConj = (q) => [q[0], -q[1], -q[2], -q[3]];
export const qNormalize = (q) => {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
};

/** Rotate vector v from body to world frame with quaternion q. */
export function qRotate(q, v) {
  const [w, x, y, z] = q;
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx),
  ];
}
/** Rotate vector v from world to body frame. */
export const qRotateInv = (q, v) => qRotate(qConj(q), v);

/** Integrate quaternion with body angular rate w (rad/s) over dt. */
export function qIntegrate(q, w, dt) {
  const ang = norm(w) * dt;
  if (ang < 1e-9) return q;
  const ax = scale(w, 1 / norm(w));
  const s = Math.sin(ang / 2);
  return qNormalize(qMul(q, [Math.cos(ang / 2), ax[0] * s, ax[1] * s, ax[2] * s]));
}

/** ZYX (yaw-pitch-roll) euler -> quaternion. */
export function qFromEuler(roll, pitch, yaw) {
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
  return [
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ];
}

/** Quaternion -> [roll, pitch, yaw] (ZYX). */
export function qToEuler(q) {
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(clamp(2 * (w * y - z * x), -1, 1));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw];
}

/** Gaussian noise (Box-Muller). */
export function randn() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

/** First-order low-pass filter helper. */
export class LowPass {
  constructor(tau, init = 0) { this.tau = tau; this.y = init; }
  update(x, dt) { this.y += (x - this.y) * (dt / (this.tau + dt)); return this.y; }
}

export class PID {
  constructor(kp, ki = 0, kd = 0, iLimit = 1, outLimit = Infinity) {
    Object.assign(this, { kp, ki, kd, iLimit, outLimit });
    this.i = 0;
    this.prev = null;
  }
  reset() { this.i = 0; this.prev = null; }
  update(err, dt, dMeas = null) {
    this.i = clamp(this.i + err * this.ki * dt, -this.iLimit, this.iLimit);
    let d = 0;
    if (dMeas !== null) d = -dMeas * this.kd;
    else if (this.prev !== null) d = ((err - this.prev) / dt) * this.kd;
    this.prev = err;
    return clamp(this.kp * err + this.i + d, -this.outLimit, this.outLimit);
  }
}
