// Sensor models that turn the true vehicle state into what a Pixhawk would
// measure. Used both for HIL_SENSOR / HIL_GPS (hardware-in-the-loop) and
// by the internal flight controller in simulation-only mode.

import { qRotateInv, randn, DEG } from '../core/math.js';
import { pressureAt, temperatureAt } from '../core/geo.js';

// Earth magnetic field approximation (Gauss) given declination / inclination / intensity.
export function magField(declDeg, inclDeg, intensity) {
  const d = declDeg * DEG, i = inclDeg * DEG;
  const h = intensity * Math.cos(i);
  return [h * Math.cos(d), h * Math.sin(d), intensity * Math.sin(i)];
}

export class SensorSuite {
  constructor(geo, opts = {}) {
    this.geo = geo;
    this.noise = opts.noise ?? 1;
    // Defaults approximate the WMM at Dhahran, Saudi Arabia (KFUPM).
    this.mag = magField(opts.declination ?? 2.9, opts.inclination ?? 37.5, opts.intensity ?? 0.44);
    this.gyroBias = [randn() * 0.002, randn() * 0.002, randn() * 0.002];
    this.accelBias = [randn() * 0.02, randn() * 0.02, randn() * 0.02];
    this.baroDrift = 0;
    this.gpsErr = [0, 0, 0];
  }

  setMagModel(decl, incl, intensity) {
    this.mag = magField(decl, incl, intensity);
  }

  imu(vehicle) {
    const n = this.noise;
    const acc = vehicle.specificForce.map((v, k) => v + this.accelBias[k] + randn() * 0.03 * n);
    const gyro = vehicle.omega.map((v, k) => v + this.gyroBias[k] + randn() * 0.004 * n);
    const magB = qRotateInv(vehicle.q, this.mag).map((v) => v + randn() * 0.003 * n);
    return { acc, gyro, mag: magB };
  }

  baro(vehicle, dt) {
    this.baroDrift += randn() * 0.002 * this.noise * Math.sqrt(dt || 0.004);
    const alt = this.geo.alt - vehicle.pos[2] + this.baroDrift + randn() * 0.05 * this.noise;
    return { pressure: pressureAt(alt), alt, temperature: temperatureAt(alt) + 10 };
  }

  gps(vehicle, dt) {
    // Slowly wandering position error (random walk with mean reversion)
    for (let k = 0; k < 3; k++) {
      this.gpsErr[k] += (-this.gpsErr[k] * 0.05 + randn() * 0.15 * this.noise) * (dt || 0.1);
    }
    const ned = [vehicle.pos[0] + this.gpsErr[0], vehicle.pos[1] + this.gpsErr[1], vehicle.pos[2] + this.gpsErr[2] * 1.5];
    const g = this.geo.toGeo(ned);
    const vel = vehicle.vel.map((v) => v + randn() * 0.03 * this.noise);
    const ground = Math.hypot(vel[0], vel[1]);
    let cog = Math.atan2(vel[1], vel[0]) / DEG;
    if (cog < 0) cog += 360;
    return { lat: g.lat, lon: g.lon, alt: g.alt, vel, groundSpeed: ground, cog, eph: 0.8, epv: 1.2, sats: 14, fix: 3 };
  }
}
