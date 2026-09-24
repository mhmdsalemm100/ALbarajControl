// Headless flight test of the physics + internal autopilot:
//   node tests/flight.test.mjs
import { Quadcopter } from '../js/sim/quadcopter.js';
import { Autopilot } from '../js/sim/autopilot.js';
import { GeoOrigin } from '../js/core/geo.js';
import { qToEuler } from '../js/core/math.js';

const geo = new GeoOrigin(26.307, 50.145, 70);
const quad = new Quadcopter();
const ap = new Autopilot(quad, geo);
const log = [];
ap.on('text', (t) => log.push(`[${quad.time.toFixed(1)}s] ${t}`));
const dt = 0.002;
const sticks = { roll: 0, pitch: 0, yaw: 0, throttle: 0.5 };
let maxTilt = 0;
function run(sec, cond) {
  const n = sec / dt;
  for (let i = 0; i < n; i++) {
    ap.update(dt, sticks);
    quad.step(dt);
    const [r, p] = qToEuler(quad.q);
    maxTilt = Math.max(maxTilt, Math.abs(r), Math.abs(p));
    if (cond && cond()) return true;
  }
  return false;
}
const alt = () => -quad.pos[2];
const fail = (m) => { console.log(log.join('\n')); console.error('FAIL:', m); process.exit(1); };
const ok = (m) => console.log('ok -', m);

ap.takeoff(10);
run(12);
if (Math.abs(alt() - 10) > 0.5) fail(`takeoff alt ${alt()}`);
if (Math.hypot(quad.pos[0], quad.pos[1]) > 0.5) fail(`takeoff drift ${quad.pos}`);
ok(`takeoff to ${alt().toFixed(2)} m, maxTilt ${(maxTilt * 57.3).toFixed(1)}°`);

// Stabilized: hover a few seconds with rates small
const w0 = Math.hypot(...quad.omega);
if (w0 > 0.05) fail(`rates not settled ${w0}`);

// Position mode stick forward for 4 s
ap.setMode('Position');
sticks.pitch = -1;
run(4);
sticks.pitch = 0;
const vN = quad.vel[0];
if (vN < 8) fail(`position mode forward speed ${vN}`);
run(6);
if (Math.hypot(quad.vel[0], quad.vel[1]) > 0.3) fail('did not stop');
ok(`position mode: flew fwd at ${vN.toFixed(1)} m/s then stopped at N=${quad.pos[0].toFixed(1)}`);

// Mission
const wp = (n, e, a) => { const g = geo.toGeo([n, e, 0]); return { command: 16, lat: g.lat, lon: g.lon, alt: a, param1: 0 }; };
ap.setMission([
  { command: 22, alt: 15 },
  wp(40, 0, 20), wp(40, 40, 20), { command: 19, ...wp(0, 40, 15), param1: 3 }, { command: 20 },
]);
ap.startMission();
maxTilt = 0;
const done = run(120, () => !ap.armed);
if (!done) fail(`mission not finished, mode ${ap.mode} idx ${ap.missionIndex} pos ${quad.pos.map((x) => x.toFixed(1))}`);
if (Math.hypot(quad.pos[0], quad.pos[1]) > 1.5) fail(`landed away from home ${quad.pos}`);
if (quad.crashed) fail('crashed');
ok(`mission + RTL completed at t=${quad.time.toFixed(1)} s, landed ${Math.hypot(quad.pos[0], quad.pos[1]).toFixed(2)} m from home, maxTilt ${(maxTilt * 57.3).toFixed(1)}°`);

// Stabilized manual flight: arm, throttle up, hover-ish
ap.setMode('Stabilized');
sticks.throttle = 0;
if (!ap.arm()) fail('arm in stabilized');
sticks.throttle = 0.7;
run(2);
if (alt() < 1) fail(`stabilized climb ${alt()}`);
sticks.roll = 0.5;
run(0.5);
const roll = qToEuler(quad.q)[0] * 57.3;
if (Math.abs(roll - 17.5) > 3) fail(`stabilized roll ${roll}`);
ok(`stabilized: climbed to ${alt().toFixed(1)} m, roll ${roll.toFixed(1)}° for 50% stick`);
