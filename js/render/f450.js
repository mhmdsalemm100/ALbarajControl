// Detailed, true-to-scale procedural model of a DJI Flame Wheel F450 with a
// Pixhawk flight controller, GPS mast, 2212 motors, 10x4.5 props, 3S LiPo
// and tall landing gear.
//
// Model axes (three.js body frame): forward = -Z, right = +X, up = +Y.
// The origin is the centre of the frame at arm level (≈ centre of gravity).

import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import {
  pcbTexture, batteryLabelTexture, fcLabelTexture, propBlurTexture, carbonTexture,
} from './textures.js';

export const GEAR_HEIGHT = 0.175; // distance from CG to the ground when landed (m)
const ARM = 0.225; // centre to motor axis (m)
const PROP_R = 0.127; // 10 inch props

// Motor index -> body FRD position (PX4 quad X) -> three.js coords
const S = Math.SQRT1_2;
const MOTOR_FRD = [[S, S], [-S, -S], [S, -S], [-S, S]];
const MOTOR_DIR = [+1, +1, -1, -1]; // +1 = CCW from above

function mat(opts) { return new THREE.MeshPhysicalMaterial(opts); }

function plateShape(size, chamfer, r = 0.006) {
  // octagon: long sides front/back/left/right, short chamfered corners on the diagonals
  const a = size / 2, c = chamfer / 2;
  const pts = [
    [a - c, -a], [a, -a + c], [a, a - c], [a - c, a], [-a + c, a], [-a, a - c], [-a, -a + c], [-a + c, -a],
  ];
  const shape = new THREE.Shape();
  pts.forEach(([x, y], i) => {
    // rounded corners via quadratic curves
    const [px, py] = pts[(i + pts.length - 1) % pts.length];
    const [nx, ny] = pts[(i + 1) % pts.length];
    const d1 = Math.hypot(x - px, y - py), d2 = Math.hypot(nx - x, ny - y);
    const s1 = [x - ((x - px) / d1) * r, y - ((y - py) / d1) * r];
    const s2 = [x + ((nx - x) / d2) * r, y + ((ny - y) / d2) * r];
    if (i === 0) shape.moveTo(...s1); else shape.lineTo(...s1);
    shape.quadraticCurveTo(x, y, ...s2);
  });
  shape.closePath();
  return shape;
}

function taperedArm(len, hRoot, hTip, wRoot, wTip) {
  const g = new THREE.BoxGeometry(len, 1, 1, 16, 2, 2);
  const p = g.attributes.position;
  for (let i = 0; i < p.count; i++) {
    const x = p.getX(i);
    const t = x / len + 0.5;
    const h = hRoot + (hTip - hRoot) * t;
    const w = wRoot + (wTip - wRoot) * t;
    // slightly arched top like the moulded F450 arm
    const arch = Math.sin(t * Math.PI) * 0.003;
    p.setY(i, p.getY(i) * h + (p.getY(i) > 0 ? arch : 0));
    p.setZ(i, p.getZ(i) * w);
    p.setX(i, x + len / 2);
  }
  g.computeVertexNormals();
  return g;
}

function bladeGeometry(dir) {
  // planform of a 10x4.5 slow-fly blade (x = span, y = chord)
  const shape = new THREE.Shape();
  const root = 0.011, tip = PROP_R;
  const chord = (r) => {
    const t = (r - root) / (tip - root);
    return 0.012 + 0.014 * Math.sin(Math.min(1, t * 1.8) * Math.PI * 0.5) * (1 - t * 0.55) - 0.006 * t * t;
  };
  const N = 18;
  shape.moveTo(root, -0.006);
  for (let i = 0; i <= N; i++) { const r = root + ((tip - root) * i) / N; shape.lineTo(r, -chord(r) * 0.35); }
  shape.quadraticCurveTo(tip + 0.004, 0, tip - 0.002, chord(tip) * 0.6);
  for (let i = N; i >= 0; i--) { const r = root + ((tip - root) * i) / N; shape.lineTo(r, chord(r) * 0.65); }
  shape.lineTo(root, 0.006);
  const g = new THREE.ExtrudeGeometry(shape, { depth: 0.0016, bevelEnabled: true, bevelThickness: 0.0004, bevelSize: 0.0005, bevelSegments: 1, curveSegments: 6 });
  g.translate(0, 0, -0.0008);
  // lay flat (chord along Z) and apply twist: pitch decreasing towards the tip
  g.rotateX(-Math.PI / 2);
  const p = g.attributes.position;
  const v = new THREE.Vector3();
  for (let i = 0; i < p.count; i++) {
    v.fromBufferAttribute(p, i);
    const t = (v.x - root) / (tip - root);
    const pitch = (24 - 14 * Math.max(0, Math.min(1, t))) * (Math.PI / 180) * dir;
    const y = v.y, z = v.z;
    v.y = y * Math.cos(pitch) - z * Math.sin(pitch);
    v.z = y * Math.sin(pitch) + z * Math.cos(pitch);
    p.setXYZ(i, v.x, v.y, v.z);
  }
  g.computeVertexNormals();
  return g;
}

export function createF450() {
  const root = new THREE.Group();
  root.name = 'F450';
  const model = new THREE.Group();
  root.add(model);

  // ---------------------------------------------------------------- materials
  const armRed = mat({ color: 0xc41d1d, roughness: 0.38, clearcoat: 0.6, clearcoatRoughness: 0.25 });
  const armWhite = mat({ color: 0xeeeeea, roughness: 0.4, clearcoat: 0.6, clearcoatRoughness: 0.25 });
  const pcb = mat({ map: pcbTexture(), roughness: 0.45, metalness: 0.1, clearcoat: 0.7 });
  const topPlate = mat({ color: 0x1d1f22, roughness: 0.55, metalness: 0.05, clearcoat: 0.4 });
  const motorBell = mat({ color: 0xb8bcc2, roughness: 0.22, metalness: 1.0 });
  const motorDark = mat({ color: 0x1a1b1e, roughness: 0.35, metalness: 0.8 });
  const motorAccent = mat({ color: 0xc0392b, roughness: 0.3, metalness: 0.8 });
  const copper = mat({ color: 0xb87333, roughness: 0.35, metalness: 1 });
  const propBlack = mat({ color: 0x151618, roughness: 0.35, clearcoat: 0.5, side: THREE.DoubleSide, transparent: true });
  const propOrange = mat({ color: 0xff6a13, roughness: 0.35, clearcoat: 0.5, side: THREE.DoubleSide, transparent: true });
  const carbon = mat({ map: carbonTexture(), roughness: 0.3, metalness: 0.2, clearcoat: 1, clearcoatRoughness: 0.1 });
  const rubber = mat({ color: 0x0e0e0f, roughness: 0.9 });
  const escBlue = mat({ color: 0x1f5fbf, roughness: 0.3, clearcoat: 0.8 });
  const fcMat = mat({ map: fcLabelTexture(), roughness: 0.45, clearcoat: 0.3 });
  const fcSide = mat({ color: 0xdfe2e5, roughness: 0.5 });
  const batMat = mat({ map: batteryLabelTexture(), roughness: 0.5, clearcoat: 0.4 });
  const batSide = mat({ color: 0x202328, roughness: 0.6 });
  const strap = mat({ color: 0x26282c, roughness: 0.95 });
  const gpsMat = mat({ color: 0x111214, roughness: 0.35, clearcoat: 0.8 });
  const redWire = mat({ color: 0xd11a1a, roughness: 0.5 });
  const blackWire = mat({ color: 0x0b0b0b, roughness: 0.5 });
  const shadowCast = (o) => o.traverse((c) => { if (c.isMesh) { c.castShadow = true; c.receiveShadow = true; } });

  // ---------------------------------------------------------------- centre plates
  const plateGeo = (size, chamfer, t) => {
    const g = new THREE.ExtrudeGeometry(plateShape(size, chamfer), { depth: t, bevelEnabled: false, curveSegments: 4 });
    g.rotateX(-Math.PI / 2);
    return g;
  };
  // bottom power distribution board (PCB)
  const bottom = new THREE.Mesh(plateGeo(0.118, 0.05, 0.0016), [pcb, pcb]);
  bottom.position.y = -0.0145;
  // map PCB texture to plate top: set UVs from XZ
  {
    const g = bottom.geometry, p = g.attributes.position, uv = g.attributes.uv;
    for (let i = 0; i < p.count; i++) uv.setXY(i, p.getX(i) / 0.118 + 0.5, p.getZ(i) / 0.118 + 0.5);
  }
  model.add(bottom);
  const top = new THREE.Mesh(plateGeo(0.105, 0.052, 0.0018), topPlate);
  top.position.y = 0.0135;
  model.add(top);
  // plate cut-outs (decorative dark recesses)
  for (const [x, z] of [[0, 0.036], [0, -0.036], [0.036, 0], [-0.036, 0]]) {
    const hole = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.0005, 20), mat({ color: 0x050505, roughness: 1 }));
    hole.position.set(x, 0.0155, z);
    model.add(hole);
  }

  // ---------------------------------------------------------------- arms, motors, props
  const props = [];
  const armGeo = taperedArm(ARM - 0.042, 0.027, 0.012, 0.034, 0.02);
  for (let i = 0; i < 4; i++) {
    const [fx, fy] = MOTOR_FRD[i];
    const X = fy * ARM, Z = -fx * ARM;
    const ang = Math.atan2(-Z, X);
    const front = fx > 0;

    const arm = new THREE.Mesh(armGeo, front ? armRed : armWhite);
    arm.position.set(Math.cos(ang) * 0.03, 0, -Math.sin(ang) * 0.03);
    arm.rotation.y = ang;
    model.add(arm);

    // under-arm ribs (moulded channel look)
    const rib = new THREE.Mesh(new THREE.BoxGeometry(ARM - 0.08, 0.004, 0.004), front ? armRed : armWhite);
    rib.position.set(Math.cos(ang) * (ARM / 2 + 0.01), -0.012, -Math.sin(ang) * (ARM / 2 + 0.01));
    rib.rotation.y = ang;
    model.add(rib);

    // ESC taped under the arm
    const esc = new THREE.Mesh(new RoundedBoxGeometry(0.046, 0.009, 0.022, 2, 0.003), escBlue);
    esc.position.set(Math.cos(ang) * 0.1, -0.018, -Math.sin(ang) * 0.1);
    esc.rotation.y = ang;
    model.add(esc);

    // motor mount pad
    const pad = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.021, 0.012, 28), front ? armRed : armWhite);
    pad.position.set(X, 0, Z);
    model.add(pad);

    const motor = new THREE.Group();
    motor.position.set(X, 0.006, Z);
    model.add(motor);
    // stator base + windings visible through the gap
    const base = new THREE.Mesh(new THREE.CylinderGeometry(0.0142, 0.0148, 0.006, 32), motorDark);
    base.position.y = 0.003;
    motor.add(base);
    const winding = new THREE.Mesh(new THREE.CylinderGeometry(0.0125, 0.0125, 0.004, 24), copper);
    winding.position.y = 0.008;
    motor.add(winding);
    const bell = new THREE.Mesh(new THREE.CylinderGeometry(0.0138, 0.0142, 0.017, 36, 1, true), motorBell);
    bell.position.y = 0.0185;
    motor.add(bell);
    const cap = new THREE.Mesh(new THREE.CylinderGeometry(0.0118, 0.0138, 0.003, 36), motorDark);
    cap.position.y = 0.0285;
    motor.add(cap);
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.0139, 0.0009, 8, 36), motorAccent);
    ring.rotation.x = Math.PI / 2;
    ring.position.y = 0.022;
    motor.add(ring);
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.0025, 0.0025, 0.014, 12), motorBell);
    shaft.position.y = 0.036;
    motor.add(shaft);

    // LED under the motor: left red, right green (navigation lights), rear white
    const ledColor = !front ? 0xffffff : fy < 0 ? 0xff2020 : 0x20ff40;
    const led = new THREE.Mesh(new THREE.SphereGeometry(0.0045, 12, 8), new THREE.MeshBasicMaterial({ color: ledColor }));
    led.position.set(X * 1.02, -0.009, Z * 1.02);
    model.add(led);

    // propeller
    const dir = MOTOR_DIR[i];
    const prop = new THREE.Group();
    prop.position.set(X, 0.045, Z);
    const bladeMat = front ? propOrange : propBlack;
    const bGeo = bladeGeometry(dir);
    const b1 = new THREE.Mesh(bGeo, bladeMat);
    const b2 = new THREE.Mesh(bGeo, bladeMat);
    b2.rotation.y = Math.PI;
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.0085, 0.0095, 0.008, 24), bladeMat);
    const nut = new THREE.Mesh(new THREE.ConeGeometry(0.0055, 0.012, 16), motorBell);
    nut.position.y = 0.009;
    const blades = new THREE.Group();
    blades.add(b1, b2, hub);
    prop.add(blades, nut);
    const blur = new THREE.Mesh(
      new THREE.CircleGeometry(PROP_R, 48),
      new THREE.MeshBasicMaterial({
        map: propBlurTexture(front ? [255, 106, 19] : [25, 25, 28]), transparent: true, opacity: 0, depthWrite: false, side: THREE.DoubleSide,
      }),
    );
    blur.rotation.x = -Math.PI / 2;
    blur.renderOrder = 2;
    prop.add(blur);
    model.add(prop);
    props.push({ group: blades, blur, bladeMat, dir });
  }

  // ---------------------------------------------------------------- flight controller & avionics
  const fcMats = [fcSide, fcSide, fcMat, fcSide, fcSide, fcSide];
  const fc = new THREE.Mesh(new RoundedBoxGeometry(0.05, 0.0155, 0.0815, 3, 0.004), fcMats);
  fc.position.set(0, 0.0145 + 0.008 + 0.004, 0.004);
  model.add(fc);
  // fix FC label UVs on top face so the arrow points forward
  const damp = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.004, 0.07), rubber);
  damp.position.set(0, 0.0175, 0.004);
  model.add(damp);
  // safety switch + buzzer
  const sw = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.008, 16), mat({ color: 0xd62c2c, emissive: 0x550000, roughness: 0.4 }));
  sw.position.set(0.038, 0.02, 0.03);
  model.add(sw);
  const buzzer = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.009, 16), rubber);
  buzzer.position.set(-0.038, 0.02, 0.03);
  model.add(buzzer);
  // RC receiver with antennas
  const rx = new THREE.Mesh(new RoundedBoxGeometry(0.03, 0.012, 0.022, 2, 0.002), mat({ color: 0x2d2f33, roughness: 0.5 }));
  rx.position.set(0, 0.021, 0.05);
  model.add(rx);
  for (const s of [-1, 1]) {
    const ant = new THREE.Mesh(new THREE.CylinderGeometry(0.0008, 0.0008, 0.09, 6), rubber);
    ant.position.set(s * 0.03, 0.035, 0.07);
    ant.rotation.z = s * -0.9;
    ant.rotation.x = 0.4;
    model.add(ant);
    const tipM = new THREE.Mesh(new THREE.CylinderGeometry(0.0012, 0.0012, 0.03, 6), mat({ color: 0xe8e8e8 }));
    tipM.position.set(s * 0.066, 0.061, 0.085);
    tipM.rotation.copy(ant.rotation);
    model.add(tipM);
  }
  // GPS mast (folding)
  const mastBase = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.008, 0.014), mat({ color: 0x9aa0a6, metalness: 0.8, roughness: 0.3 }));
  mastBase.position.set(0, 0.019, 0.042);
  model.add(mastBase);
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.0035, 0.0035, 0.1, 12), carbon);
  mast.position.set(0, 0.07, 0.042);
  model.add(mast);
  const gps = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.032, 0.013, 40), gpsMat);
  gps.position.set(0, 0.125, 0.042);
  model.add(gps);
  const arrow = new THREE.Mesh(new THREE.ConeGeometry(0.009, 0.02, 3), mat({ color: 0xeeeeee, roughness: 0.4 }));
  arrow.rotation.x = -Math.PI / 2;
  arrow.rotation.y = Math.PI;
  arrow.scale.y = 1;
  arrow.position.set(0, 0.1318, 0.036);
  arrow.scale.set(1, 1, 0.12);
  model.add(arrow);
  const gpsLed = new THREE.Mesh(new THREE.SphereGeometry(0.0035, 10, 8), new THREE.MeshBasicMaterial({ color: 0x3aa0ff }));
  gpsLed.position.set(0, 0.132, 0.058);
  model.add(gpsLed);

  // FPV camera at the front
  const cam = new THREE.Mesh(new RoundedBoxGeometry(0.02, 0.02, 0.02, 2, 0.003), mat({ color: 0x1b1c1f, roughness: 0.5 }));
  cam.position.set(0, -0.004, -0.066);
  model.add(cam);
  const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.0065, 0.0075, 0.01, 20), mat({ color: 0x050608, roughness: 0.05, metalness: 0.5, clearcoat: 1 }));
  lens.rotation.x = Math.PI / 2;
  lens.position.set(0, -0.004, -0.079);
  model.add(lens);

  // ---------------------------------------------------------------- battery
  const bat = new THREE.Mesh(new RoundedBoxGeometry(0.034, 0.025, 0.105, 3, 0.004), [batSide, batSide, batMat, batSide, batSide, batSide]);
  bat.position.set(0, -0.0145 - 0.0145, 0.002);
  model.add(bat);
  for (const z of [-0.03, 0.03]) {
    const st = new THREE.Mesh(new THREE.BoxGeometry(0.037, 0.028, 0.012), strap);
    st.position.set(0, bat.position.y, z);
    model.add(st);
  }
  // power leads (XT60)
  const lead = (pts, m) => {
    const curve = new THREE.CatmullRomCurve3(pts.map((p) => new THREE.Vector3(...p)));
    model.add(new THREE.Mesh(new THREE.TubeGeometry(curve, 24, 0.0018, 6, false), m));
  };
  lead([[0.004, -0.028, 0.055], [0.006, -0.03, 0.075], [0.01, -0.02, 0.07], [0.012, -0.012, 0.05]], redWire);
  lead([[-0.004, -0.028, 0.055], [-0.006, -0.032, 0.078], [-0.01, -0.021, 0.072], [-0.012, -0.012, 0.05]], blackWire);
  const xt60 = new THREE.Mesh(new THREE.BoxGeometry(0.016, 0.008, 0.012), mat({ color: 0xf2c200, roughness: 0.4 }));
  xt60.position.set(0, -0.031, 0.075);
  model.add(xt60);

  // ---------------------------------------------------------------- landing gear (tall skids)
  const gearY = -GEAR_HEIGHT;
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const from = new THREE.Vector3(sx * 0.035, -0.016, sz * 0.045);
      const to = new THREE.Vector3(sx * 0.095, gearY + 0.008, sz * 0.055);
      const len = from.distanceTo(to);
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.005, len, 12), carbon);
      leg.position.copy(from).add(to).multiplyScalar(0.5);
      leg.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), to.clone().sub(from).normalize());
      model.add(leg);
    }
    const skid = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.22, 16), carbon);
    skid.rotation.x = Math.PI / 2;
    skid.position.set(sx * 0.095, gearY + 0.005, 0);
    model.add(skid);
    for (const sz of [-1, 1]) {
      const capM = new THREE.Mesh(new THREE.SphereGeometry(0.0065, 12, 8), rubber);
      capM.position.set(sx * 0.095, gearY + 0.005, sz * 0.11);
      model.add(capM);
    }
    const foam = new THREE.Mesh(new THREE.CylinderGeometry(0.0072, 0.0072, 0.06, 16), mat({ color: 0x1c1c1c, roughness: 0.95 }));
    foam.rotation.x = Math.PI / 2;
    foam.position.set(sx * 0.095, gearY + 0.005, 0);
    model.add(foam);
  }

  shadowCast(model);
  props.forEach((p) => { p.blur.castShadow = false; });

  // ---------------------------------------------------------------- per-frame animation
  const update = (motorOut, propAngles, armed) => {
    for (let i = 0; i < 4; i++) {
      const p = props[i];
      const spin = motorOut[i];
      p.group.rotation.y = propAngles[i] * -1; // FRD +z (down) rotation == three -Y rotation
      const blur = Math.min(1, Math.max(0, (spin - 0.04) * 3.2));
      p.blur.material.opacity = blur * 0.85;
      p.bladeMat.opacity = 1 - blur * 0.75;
      p.blur.rotation.z = propAngles[i];
    }
    gpsLed.material.color.setHex(armed ? 0x20ff60 : (Math.floor(performance.now() / 500) % 2 ? 0x3aa0ff : 0x0a2a55));
  };

  return { group: root, update, gearHeight: GEAR_HEIGHT };
}
