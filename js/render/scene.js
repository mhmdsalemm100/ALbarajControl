// Renderer, cameras and the glue that places the vehicle in the world.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { World } from './world.js';
import { createF450, GEAR_HEIGHT } from './f450.js';

export const QUALITY = {
  low: { pixelRatio: 0.75, shadowMap: 0, terrainSegments: 128, textureSize: 256, trees: 350, buildings: 16, clouds: 12, antialias: false },
  medium: { pixelRatio: 1, shadowMap: 1024, terrainSegments: 200, textureSize: 512, trees: 900, buildings: 32, clouds: 24, antialias: true },
  high: { pixelRatio: 1.5, shadowMap: 2048, terrainSegments: 280, textureSize: 512, trees: 1600, buildings: 40, clouds: 32, antialias: true },
  ultra: { pixelRatio: 2, shadowMap: 4096, terrainSegments: 360, textureSize: 1024, trees: 2600, buildings: 48, clouds: 40, antialias: true },
};

export const CAMERA_MODES = ['chase', 'orbit', 'fpv', 'pilot', 'top'];
const CAMERA_LABEL = { chase: 'Chase', orbit: 'Orbit', fpv: 'FPV', pilot: 'Pilot (line of sight)', top: 'Top-down' };

// NED (north, east, down) -> three (x=east, y=up, z=south)
export const nedToThree = (n) => new THREE.Vector3(n[1], -n[2], -n[0]);
// rotation that maps NED/FRD vectors into three.js world/body axes
const M = new THREE.Matrix4().set(0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1);
const qM = new THREE.Quaternion().setFromRotationMatrix(M);
const qMi = qM.clone().invert();
export function quatNedToThree(q) {
  const qn = new THREE.Quaternion(q[1], q[2], q[3], q[0]);
  return qM.clone().multiply(qn).multiply(qMi);
}

export class SceneView {
  constructor(container, qualityName = 'high') {
    this.container = container;
    this.qualityName = qualityName;
    const Q = QUALITY[qualityName];
    this.renderer = new THREE.WebGLRenderer({ antialias: Q.antialias, powerPreference: 'high-performance', preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, Q.pixelRatio));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.shadowMap.enabled = Q.shadowMap > 0;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(60, 1, 0.05, 12000);
    this.camera.position.set(0, 0.8, 1.8);
    this.world = new World(this.scene, this.renderer, Q);

    this.drone = createF450();
    this.scene.add(this.drone.group);

    // setpoint / waypoint markers
    this.markers = new THREE.Group();
    this.scene.add(this.markers);
    this.pathLine = null;

    // trail
    this.trailMax = 1500;
    this.trailPos = new Float32Array(this.trailMax * 3);
    const tg = new THREE.BufferGeometry();
    tg.setAttribute('position', new THREE.BufferAttribute(this.trailPos, 3));
    tg.setDrawRange(0, 0);
    this.trail = new THREE.Line(tg, new THREE.LineBasicMaterial({ color: 0xff6a13, transparent: true, opacity: 0.8 }));
    this.trail.frustumCulled = false;
    this.trailCount = 0;
    this.showTrail = true;
    this.scene.add(this.trail);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.enabled = false;
    this.controls.minDistance = 0.6;
    this.controls.maxDistance = 400;

    this.cameraMode = 'chase';
    this.chase = { dist: 1.7, height: 0.55, yaw: 0 };
    this.pilotPos = new THREE.Vector3(-9, 1.7, 7.2);
    this.dronePos = new THREE.Vector3();
    this.droneQuat = new THREE.Quaternion();
    this.headingYaw = 0;
    this.fov = 60;

    this.renderer.domElement.addEventListener('wheel', (e) => {
      if (this.cameraMode === 'chase') {
        this.chase.dist = THREE.MathUtils.clamp(this.chase.dist * (e.deltaY > 0 ? 1.1 : 0.9), 0.8, 60);
      }
      if (this.cameraMode === 'pilot' || this.cameraMode === 'fpv') {
        this.fov = THREE.MathUtils.clamp(this.fov * (e.deltaY > 0 ? 1.08 : 0.92), 10, 100);
      }
    }, { passive: true });

    this.resize();
    new ResizeObserver(() => this.resize()).observe(container);
  }

  get cameraLabel() { return CAMERA_LABEL[this.cameraMode]; }

  setCameraMode(mode) {
    this.cameraMode = mode;
    this.controls.enabled = mode === 'orbit';
    if (mode === 'orbit') {
      this.controls.target.copy(this.dronePos);
      this.camera.position.copy(this.dronePos).add(new THREE.Vector3(2.5, 1.2, 2.5));
    }
    this.fov = mode === 'fpv' ? 90 : mode === 'pilot' ? 50 : 60;
  }

  nextCamera() {
    const i = CAMERA_MODES.indexOf(this.cameraMode);
    this.setCameraMode(CAMERA_MODES[(i + 1) % CAMERA_MODES.length]);
    return this.cameraMode;
  }

  resize() {
    const w = this.container.clientWidth || 1, h = this.container.clientHeight || 1;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  terrainHeightNed(n, e) {
    return this.world.heightAt(e, -n);
  }

  /** Called every frame with the true vehicle state. */
  updateVehicle(state, dt) {
    this.dronePos.copy(nedToThree(state.pos));
    this.droneQuat.copy(quatNedToThree(state.q));
    const g = this.drone.group;
    g.position.copy(this.dronePos);
    g.quaternion.copy(this.droneQuat);
    this.drone.update(state.motorOut, state.propAngle, state.armed);
    this.world.followShadow(this.dronePos);
    this.world.update(dt, state.wind || [0, 0, 0]);

    // trail
    if (this.showTrail && state.armed) {
      const last = this.trailCount ? new THREE.Vector3().fromArray(this.trailPos, ((this.trailCount - 1) % this.trailMax) * 3) : null;
      if (!last || last.distanceTo(this.dronePos) > 0.4) {
        if (this.trailCount >= this.trailMax) {
          this.trailPos.copyWithin(0, 3);
          this.trailCount = this.trailMax - 1;
        }
        this.dronePos.toArray(this.trailPos, this.trailCount * 3);
        this.trailCount++;
        this.trail.geometry.setDrawRange(0, this.trailCount);
        this.trail.geometry.attributes.position.needsUpdate = true;
      }
    }
    this.trail.visible = this.showTrail;
    this.updateCamera(dt);
  }

  clearTrail() {
    this.trailCount = 0;
    this.trail.geometry.setDrawRange(0, 0);
  }

  updateCamera(dt) {
    const cam = this.camera;
    const p = this.dronePos;
    // heading (yaw only) of the drone in three coords
    const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(this.droneQuat);
    const yaw = Math.atan2(-fwd.x, -fwd.z);
    const k = 1 - Math.exp(-dt * 3);
    this.headingYaw += Math.atan2(Math.sin(yaw - this.headingYaw), Math.cos(yaw - this.headingYaw)) * k;
    switch (this.cameraMode) {
      case 'chase': {
        const hy = this.headingYaw;
        const target = new THREE.Vector3(
          p.x + Math.sin(hy) * this.chase.dist,
          p.y + this.chase.height * (this.chase.dist / 1.7),
          p.z + Math.cos(hy) * this.chase.dist,
        );
        const groundY = this.world.heightAt(target.x, target.z) + 0.3;
        if (target.y < groundY) target.y = groundY;
        // position is rigidly attached (heading is already smoothed) so the view
        // stays framed at any frame rate
        cam.position.lerp(target, Math.min(1, 1 - Math.exp(-dt * 25)));
        cam.lookAt(p.x, p.y + 0.15, p.z);
        break;
      }
      case 'orbit':
        this.controls.target.lerp(p, 1 - Math.exp(-dt * 10));
        this.controls.update();
        break;
      case 'fpv': {
        const off = new THREE.Vector3(0, -0.004, -0.09).applyQuaternion(this.droneQuat);
        cam.position.copy(p).add(off);
        // 25 deg camera up-tilt, like a real FPV setup
        const q = this.droneQuat.clone().multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), THREE.MathUtils.degToRad(20)));
        cam.quaternion.copy(q);
        break;
      }
      case 'pilot':
        cam.position.copy(this.pilotPos);
        cam.lookAt(p);
        break;
      case 'top':
        cam.position.lerp(new THREE.Vector3(p.x, p.y + 40, p.z + 0.01), 1 - Math.exp(-dt * 5));
        cam.lookAt(p);
        break;
    }
    if (Math.abs(cam.fov - this.fov) > 0.01) { cam.fov = this.fov; cam.updateProjectionMatrix(); }
  }

  /** Show mission waypoints as 3D markers. items: [{ned:[n,e,d], label}] */
  setMissionMarkers(points) {
    this.markers.clear();
    if (!points.length) return;
    const pts = [];
    points.forEach((pt, i) => {
      const v = nedToThree(pt.ned);
      pts.push(v);
      const pole = new THREE.Mesh(
        new THREE.CylinderGeometry(0.04, 0.04, Math.max(0.1, v.y - this.world.heightAt(v.x, v.z))),
        new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.5 }),
      );
      const groundY = this.world.heightAt(v.x, v.z);
      pole.position.set(v.x, (v.y + groundY) / 2, v.z);
      const ball = new THREE.Mesh(new THREE.SphereGeometry(0.3, 20, 12), new THREE.MeshBasicMaterial({ color: pt.color || 0x2e9df7, transparent: true, opacity: 0.85 }));
      ball.position.copy(v);
      const sprite = makeLabel(String(pt.label ?? i + 1));
      sprite.position.copy(v).add(new THREE.Vector3(0, 0.8, 0));
      this.markers.add(pole, ball, sprite);
    });
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineDashedMaterial({ color: 0xffe14d, dashSize: 1.5, gapSize: 0.8 }));
    line.computeLineDistances();
    this.markers.add(line);
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }

  screenshot() {
    this.render();
    return this.renderer.domElement.toDataURL('image/png');
  }
}

function makeLabel(text) {
  const c = document.createElement('canvas');
  c.width = 128; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(20,24,30,0.85)';
  g.beginPath();
  g.roundRect(4, 4, 120, 56, 14);
  g.fill();
  g.fillStyle = '#fff';
  g.font = 'bold 36px Arial';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(text, 64, 34);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: t, depthTest: false }));
  s.scale.set(1.1, 0.55, 1);
  return s;
}

export { GEAR_HEIGHT };
