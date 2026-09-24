// The 3D environment: sky, sun, terrain, airfield, trees, buildings, lake,
// windsock and clouds. Everything is procedural (no downloads needed).
//
// World axes: three.js X = East, Y = Up, Z = South (= -North).

import * as THREE from 'three';
import { Sky } from 'three/addons/objects/Sky.js';
import {
  makeNoise, mulberry32, grassTextures, sandTextures, asphaltTextures, runwayTexture, helipadTexture,
  facadeTexture, hangarTexture, cloudTexture, barkTexture,
} from './textures.js';

const TERRAIN_SIZE = 4000;
const LAKE = { x: -380, z: 260, r: 150 };
const RUNWAY = { x: 90, z: 0, len: 420, w: 28 };
const APRON = { x: 150, z: 60, w: 70, d: 90 };

const smooth = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

export class World {
  constructor(scene, renderer, quality) {
    this.scene = scene;
    this.renderer = renderer;
    this.quality = quality;
    this.noise = makeNoise(42);
    this.wind = { dir: 45, speed: 0 };
    this.time = 0;

    this.buildSky();
    this.buildLights();
    this.buildTerrain();
    this.buildAirfield();
    this.buildLake();
    this.buildTrees();
    this.buildBuildings();
    this.buildWindsock();
    this.buildClouds();
    this.setTimeOfDay(10.5);
  }

  // ------------------------------------------------------------------ height field
  heightAt(x, z) {
    const n = this.noise;
    const d = Math.hypot(x, z);
    const hills = (n.fbm(x / 520 + 100, z / 520 + 100, 5, 1e6) - 0.45) * 70;
    const small = (n.fbm(x / 90, z / 90, 3, 1e6) - 0.5) * 4;
    const ridge = Math.pow(1 - Math.abs(n.noise2(x / 380 + 7, z / 380 + 3, 1e6) * 2 - 1), 3);
    const mountains = smooth(900, 1900, d) * (120 + ridge * 260);
    let h = smooth(260, 650, d) * (hills + small) + mountains + small * smooth(120, 260, d);
    // airfield & helipad flat zones
    const inAirfield = Math.max(
      1 - smooth(0, 60, Math.max(Math.abs(x - RUNWAY.x) - RUNWAY.w, Math.abs(z - RUNWAY.z) - RUNWAY.len / 2)),
      1 - smooth(0, 40, d - 45),
      1 - smooth(0, 40, Math.max(Math.abs(x - APRON.x) - APRON.w / 2 - 50, Math.abs(z - APRON.z) - APRON.d / 2 - 20)),
    );
    h *= 1 - inAirfield;
    // lake basin
    const ld = Math.hypot(x - LAKE.x, z - LAKE.z);
    if (ld < LAKE.r * 1.6) h = h * smooth(LAKE.r * 0.9, LAKE.r * 1.6, ld) - (1 - smooth(LAKE.r * 0.6, LAKE.r * 1.05, ld)) * 6;
    return h;
  }

  // ------------------------------------------------------------------ sky & lights
  buildSky() {
    this.sky = new Sky();
    this.sky.scale.setScalar(20000);
    const u = this.sky.material.uniforms;
    u.turbidity.value = 4.5;
    u.rayleigh.value = 1.2;
    u.mieCoefficient.value = 0.004;
    u.mieDirectionalG.value = 0.82;
    this.scene.add(this.sky);
    this.sun = new THREE.Vector3();
    this.pmrem = new THREE.PMREMGenerator(this.renderer);
    this.envScene = new THREE.Scene();
    this.envSky = new Sky();
    this.envSky.scale.setScalar(1000);
    this.envScene.add(this.envSky);
    this.scene.fog = new THREE.FogExp2(0xc4d3e2, 0.00028);
  }

  buildLights() {
    this.hemi = new THREE.HemisphereLight(0xcfe3ff, 0x5b6b3a, 0.55);
    this.scene.add(this.hemi);
    this.sunLight = new THREE.DirectionalLight(0xfff1dc, 3.0);
    const sm = this.quality.shadowMap;
    this.sunLight.castShadow = sm > 0;
    this.sunLight.shadow.mapSize.set(sm || 1024, sm || 1024);
    const c = this.sunLight.shadow.camera;
    c.left = c.bottom = -35; c.right = c.top = 35; c.near = 1; c.far = 900;
    this.sunLight.shadow.bias = -0.0002;
    this.sunLight.shadow.normalBias = 0.02;
    this.scene.add(this.sunLight, this.sunLight.target);
  }

  setTimeOfDay(hour) {
    this.hour = hour;
    // simple solar path: sunrise 6, sunset 18.5
    const t = (hour - 6) / 12.5;
    const elev = Math.max(-6, Math.sin(Math.PI * t) * 72);
    const azim = 90 + t * 180; // east -> south -> west
    const phi = THREE.MathUtils.degToRad(90 - elev);
    const theta = THREE.MathUtils.degToRad(azim);
    // three: X=east, Z=south; azimuth measured from north clockwise
    this.sun.set(Math.sin(phi) * Math.sin(theta), Math.cos(phi), -Math.sin(phi) * Math.cos(theta));
    this.sky.material.uniforms.sunPosition.value.copy(this.sun);
    this.envSky.material.uniforms.sunPosition.value.copy(this.sun);
    for (const k of ['turbidity', 'rayleigh', 'mieCoefficient', 'mieDirectionalG']) this.envSky.material.uniforms[k].value = this.sky.material.uniforms[k].value;
    const day = THREE.MathUtils.clamp(elev / 25, 0, 1);
    const warm = 1 - THREE.MathUtils.clamp(elev / 30, 0, 1);
    this.sunLight.intensity = 0.2 + 2.9 * day;
    this.sunLight.color.setRGB(1, 0.93 - warm * 0.25, 0.84 - warm * 0.45);
    this.hemi.intensity = 0.15 + 0.5 * day;
    this.scene.fog.color.setRGB(0.55 + 0.22 * day - warm * 0.05, 0.62 + 0.2 * day - warm * 0.12, 0.7 + 0.19 * day - warm * 0.2);
    if (this.envRT) this.envRT.dispose();
    this.envRT = this.pmrem.fromScene(this.envScene, 0.02);
    this.scene.environment = this.envRT.texture;
    this.scene.environmentIntensity = 0.35 + 0.5 * day;
    this.renderer.toneMappingExposure = 0.55 + 0.35 * (1 - day * 0.5);
  }

  followShadow(target) {
    // keep a tight shadow frustum around the vehicle for crisp shadows
    this.sunLight.position.copy(target).addScaledVector(this.sun, 400);
    this.sunLight.target.position.copy(target);
  }

  // ------------------------------------------------------------------ terrain
  buildTerrain() {
    const seg = this.quality.terrainSegments;
    const geo = new THREE.PlaneGeometry(TERRAIN_SIZE, TERRAIN_SIZE, seg, seg);
    geo.rotateX(-Math.PI / 2);
    const pos = geo.attributes.position;
    const blend = new Float32Array(pos.count);
    const colors = new Float32Array(pos.count * 3);
    const n = this.noise;
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i), z = pos.getZ(i);
      const h = this.heightAt(x, z);
      pos.setY(i, h);
      const d = Math.hypot(x, z);
      const macro = n.fbm(x / 140, z / 140, 4, 1e6);
      // grass near the airfield, dry/sand further out and on mountains
      let b = smooth(500, 1300, d + (macro - 0.5) * 500) + smooth(40, 120, h) * 0.8;
      const ld = Math.hypot(x - LAKE.x, z - LAKE.z);
      b = Math.max(b, 1 - smooth(LAKE.r * 0.95, LAKE.r * 1.15, ld)); // sandy shore
      blend[i] = Math.min(1, b);
      const tint = 0.82 + macro * 0.36;
      colors[i * 3] = tint * (1 + (macro - 0.5) * 0.15);
      colors[i * 3 + 1] = tint;
      colors[i * 3 + 2] = tint * 0.95;
    }
    geo.setAttribute('blend', new THREE.BufferAttribute(blend, 1));
    geo.computeVertexNormals();
    // darker, greyer rock on steep slopes
    const nrm = geo.attributes.normal;
    for (let i = 0; i < pos.count; i++) {
      const rock = smooth(0.12, 0.4, 1 - nrm.getY(i));
      colors[i * 3] *= 1 - rock * 0.42;
      colors[i * 3 + 1] *= 1 - rock * 0.46;
      colors[i * 3 + 2] *= 1 - rock * 0.44;
    }
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    // UVs in metres / tile size
    const uv = geo.attributes.uv;
    for (let i = 0; i < uv.count; i++) uv.setXY(i, pos.getX(i) / 6, pos.getZ(i) / 6);

    const grass = grassTextures(this.quality.textureSize);
    const sand = sandTextures(this.quality.textureSize);
    const aniso = this.renderer.capabilities.getMaxAnisotropy();
    [grass.map, grass.normalMap, sand.map, sand.normalMap].forEach((t) => { t.anisotropy = aniso; t.repeat.set(1, 1); });
    const mat = new THREE.MeshStandardMaterial({
      map: grass.map, normalMap: grass.normalMap, normalScale: new THREE.Vector2(0.6, 0.6), vertexColors: true, roughness: 0.95,
    });
    mat.onBeforeCompile = (sh) => {
      sh.uniforms.sandMap = { value: sand.map };
      sh.vertexShader = sh.vertexShader
        .replace('#include <common>', '#include <common>\nattribute float blend;\nvarying float vBlend;\nvarying vec3 vWorldP;')
        .replace('#include <begin_vertex>', '#include <begin_vertex>\nvBlend = blend;\nvWorldP = (modelMatrix * vec4(position,1.0)).xyz;');
      sh.fragmentShader = sh.fragmentShader
        .replace('#include <common>', '#include <common>\nuniform sampler2D sandMap;\nvarying float vBlend;\nvarying vec3 vWorldP;')
        .replace('#include <map_fragment>', `
          vec4 gTex = texture2D(map, vMapUv);
          vec4 gTex2 = texture2D(map, vMapUv * 0.137 + 0.31);
          vec4 sTex = texture2D(sandMap, vMapUv * 0.5);
          vec4 sTex2 = texture2D(sandMap, vMapUv * 0.071);
          vec4 gCol = mix(gTex, gTex2, 0.45);
          vec4 sCol = mix(sTex, sTex2, 0.5);
          diffuseColor *= mix(gCol, sCol, smoothstep(0.0, 1.0, vBlend));
        `);
    };
    const mesh = new THREE.Mesh(geo, mat);
    mesh.receiveShadow = true;
    mesh.name = 'terrain';
    this.scene.add(mesh);
    this.terrain = mesh;
  }

  // ------------------------------------------------------------------ airfield
  buildAirfield() {
    const g = new THREE.Group();
    // runway
    const rw = new THREE.Mesh(
      new THREE.PlaneGeometry(RUNWAY.w, RUNWAY.len),
      new THREE.MeshStandardMaterial({ map: runwayTexture(), roughness: 0.85 }),
    );
    rw.rotation.x = -Math.PI / 2;
    rw.position.set(RUNWAY.x, 0.03, RUNWAY.z);
    rw.receiveShadow = true;
    g.add(rw);

    const asphalt = asphaltTextures(512);
    asphalt.map.repeat.set(8, 8);
    asphalt.normalMap.repeat.set(8, 8);
    const apronMat = new THREE.MeshStandardMaterial({ map: asphalt.map, normalMap: asphalt.normalMap, roughness: 0.9, color: 0xb8b8b8 });
    const apron = new THREE.Mesh(new THREE.PlaneGeometry(APRON.w, APRON.d), apronMat);
    apron.rotation.x = -Math.PI / 2;
    apron.position.set(APRON.x, 0.025, APRON.z);
    apron.receiveShadow = true;
    g.add(apron);
    const taxi = new THREE.Mesh(new THREE.PlaneGeometry(APRON.x - RUNWAY.x - APRON.w / 2 + 4, 14), apronMat);
    taxi.rotation.x = -Math.PI / 2;
    taxi.position.set((RUNWAY.x + RUNWAY.w / 2 + APRON.x - APRON.w / 2) / 2, 0.02, APRON.z);
    taxi.receiveShadow = true;
    g.add(taxi);

    // helipad (home / start position)
    const pad = new THREE.Mesh(new THREE.CylinderGeometry(6, 6.1, 0.12, 64), [
      new THREE.MeshStandardMaterial({ color: 0x77797c, roughness: 0.9 }),
      new THREE.MeshStandardMaterial({ map: helipadTexture(), roughness: 0.75 }),
      new THREE.MeshStandardMaterial({ color: 0x77797c }),
    ]);
    pad.position.y = -0.06 + 0.001;
    pad.receiveShadow = true;
    g.add(pad);
    // pad edge lights
    const lightMat = new THREE.MeshBasicMaterial({ color: 0xffd35a });
    for (let i = 0; i < 16; i++) {
      const a = (i / 16) * Math.PI * 2;
      const l = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.09, 0.12, 10), lightMat);
      l.position.set(Math.cos(a) * 6.3, 0.06, Math.sin(a) * 6.3);
      g.add(l);
    }
    // runway edge lights
    const rwLight = new THREE.InstancedMesh(new THREE.CylinderGeometry(0.1, 0.12, 0.3, 8), new THREE.MeshBasicMaterial({ color: 0xfff4c8 }), 60);
    const m = new THREE.Matrix4();
    let k = 0;
    for (let i = 0; i < 30; i++) {
      for (const s of [-1, 1]) {
        m.makeTranslation(RUNWAY.x + s * (RUNWAY.w / 2 + 1), 0.15, RUNWAY.z - RUNWAY.len / 2 + (i + 0.5) * (RUNWAY.len / 30));
        rwLight.setMatrixAt(k++, m);
      }
    }
    g.add(rwLight);
    // pilot station: table + ground-station laptop next to the pad
    const table = new THREE.Mesh(new THREE.BoxGeometry(1.4, 0.05, 0.7), new THREE.MeshStandardMaterial({ color: 0xdedede, roughness: 0.5 }));
    table.position.set(-9, 0.75, 6);
    table.castShadow = table.receiveShadow = true;
    g.add(table);
    for (const [dx, dz] of [[-0.62, -0.28], [0.62, -0.28], [-0.62, 0.28], [0.62, 0.28]]) {
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.75), new THREE.MeshStandardMaterial({ color: 0x333333, metalness: 0.7 }));
      leg.position.set(-9 + dx, 0.375, 6 + dz);
      leg.castShadow = true;
      g.add(leg);
    }
    const laptop = new THREE.Group();
    const lbase = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.02, 0.24), new THREE.MeshStandardMaterial({ color: 0x2b2d31, metalness: 0.6, roughness: 0.3 }));
    const shell = new THREE.MeshStandardMaterial({ color: 0x2b2d31 });
    const lscreen = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.22, 0.01), [shell, shell, shell, shell, new THREE.MeshBasicMaterial({ color: 0x3a6fd8 }), shell]);
    lscreen.position.set(0, 0.11, -0.12);
    lscreen.rotation.x = -0.25;
    laptop.add(lbase, lscreen);
    laptop.position.set(-9, 0.785, 6);
    laptop.rotation.y = Math.PI * 0.85;
    laptop.traverse((o) => { o.castShadow = true; });
    g.add(laptop);
    // traffic cones around the pad
    const coneMat = new THREE.MeshStandardMaterial({ color: 0xff6a13, roughness: 0.6 });
    for (let i = 0; i < 4; i++) {
      const a = (i / 4) * Math.PI * 2 + Math.PI / 4;
      const c = new THREE.Mesh(new THREE.ConeGeometry(0.18, 0.5, 16), coneMat);
      c.position.set(Math.cos(a) * 8.5, 0.25, Math.sin(a) * 8.5);
      c.castShadow = true;
      g.add(c);
    }
    this.scene.add(g);
  }

  buildLake() {
    const water = new THREE.Mesh(
      new THREE.CircleGeometry(LAKE.r * 1.12, 96),
      new THREE.MeshPhysicalMaterial({ color: 0x1d4a63, roughness: 0.04, metalness: 0.1, clearcoat: 1, transparent: true, opacity: 0.92 }),
    );
    water.rotation.x = -Math.PI / 2;
    water.position.set(LAKE.x, -1.2, LAKE.z);
    water.receiveShadow = true;
    this.water = water;
    this.scene.add(water);
  }

  // ------------------------------------------------------------------ trees
  buildTrees() {
    const count = this.quality.trees;
    const rnd = mulberry32(77);
    const trunkGeo = new THREE.CylinderGeometry(0.12, 0.22, 1, 7);
    trunkGeo.translate(0, 0.5, 0);
    const pineGeo = (() => {
      const parts = [];
      for (let i = 0; i < 3; i++) {
        const c = new THREE.ConeGeometry(1.5 - i * 0.35, 2.2 - i * 0.3, 9);
        c.translate(0, 1.6 + i * 1.15, 0);
        parts.push(c);
      }
      return mergeGeometries(parts);
    })();
    const leafGeo = (() => {
      const s = new THREE.IcosahedronGeometry(1.6, 2);
      const p = s.attributes.position;
      const nz = this.noise;
      for (let i = 0; i < p.count; i++) {
        const x = p.getX(i), y = p.getY(i), z = p.getZ(i);
        const f = 0.8 + nz.noise2(x * 2 + 5, z * 2 + y, 1e6) * 0.45;
        p.setXYZ(i, x * f, y * f * 0.85 + 2.6, z * f);
      }
      s.computeVertexNormals();
      return s;
    })();
    const bark = new THREE.MeshStandardMaterial({ map: barkTexture(), roughness: 0.95 });
    const pineMat = new THREE.MeshStandardMaterial({ color: 0x2f5a31, roughness: 0.85, flatShading: true });
    const leafMat = new THREE.MeshStandardMaterial({ color: 0x4d7f35, roughness: 0.8 });

    const trunks = new THREE.InstancedMesh(trunkGeo, bark, count);
    const pines = new THREE.InstancedMesh(pineGeo, pineMat, count);
    const leaves = new THREE.InstancedMesh(leafGeo, leafMat, count);
    let np = 0, nl = 0, nt = 0;
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(), p = new THREE.Vector3();
    const col = new THREE.Color();
    this.treeSpots = [];
    let guard = 0;
    while (nt < count && guard++ < count * 20) {
      // clusters of trees (forests) + scattered
      const a = rnd() * Math.PI * 2;
      const r = 90 + Math.pow(rnd(), 0.8) * 1250;
      const x = Math.cos(a) * r, z = Math.sin(a) * r;
      const forest = this.noise.fbm(x / 260, z / 260, 3, 1e6);
      if (forest < 0.48 && rnd() > 0.15) continue;
      if (Math.abs(x - RUNWAY.x) < RUNWAY.w + 40 && Math.abs(z - RUNWAY.z) < RUNWAY.len / 2 + 60) continue;
      if (Math.abs(x - APRON.x) < APRON.w / 2 + 30 && Math.abs(z - APRON.z) < APRON.d / 2 + 30) continue;
      if (Math.hypot(x - LAKE.x, z - LAKE.z) < LAKE.r * 1.2) continue;
      const h = this.heightAt(x, z);
      if (h > 150) continue;
      const height = 5 + rnd() * 9;
      const isPine = rnd() > 0.45;
      q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), rnd() * Math.PI * 2);
      p.set(x, h - 0.2, z);
      s.set(height / 8, (isPine ? height * 0.55 : height * 0.5), height / 8);
      m.compose(p, q, s);
      trunks.setMatrixAt(nt++, m);
      const sc = height / (isPine ? 5.2 : 5.5);
      s.set(sc * (0.85 + rnd() * 0.3), sc, sc * (0.85 + rnd() * 0.3));
      m.compose(p, q, s);
      col.setHSL(0.25 + rnd() * 0.08, 0.35 + rnd() * 0.25, 0.28 + rnd() * 0.16);
      if (isPine) { pines.setMatrixAt(np, m); pines.setColorAt(np++, col); } else { leaves.setMatrixAt(nl, m); leaves.setColorAt(nl++, col); }
      this.treeSpots.push([x, z, height]);
    }
    trunks.count = nt; pines.count = np; leaves.count = nl;
    for (const im of [trunks, pines, leaves]) { im.castShadow = true; im.receiveShadow = true; im.instanceMatrix.needsUpdate = true; if (im.instanceColor) im.instanceColor.needsUpdate = true; this.scene.add(im); }
  }

  // ------------------------------------------------------------------ buildings
  buildBuildings() {
    const g = new THREE.Group();
    // hangars on the apron
    const hangarMat = new THREE.MeshStandardMaterial({ map: hangarTexture(), roughness: 0.45, metalness: 0.6 });
    const wallMat = new THREE.MeshStandardMaterial({ color: 0xd9d5cc, roughness: 0.8 });
    const doorMat = new THREE.MeshStandardMaterial({ color: 0x55606b, roughness: 0.5, metalness: 0.5 });
    for (let i = 0; i < 2; i++) {
      const hz = APRON.z - 22 + i * 44;
      const hx = APRON.x + APRON.w / 2 + 14;
      const roof = new THREE.Mesh(new THREE.CylinderGeometry(12, 12, 30, 40, 1, true, 0, Math.PI), hangarMat);
      roof.rotation.set(0, 0, Math.PI / 2);
      const rg = new THREE.Group();
      rg.add(roof);
      const wallShape = new THREE.Shape();
      wallShape.absarc(0, 0, 12, 0, Math.PI, false);
      const wallGeo = new THREE.ShapeGeometry(wallShape, 24);
      const back = new THREE.Mesh(wallGeo, wallMat);
      back.position.x = 15;
      back.rotation.y = Math.PI / 2;
      const front = new THREE.Mesh(wallGeo, doorMat);
      front.position.x = -15;
      front.rotation.y = -Math.PI / 2;
      rg.add(back, front);
      rg.position.set(hx, 0, hz);
      rg.traverse((o) => { o.castShadow = true; o.receiveShadow = true; });
      g.add(rg);
    }
    // control tower
    const tower = new THREE.Group();
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(2.4, 3, 18, 20), new THREE.MeshStandardMaterial({ color: 0xe6e1d6, roughness: 0.7 }));
    shaft.position.y = 9;
    const cab = new THREE.Mesh(new THREE.CylinderGeometry(5, 4, 4, 8), new THREE.MeshPhysicalMaterial({ color: 0x264b63, roughness: 0.05, metalness: 0.4, clearcoat: 1 }));
    cab.position.y = 20;
    const roofT = new THREE.Mesh(new THREE.CylinderGeometry(5.6, 5.2, 0.8, 8), new THREE.MeshStandardMaterial({ color: 0x303338 }));
    roofT.position.y = 22.4;
    const beacon = new THREE.Mesh(new THREE.SphereGeometry(0.4, 12, 8), new THREE.MeshBasicMaterial({ color: 0xff3030 }));
    beacon.position.y = 23.5;
    this.beacon = beacon;
    tower.add(shaft, cab, roofT, beacon);
    tower.position.set(APRON.x + 10, 0, APRON.z + APRON.d / 2 + 22);
    tower.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });
    g.add(tower);

    // a small town to the north
    const rnd = mulberry32(99);
    const facades = [facadeTexture(1), facadeTexture(2, { base: '#b7b0a3', glass: '#324f63' }), facadeTexture(3, { base: '#8f9aa5', glass: '#1e3446', cols: 6, rows: 16 })];
    const facadeMats = facades.map((t) => new THREE.MeshStandardMaterial({ map: t, roughness: 0.6, metalness: 0.15 }));
    const roofMat = new THREE.MeshStandardMaterial({ color: 0x4a4d52, roughness: 0.9 });
    for (let i = 0; i < this.quality.buildings; i++) {
      const bx = -250 + (i % 8) * 55 + (rnd() - 0.5) * 12;
      const bz = -560 - Math.floor(i / 8) * 60 + (rnd() - 0.5) * 12;
      const w = 16 + rnd() * 18, d = 16 + rnd() * 18, h = 10 + Math.pow(rnd(), 2) * 70;
      const fm = facadeMats[Math.floor(rnd() * facadeMats.length)].clone();
      fm.map = fm.map.clone();
      fm.map.repeat.set(w / 20, h / 30);
      fm.map.needsUpdate = true;
      const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), [fm, fm, roofMat, roofMat, fm, fm]);
      const base = this.heightAt(bx, bz);
      b.position.set(bx, base + h / 2 - 0.5, bz);
      b.castShadow = true;
      b.receiveShadow = true;
      g.add(b);
    }
    this.scene.add(g);
  }

  buildWindsock() {
    const g = new THREE.Group();
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.06, 5, 10), new THREE.MeshStandardMaterial({ color: 0xdddddd, metalness: 0.7, roughness: 0.3 }));
    pole.position.y = 2.5;
    g.add(pole);
    const sock = new THREE.Group();
    const segs = 5;
    for (let i = 0; i < segs; i++) {
      const r0 = 0.3 - i * 0.035, r1 = 0.3 - (i + 1) * 0.035;
      const s = new THREE.Mesh(new THREE.CylinderGeometry(r1, r0, 0.36, 16, 1, true), new THREE.MeshStandardMaterial({ color: i % 2 ? 0xffffff : 0xff5a1f, side: THREE.DoubleSide, roughness: 0.8 }));
      s.rotation.z = Math.PI / 2;
      s.position.x = -0.18 - i * 0.36;
      sock.add(s);
    }
    sock.position.y = 4.9;
    g.add(sock);
    g.position.set(-12, 0, -10);
    g.traverse((o) => { if (o.isMesh) o.castShadow = true; });
    this.windsock = sock;
    this.scene.add(g);
  }

  buildClouds() {
    this.clouds = new THREE.Group();
    const rnd = mulberry32(5);
    const texs = [cloudTexture(1), cloudTexture(2), cloudTexture(3)];
    for (let i = 0; i < this.quality.clouds; i++) {
      const mat = new THREE.SpriteMaterial({ map: texs[i % 3], transparent: true, depthWrite: false, opacity: 0.85, fog: false });
      const s = new THREE.Sprite(mat);
      const a = rnd() * Math.PI * 2, r = 300 + rnd() * 2600;
      s.position.set(Math.cos(a) * r, 280 + rnd() * 320, Math.sin(a) * r);
      const sc = 260 + rnd() * 380;
      s.scale.set(sc, sc * 0.55, 1);
      this.clouds.add(s);
    }
    this.scene.add(this.clouds);
  }

  update(dt, windNED) {
    this.time += dt;
    // windsock points downwind, droops at low speed
    const speed = Math.hypot(windNED[0], windNED[1]);
    const dirTo = Math.atan2(windNED[1], windNED[0]); // direction wind is blowing towards (from north, clockwise)
    // three: sock built along -X; rotate so that -X points to (east=sin, south=-cos)
    this.windsock.rotation.set(0, 0, 0);
    this.windsock.rotation.y = -(dirTo - Math.PI / 2) + Math.PI;
    const droop = THREE.MathUtils.clamp(1 - speed / 8, 0, 1);
    this.windsock.rotation.z = -droop * 1.2 + Math.sin(this.time * 3) * 0.04 * (1 - droop);
    if (this.beacon) this.beacon.visible = Math.floor(this.time * 1.2) % 2 === 0;
    this.clouds.position.x += windNED[1] * dt * 0.3;
    this.clouds.position.z -= windNED[0] * dt * 0.3;
  }
}

// Lightweight geometry merger (positions/normals/uvs of non-indexed or indexed geometries)
function mergeGeometries(geos) {
  const parts = geos.map((g) => (g.index ? g.toNonIndexed() : g));
  let total = 0;
  parts.forEach((g) => { total += g.attributes.position.count; });
  const pos = new Float32Array(total * 3), nor = new Float32Array(total * 3), uv = new Float32Array(total * 2);
  let o = 0;
  for (const g of parts) {
    pos.set(g.attributes.position.array, o * 3);
    nor.set(g.attributes.normal.array, o * 3);
    if (g.attributes.uv) uv.set(g.attributes.uv.array, o * 2);
    o += g.attributes.position.count;
  }
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  out.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  out.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  return out;
}
