// Procedurally generated textures (canvas based) so the simulator needs no
// external image assets and loads instantly from GitHub Pages.

import * as THREE from 'three';

function canvas(w, h = w) {
  const c = document.createElement('canvas');
  c.width = w;
  c.height = h;
  return [c, c.getContext('2d')];
}

// Deterministic PRNG so textures look the same every load
export function mulberry32(seed) {
  return function () {
    let t = (seed += 0x6d2b79f5);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Tileable value noise ----------------------------------------------------------
export function makeNoise(seed = 1) {
  const rnd = mulberry32(seed);
  const P = 256;
  const perm = new Uint8Array(P * 2);
  const vals = new Float32Array(P);
  for (let i = 0; i < P; i++) { perm[i] = i; vals[i] = rnd(); }
  for (let i = P - 1; i > 0; i--) { const j = Math.floor(rnd() * (i + 1)); [perm[i], perm[j]] = [perm[j], perm[i]]; }
  for (let i = 0; i < P; i++) perm[i + P] = perm[i];
  const fade = (t) => t * t * (3 - 2 * t);
  const wrap = (a, period) => (((a % period) + period) % period) & 255;
  const v = (x, y, period) => vals[perm[wrap(x, period) + perm[wrap(y, period)]]];
  function noise2(x, y, period = 256) {
    const xi = Math.floor(x), yi = Math.floor(y);
    const xf = x - xi, yf = y - yi;
    const u = fade(xf), w = fade(yf);
    const a = v(xi, yi, period), b = v(xi + 1, yi, period), c = v(xi, yi + 1, period), d = v(xi + 1, yi + 1, period);
    return a + (b - a) * u + (c - a) * w + (a - b - c + d) * u * w;
  }
  function fbm(x, y, oct = 5, period = 256) {
    let s = 0, amp = 0.5, f = 1, n = 0;
    for (let o = 0; o < oct; o++) { s += amp * noise2(x * f, y * f, period * f); n += amp; amp *= 0.5; f *= 2; }
    return s / n;
  }
  return { noise2, fbm };
}

function toTexture(c, { repeat = 1, srgb = true, aniso = 8 } = {}) {
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(repeat, repeat);
  t.anisotropy = aniso;
  if (srgb) t.colorSpace = THREE.SRGBColorSpace;
  t.needsUpdate = true;
  return t;
}

/** Build a tangent-space normal map from a greyscale height field. */
function normalFromHeight(height, size, strength = 2) {
  const [c, g] = canvas(size);
  const img = g.createImageData(size, size);
  const h = (x, y) => height[((y + size) % size) * size + ((x + size) % size)];
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const dx = (h(x + 1, y) - h(x - 1, y)) * strength;
      const dy = (h(x, y + 1) - h(x, y - 1)) * strength;
      const n = Math.hypot(dx, dy, 1);
      const i = (y * size + x) * 4;
      img.data[i] = ((-dx / n) * 0.5 + 0.5) * 255;
      img.data[i + 1] = ((dy / n) * 0.5 + 0.5) * 255;
      img.data[i + 2] = (1 / n) * 255;
      img.data[i + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  return toTexture(c, { srgb: false });
}

export function grassTextures(size = 512) {
  const { fbm } = makeNoise(7);
  const rnd = mulberry32(3);
  const [c, g] = canvas(size);
  const img = g.createImageData(size, size);
  const height = new Float32Array(size * size);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const n = fbm((x / size) * 8, (y / size) * 8, 5, 8);
      const n2 = fbm((x / size) * 32, (y / size) * 32, 3, 32);
      const i = (y * size + x) * 4;
      const r = rnd();
      height[y * size + x] = n2 * 0.6 + r * 0.4;
      img.data[i] = 60 + n * 38 + r * 16 - 8;
      img.data[i + 1] = 82 + n * 46 + n2 * 16 + r * 18;
      img.data[i + 2] = 38 + n * 16 + r * 10;
      img.data[i + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  // grass blades
  for (let k = 0; k < size * 14; k++) {
    const x = rnd() * size, y = rnd() * size;
    const l = 2 + rnd() * 5;
    const a = -Math.PI / 2 + (rnd() - 0.5) * 1.2;
    const light = rnd();
    g.strokeStyle = `rgba(${72 + light * 60},${96 + light * 70},${40 + light * 28},${0.35 + rnd() * 0.4})`;
    g.lineWidth = 0.6 + rnd() * 0.8;
    g.beginPath();
    g.moveTo(x, y);
    g.lineTo(x + Math.cos(a) * l, y + Math.sin(a) * l);
    g.stroke();
  }
  return { map: toTexture(c), normalMap: normalFromHeight(height, size, 3) };
}

export function sandTextures(size = 512) {
  const { fbm } = makeNoise(11);
  const rnd = mulberry32(5);
  const [c, g] = canvas(size);
  const img = g.createImageData(size, size);
  const height = new Float32Array(size * size);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const n = fbm((x / size) * 6, (y / size) * 6, 5, 6);
      const ripple = Math.sin((x / size) * Math.PI * 2 * 24 + n * 9) * 0.5 + 0.5;
      const r = rnd();
      const i = (y * size + x) * 4;
      height[y * size + x] = ripple * 0.7 + r * 0.3;
      img.data[i] = 196 + n * 40 + r * 12 - ripple * 12;
      img.data[i + 1] = 168 + n * 34 + r * 10 - ripple * 12;
      img.data[i + 2] = 122 + n * 24 + r * 8 - ripple * 10;
      img.data[i + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  return { map: toTexture(c), normalMap: normalFromHeight(height, size, 2) };
}

export function asphaltTextures(size = 512) {
  const rnd = mulberry32(9);
  const { fbm } = makeNoise(21);
  const [c, g] = canvas(size);
  const img = g.createImageData(size, size);
  const height = new Float32Array(size * size);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const n = fbm((x / size) * 10, (y / size) * 10, 4, 10);
      const r = rnd();
      const stone = r > 0.93 ? 30 : 0;
      const v = 58 + n * 30 + r * 16 + stone;
      const i = (y * size + x) * 4;
      height[y * size + x] = r * 0.8 + n * 0.2;
      img.data[i] = v; img.data[i + 1] = v; img.data[i + 2] = v + 3; img.data[i + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  return { map: toTexture(c), normalMap: normalFromHeight(height, size, 1.5) };
}

/** Runway surface with markings (single texture along the runway length). */
export function runwayTexture() {
  const [c, g] = canvas(256, 2048);
  const rnd = mulberry32(12);
  g.fillStyle = '#3d3f42';
  g.fillRect(0, 0, 256, 2048);
  for (let i = 0; i < 30000; i++) {
    const v = 50 + rnd() * 40;
    g.fillStyle = `rgba(${v},${v},${v + 3},0.5)`;
    g.fillRect(rnd() * 256, rnd() * 2048, 1.5, 1.5);
  }
  // tyre marks
  for (const end of [0, 1]) {
    for (let i = 0; i < 40; i++) {
      g.fillStyle = `rgba(20,20,20,${0.05 + rnd() * 0.08})`;
      const y = end ? 2048 - 380 + rnd() * 200 : 180 + rnd() * 200;
      g.fillRect(90 + rnd() * 76, y, 3 + rnd() * 6, 60 + rnd() * 120);
    }
  }
  g.fillStyle = '#e9e9e2';
  // edge lines
  g.fillRect(8, 0, 5, 2048);
  g.fillRect(243, 0, 5, 2048);
  // centre dashes
  for (let y = 260; y < 2048 - 260; y += 70) g.fillRect(125, y, 6, 38);
  // thresholds (piano keys)
  for (const y of [30, 2048 - 110]) for (let x = 22; x < 234; x += 22) g.fillRect(x, y, 12, 80);
  // numbers
  g.font = 'bold 64px Arial';
  g.textAlign = 'center';
  g.save(); g.translate(128, 190); g.fillText('36', 0, 20); g.restore();
  g.save(); g.translate(128, 2048 - 170); g.rotate(Math.PI); g.fillText('18', 0, 20); g.restore();
  const t = toTexture(c);
  t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping;
  return t;
}

export function helipadTexture() {
  const S = 1024;
  const [c, g] = canvas(S);
  const rnd = mulberry32(4);
  g.fillStyle = '#6d6f72';
  g.fillRect(0, 0, S, S);
  for (let i = 0; i < 40000; i++) {
    const v = 90 + rnd() * 40;
    g.fillStyle = `rgba(${v},${v},${v},0.35)`;
    g.fillRect(rnd() * S, rnd() * S, 2, 2);
  }
  // concrete slab joints
  g.strokeStyle = 'rgba(40,40,40,0.5)';
  g.lineWidth = 3;
  for (let i = 1; i < 4; i++) {
    g.beginPath(); g.moveTo((i * S) / 4, 0); g.lineTo((i * S) / 4, S); g.stroke();
    g.beginPath(); g.moveTo(0, (i * S) / 4); g.lineTo(S, (i * S) / 4); g.stroke();
  }
  g.strokeStyle = '#f2c230';
  g.lineWidth = 26;
  g.beginPath(); g.arc(S / 2, S / 2, S * 0.42, 0, Math.PI * 2); g.stroke();
  g.strokeStyle = '#ffffff';
  g.lineWidth = 10;
  g.beginPath(); g.arc(S / 2, S / 2, S * 0.36, 0, Math.PI * 2); g.stroke();
  g.fillStyle = '#ffffff';
  g.font = `bold ${S * 0.42}px Arial`;
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText('H', S / 2, S / 2 + S * 0.02);
  // north arrow
  g.fillStyle = '#f2c230';
  g.beginPath(); g.moveTo(S / 2, 18); g.lineTo(S / 2 - 26, 70); g.lineTo(S / 2 + 26, 70); g.fill();
  const t = toTexture(c);
  t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping;
  return t;
}

export function facadeTexture(seed = 1, { cols = 8, rows = 12, base = '#c9c3b8', glass = '#27455e' } = {}) {
  const rnd = mulberry32(seed);
  const [c, g] = canvas(512);
  g.fillStyle = base;
  g.fillRect(0, 0, 512, 512);
  const cw = 512 / cols, rh = 512 / rows;
  for (let r = 0; r < rows; r++) {
    for (let k = 0; k < cols; k++) {
      const lit = rnd() > 0.82;
      const grd = g.createLinearGradient(0, r * rh, 0, (r + 1) * rh);
      grd.addColorStop(0, lit ? '#f4e1a8' : glass);
      grd.addColorStop(1, lit ? '#d9b86a' : '#101d29');
      g.fillStyle = grd;
      g.fillRect(k * cw + cw * 0.14, r * rh + rh * 0.18, cw * 0.72, rh * 0.62);
      g.fillStyle = 'rgba(255,255,255,0.12)';
      g.fillRect(k * cw + cw * 0.14, r * rh + rh * 0.18, cw * 0.72, rh * 0.08);
    }
  }
  return toTexture(c);
}

export function hangarTexture() {
  const [c, g] = canvas(512);
  const grd = g.createLinearGradient(0, 0, 512, 0);
  for (let i = 0; i <= 32; i++) grd.addColorStop(i / 32, i % 2 ? '#aeb4b9' : '#c8ccd0');
  g.fillStyle = grd;
  g.fillRect(0, 0, 512, 512);
  g.fillStyle = 'rgba(80,60,40,0.12)';
  for (let i = 0; i < 60; i++) g.fillRect(Math.random() * 512, 380 + Math.random() * 132, 3, 40);
  return toTexture(c);
}

export function pcbTexture() {
  const [c, g] = canvas(512);
  const rnd = mulberry32(8);
  g.fillStyle = '#10261a';
  g.fillRect(0, 0, 512, 512);
  g.strokeStyle = 'rgba(70,140,90,0.55)';
  g.lineWidth = 2;
  for (let i = 0; i < 90; i++) {
    let x = rnd() * 512, y = rnd() * 512;
    g.beginPath(); g.moveTo(x, y);
    for (let s = 0; s < 4; s++) {
      if (rnd() > 0.5) x += (rnd() - 0.5) * 160; else y += (rnd() - 0.5) * 160;
      g.lineTo(x, y);
    }
    g.stroke();
  }
  // solder pads
  g.fillStyle = '#d4b25a';
  const pads = [[256, 30], [256, 482], [30, 256], [482, 256]];
  for (const [x, y] of pads) { g.fillRect(x - 40, y - 16, 30, 32); g.fillRect(x + 10, y - 16, 30, 32); }
  g.fillStyle = '#e8e8e8';
  g.font = 'bold 22px Arial';
  g.textAlign = 'center';
  g.fillText('F450  FLAME WHEEL', 256, 250);
  g.font = '16px Arial';
  g.fillText('+ BAT -', 256, 280);
  return toTexture(c);
}

export function batteryLabelTexture() {
  const [c, g] = canvas(512, 256);
  const grd = g.createLinearGradient(0, 0, 512, 256);
  grd.addColorStop(0, '#1b1d22');
  grd.addColorStop(1, '#2c3038');
  g.fillStyle = grd;
  g.fillRect(0, 0, 512, 256);
  g.fillStyle = '#e8b22b';
  g.fillRect(0, 0, 512, 40);
  g.fillStyle = '#111';
  g.font = 'bold 28px Arial';
  g.fillText('LiPo  3S1P  11.1V', 20, 30);
  g.fillStyle = '#fff';
  g.font = 'bold 76px Arial';
  g.fillText('2200', 24, 140);
  g.font = 'bold 36px Arial';
  g.fillText('mAh  35C', 250, 140);
  g.fillStyle = '#9aa0a6';
  g.font = '22px Arial';
  g.fillText('24.4Wh  •  Discharge 77A', 24, 200);
  return toTexture(c);
}

export function fcLabelTexture() {
  const [c, g] = canvas(256);
  const grd = g.createLinearGradient(0, 0, 0, 256);
  grd.addColorStop(0, '#f4f5f6');
  grd.addColorStop(1, '#d7dadd');
  g.fillStyle = grd;
  g.fillRect(0, 0, 256, 256);
  g.fillStyle = '#222';
  g.font = 'bold 30px Arial';
  g.textAlign = 'center';
  g.fillText('PIXHAWK', 128, 116);
  g.font = '18px Arial';
  g.fillStyle = '#555';
  g.fillText('Autopilot', 128, 146);
  g.fillStyle = '#333';
  g.beginPath(); g.moveTo(128, 20); g.lineTo(108, 60); g.lineTo(148, 60); g.fill();
  return toTexture(c);
}

export function propBlurTexture(color = [30, 30, 30]) {
  const [c, g] = canvas(256);
  const grd = g.createRadialGradient(128, 128, 8, 128, 128, 128);
  const [r, gg, b] = color;
  grd.addColorStop(0, `rgba(${r},${gg},${b},0.0)`);
  grd.addColorStop(0.12, `rgba(${r},${gg},${b},0.45)`);
  grd.addColorStop(0.6, `rgba(${r},${gg},${b},0.28)`);
  grd.addColorStop(0.9, `rgba(${r},${gg},${b},0.18)`);
  grd.addColorStop(0.97, `rgba(230,230,230,0.35)`);
  grd.addColorStop(1, `rgba(${r},${gg},${b},0)`);
  g.fillStyle = grd;
  g.fillRect(0, 0, 256, 256);
  // subtle blade streaks
  g.globalCompositeOperation = 'destination-out';
  for (let i = 0; i < 6; i++) {
    g.strokeStyle = 'rgba(0,0,0,0.12)';
    g.lineWidth = 6;
    g.beginPath(); g.arc(128, 128, 40 + i * 14, 0, Math.PI * 2); g.stroke();
  }
  return toTexture(c);
}

export function cloudTexture(seed = 1) {
  const rnd = mulberry32(seed);
  const [c, g] = canvas(256);
  for (let i = 0; i < 26; i++) {
    const x = 50 + rnd() * 156, y = 90 + rnd() * 70, r = 20 + rnd() * 50;
    const grd = g.createRadialGradient(x, y, 0, x, y, r);
    const shade = 225 + rnd() * 30;
    grd.addColorStop(0, `rgba(${shade},${shade},${shade + 5},0.55)`);
    grd.addColorStop(1, 'rgba(255,255,255,0)');
    g.fillStyle = grd;
    g.fillRect(0, 0, 256, 256);
  }
  const t = toTexture(c);
  t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping;
  return t;
}

export function barkTexture() {
  const [c, g] = canvas(128);
  const rnd = mulberry32(31);
  g.fillStyle = '#4a3527';
  g.fillRect(0, 0, 128, 128);
  for (let i = 0; i < 120; i++) {
    g.strokeStyle = `rgba(${30 + rnd() * 40},${20 + rnd() * 25},${12 + rnd() * 15},0.8)`;
    g.lineWidth = 1 + rnd() * 2;
    const x = rnd() * 128;
    g.beginPath(); g.moveTo(x, 0); g.lineTo(x + (rnd() - 0.5) * 8, 128); g.stroke();
  }
  return toTexture(c);
}

export function carbonTexture() {
  const [c, g] = canvas(128);
  for (let y = 0; y < 128; y += 8) {
    for (let x = 0; x < 128; x += 8) {
      const odd = ((x + y) / 8) % 2;
      const grd = odd ? g.createLinearGradient(x, y, x + 8, y) : g.createLinearGradient(x, y, x, y + 8);
      grd.addColorStop(0, '#15171a');
      grd.addColorStop(0.5, '#3a3e44');
      grd.addColorStop(1, '#15171a');
      g.fillStyle = grd;
      g.fillRect(x, y, 8, 8);
    }
  }
  return toTexture(c, { repeat: 4 });
}
