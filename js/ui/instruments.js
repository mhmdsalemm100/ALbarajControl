// Canvas-drawn flight instruments in a QGroundControl-like style.

const RAD = 180 / Math.PI;

function setup(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const g = canvas.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { g, w, h };
}

export function drawAttitude(canvas, roll, pitch) {
  const { g, w, h } = setup(canvas);
  const r = Math.min(w, h) / 2 - 2;
  const cx = w / 2, cy = h / 2;
  g.clearRect(0, 0, w, h);
  g.save();
  g.beginPath();
  g.arc(cx, cy, r, 0, Math.PI * 2);
  g.clip();
  g.translate(cx, cy);
  g.rotate(-roll);
  const pxPerDeg = r / 35;
  const py = pitch * RAD * pxPerDeg;
  // sky / ground
  const sky = g.createLinearGradient(0, -r * 2 + py, 0, py);
  sky.addColorStop(0, '#1c5aa6');
  sky.addColorStop(1, '#5fa8e8');
  g.fillStyle = sky;
  g.fillRect(-r * 2, -r * 3 + py, r * 4, r * 3);
  const gnd = g.createLinearGradient(0, py, 0, py + r * 2);
  gnd.addColorStop(0, '#8a5a2b');
  gnd.addColorStop(1, '#4e3217');
  g.fillStyle = gnd;
  g.fillRect(-r * 2, py, r * 4, r * 3);
  g.strokeStyle = '#fff';
  g.lineWidth = 1.5;
  g.beginPath(); g.moveTo(-r * 2, py); g.lineTo(r * 2, py); g.stroke();
  // pitch ladder
  g.font = `${Math.max(8, r * 0.13)}px sans-serif`;
  g.fillStyle = '#fff';
  g.textAlign = 'left';
  g.textBaseline = 'middle';
  for (let d = -60; d <= 60; d += 5) {
    if (!d) continue;
    const y = py - d * pxPerDeg;
    const len = d % 10 === 0 ? r * 0.36 : r * 0.18;
    g.beginPath(); g.moveTo(-len / 2, y); g.lineTo(len / 2, y); g.stroke();
    if (d % 10 === 0) { g.fillText(String(Math.abs(d)), len / 2 + 3, y); }
  }
  g.restore();
  // roll scale
  g.save();
  g.translate(cx, cy);
  g.strokeStyle = '#fff';
  g.fillStyle = '#fff';
  g.lineWidth = 1.5;
  for (const a of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
    const t = (a - 90) / RAD;
    const l = a % 30 === 0 ? 0.14 : 0.08;
    g.beginPath();
    g.moveTo(Math.cos(t) * r * 0.97, Math.sin(t) * r * 0.97);
    g.lineTo(Math.cos(t) * r * (0.97 - l), Math.sin(t) * r * (0.97 - l));
    g.stroke();
  }
  g.rotate(-roll);
  g.beginPath();
  g.moveTo(0, -r * 0.8); g.lineTo(-r * 0.06, -r * 0.7); g.lineTo(r * 0.06, -r * 0.7); g.closePath();
  g.fill();
  g.restore();
  // aircraft symbol
  g.save();
  g.translate(cx, cy);
  g.strokeStyle = '#ffd400';
  g.lineWidth = 3;
  g.lineCap = 'round';
  g.beginPath();
  g.moveTo(-r * 0.5, 0); g.lineTo(-r * 0.18, 0); g.lineTo(-r * 0.08, r * 0.08);
  g.moveTo(r * 0.5, 0); g.lineTo(r * 0.18, 0); g.lineTo(r * 0.08, r * 0.08);
  g.stroke();
  g.fillStyle = '#ffd400';
  g.beginPath(); g.arc(0, 0, 3, 0, Math.PI * 2); g.fill();
  g.restore();
  g.strokeStyle = 'rgba(255,255,255,0.8)';
  g.lineWidth = 2;
  g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.stroke();
}

export function drawCompass(canvas, headingDeg, { homeBearing = null, targetBearing = null } = {}) {
  const { g, w, h } = setup(canvas);
  const r = Math.min(w, h) / 2 - 2;
  const cx = w / 2, cy = h / 2;
  g.clearRect(0, 0, w, h);
  g.fillStyle = '#0f1318';
  g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.fill();
  g.save();
  g.translate(cx, cy);
  g.rotate(-headingDeg / RAD);
  g.strokeStyle = '#cfd8dc';
  g.fillStyle = '#fff';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  for (let a = 0; a < 360; a += 10) {
    const t = (a - 90) / RAD;
    const l = a % 30 === 0 ? 0.14 : 0.07;
    g.lineWidth = a % 30 === 0 ? 1.8 : 1;
    g.beginPath();
    g.moveTo(Math.cos(t) * r * 0.96, Math.sin(t) * r * 0.96);
    g.lineTo(Math.cos(t) * r * (0.96 - l), Math.sin(t) * r * (0.96 - l));
    g.stroke();
  }
  const labels = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' };
  g.font = `bold ${Math.max(9, r * 0.2)}px sans-serif`;
  for (const [a, txt] of Object.entries(labels)) {
    const t = (a - 90) / RAD;
    g.save();
    g.translate(Math.cos(t) * r * 0.66, Math.sin(t) * r * 0.66);
    g.rotate(a / RAD);
    g.fillStyle = txt === 'N' ? '#ff5252' : '#fff';
    g.fillText(txt, 0, 0);
    g.restore();
  }
  const marker = (bearing, color, label) => {
    if (bearing === null || bearing === undefined) return;
    const t = (bearing - 90) / RAD;
    g.fillStyle = color;
    g.beginPath(); g.arc(Math.cos(t) * r * 0.82, Math.sin(t) * r * 0.82, r * 0.08, 0, Math.PI * 2); g.fill();
    g.fillStyle = '#000';
    g.font = `bold ${Math.max(7, r * 0.1)}px sans-serif`;
    g.fillText(label, Math.cos(t) * r * 0.82, Math.sin(t) * r * 0.82 + 0.5);
  };
  marker(homeBearing, '#34c759', 'H');
  marker(targetBearing, '#ffcc00', 'T');
  g.restore();
  // vehicle symbol
  g.save();
  g.translate(cx, cy);
  g.fillStyle = '#ff6a13';
  g.beginPath();
  g.moveTo(0, -r * 0.42); g.lineTo(r * 0.22, r * 0.3); g.lineTo(0, r * 0.16); g.lineTo(-r * 0.22, r * 0.3); g.closePath();
  g.fill();
  g.restore();
  // lubber line and heading box
  g.fillStyle = '#fff';
  g.beginPath(); g.moveTo(cx, cy - r + 1); g.lineTo(cx - 5, cy - r - 5 + 8); g.lineTo(cx + 5, cy - r - 5 + 8); g.fill();
  g.strokeStyle = 'rgba(255,255,255,0.8)';
  g.lineWidth = 2;
  g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.stroke();
}

/** Simple rolling line chart. series: [{values:[], color, label}] */
export function drawChart(canvas, series, { min = null, max = null } = {}) {
  const { g, w, h } = setup(canvas);
  g.clearRect(0, 0, w, h);
  g.fillStyle = '#0d1116';
  g.fillRect(0, 0, w, h);
  let lo = min, hi = max;
  if (lo === null || hi === null) {
    const all = series.flatMap((s) => s.values);
    lo = lo ?? Math.min(...all, 0);
    hi = hi ?? Math.max(...all, 1);
  }
  if (hi - lo < 1e-6) hi = lo + 1;
  g.strokeStyle = 'rgba(255,255,255,0.08)';
  g.lineWidth = 1;
  g.fillStyle = 'rgba(255,255,255,0.5)';
  g.font = '10px sans-serif';
  for (let i = 0; i <= 4; i++) {
    const y = 6 + ((h - 12) * i) / 4;
    g.beginPath(); g.moveTo(0, y); g.lineTo(w, y); g.stroke();
    g.fillText((hi - ((hi - lo) * i) / 4).toFixed(1), 4, y + 10 > h ? y - 2 : y + 10);
  }
  series.forEach((s, k) => {
    const n = s.values.length;
    if (n < 2) return;
    g.strokeStyle = s.color;
    g.lineWidth = 1.6;
    g.beginPath();
    s.values.forEach((v, i) => {
      const x = (i / (n - 1)) * w;
      const y = 6 + (h - 12) * (1 - (v - lo) / (hi - lo));
      if (i) g.lineTo(x, y); else g.moveTo(x, y);
    });
    g.stroke();
    g.fillStyle = s.color;
    g.fillText(s.label, w - 90, 14 + k * 13);
  });
}
