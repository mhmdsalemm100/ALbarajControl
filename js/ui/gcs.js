// QGroundControl-style ground station: map (fly view), mission planner,
// analyze tools (MAVLink inspector, live charts, message log) and setup.

import { MAV_CMD, MAV_FRAME } from '../mavlink/mavlink.js';
import { distance } from '../core/geo.js';
import { drawChart } from './instruments.js';

const L = window.L;
const $ = (id) => document.getElementById(id);

export const CMD_INFO = {
  [MAV_CMD.NAV_WAYPOINT]: { name: 'Waypoint', cls: '', pos: true },
  [MAV_CMD.NAV_LOITER_TIME]: { name: 'Loiter (time)', cls: '', pos: true },
  [MAV_CMD.NAV_TAKEOFF]: { name: 'Takeoff', cls: 't', pos: false },
  [MAV_CMD.NAV_LAND]: { name: 'Land', cls: 'l', pos: true },
  [MAV_CMD.NAV_RETURN_TO_LAUNCH]: { name: 'Return', cls: 'r', pos: false },
  [MAV_CMD.DO_CHANGE_SPEED]: { name: 'Change speed', cls: 'r', pos: false },
};

export class GroundStation {
  constructor(app) {
    this.app = app;
    this.plan = [];
    this.missionCurrent = -1;
    this.history = { roll: [], pitch: [], alt: [], climb: [], m: [[], [], [], []] };
    this.selectedMsg = null;
    this.initMap();
    this.initPlan();
    this.initAnalyze();
    this.initSetup();
  }

  // ================================================================== map
  initMap() {
    const home = this.app.homeGeo();
    const map = L.map('map', { zoomControl: true, attributionControl: true, maxZoom: 21 }).setView([home.lat, home.lon], 18);
    const sat = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
      maxZoom: 21, maxNativeZoom: 19, attribution: 'Imagery &copy; Esri, Maxar, Earthstar Geographics',
    });
    const osm = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 21, maxNativeZoom: 19, attribution: '&copy; OpenStreetMap contributors' });
    sat.addTo(map);
    L.control.layers({ Satellite: sat, Street: osm }, {}, { position: 'topright' }).addTo(map);
    L.control.scale({ imperial: false }).addTo(map);
    this.map = map;

    const vehSvg = '<svg viewBox="0 0 40 40" width="40" height="40"><path d="M20 3 L33 34 L20 27 L7 34 Z" fill="#ff6a13" stroke="#fff" stroke-width="2.2" stroke-linejoin="round"/></svg>';
    this.vehMarker = L.marker([home.lat, home.lon], {
      icon: L.divIcon({ className: '', html: `<div class="veh-icon">${vehSvg}</div>`, iconSize: [40, 40], iconAnchor: [20, 20] }),
      zIndexOffset: 1000, interactive: false,
    }).addTo(map);
    this.homeMarker = L.marker([home.lat, home.lon], {
      icon: L.divIcon({ className: '', html: '<div class="home-icon">H</div>', iconSize: [24, 24], iconAnchor: [12, 12] }),
      interactive: false,
    }).addTo(map);
    this.track = L.polyline([], { color: '#ff6a13', weight: 3, opacity: 0.85 }).addTo(map);
    this.missionLine = L.polyline([], { color: '#ffe14d', weight: 3, dashArray: '8 6' }).addTo(map);
    this.wpLayer = L.layerGroup().addTo(map);
    this.follow = true;
    map.on('dragstart', () => { this.follow = false; });

    map.on('click', (e) => {
      if (this.app.view === 'plan') this.addWaypoint(e.latlng.lat, e.latlng.lng);
    });
    map.on('contextmenu', (e) => {
      if (this.app.view === 'plan') return;
      const div = document.createElement('div');
      div.className = 'map-menu';
      const b1 = document.createElement('button');
      b1.textContent = 'Go to location';
      b1.onclick = () => { this.app.vehicle.gotoLocation(e.latlng.lat, e.latlng.lng); map.closePopup(); };
      const b2 = document.createElement('button');
      b2.textContent = 'Center / follow vehicle';
      b2.onclick = () => { this.follow = true; map.closePopup(); };
      div.append(b1, b2);
      L.popup().setLatLng(e.latlng).setContent(div).openOn(map);
    });
    // tap-and-hold on touch screens
    let holdTimer;
    map.on('mousedown touchstart', (e) => { holdTimer = setTimeout(() => map.fire('contextmenu', e), 700); });
    map.on('mouseup touchend mousemove dragstart', () => clearTimeout(holdTimer));
  }

  invalidate() { setTimeout(() => this.map.invalidateSize(), 280); }

  updateMap(t) {
    if (!t || !t.lat) return;
    const ll = [t.lat, t.lon];
    this.vehMarker.setLatLng(ll);
    const el = this.vehMarker.getElement()?.querySelector('.veh-icon');
    if (el) el.style.transform = `rotate(${t.heading}deg)`;
    if (t.home) this.homeMarker.setLatLng([t.home.lat, t.home.lon]);
    const pts = this.track.getLatLngs();
    const last = pts[pts.length - 1];
    if (t.armed && (!last || distance(last.lat, last.lng, t.lat, t.lon) > 0.8)) {
      this.track.addLatLng(ll);
      if (pts.length > 4000) this.track.setLatLngs(pts.slice(-3000));
    }
    if (this.follow && this.app.view !== 'plan') {
      if (!this.map.getBounds().pad(-0.25).contains(ll)) this.map.panTo(ll, { animate: true });
    }
    if (t.missionIndex !== this.missionCurrent) {
      this.missionCurrent = t.missionIndex;
      this.renderPlan(false);
    }
  }

  clearTrack() { this.track.setLatLngs([]); }

  // ================================================================== plan
  initPlan() {
    const alt = () => Number($('planAlt').value) || 20;
    $('planTakeoff').onclick = () => { this.plan.unshift({ command: MAV_CMD.NAV_TAKEOFF, alt: alt() }); this.planChanged(); };
    $('planRtl').onclick = () => { this.plan.push({ command: MAV_CMD.NAV_RETURN_TO_LAUNCH }); this.planChanged(); };
    $('planLand').onclick = () => {
      const c = this.map.getCenter();
      this.plan.push({ command: MAV_CMD.NAV_LAND, lat: c.lat, lon: c.lng, alt: 0 });
      this.planChanged();
    };
    $('planClear').onclick = () => { if (!this.plan.length || confirm('Clear all mission items?')) { this.plan = []; this.planChanged(); } };
    $('planOrbit').onclick = () => this.generateOrbit();
    $('planGrid').onclick = () => this.generateSurvey();
    $('planUpload').onclick = () => this.upload();
    $('planStart').onclick = async () => { if (await this.upload()) this.app.vehicle.startMission(); };
    $('planDownload').onclick = () => this.download();
    $('planSave').onclick = () => this.savePlan();
    $('planOpen').onchange = (e) => { const f = e.target.files[0]; if (f) f.text().then((txt) => this.loadPlan(txt)); e.target.value = ''; };
    $('planAlt').onchange = () => this.renderPlan();
    $('planSpeed').onchange = () => this.renderPlan();
    this.renderPlan();
  }

  addWaypoint(lat, lon) {
    const alt = Number($('planAlt').value) || 20;
    if (!this.plan.length) this.plan.push({ command: MAV_CMD.NAV_TAKEOFF, alt });
    // insert before a trailing RTL / land
    const lastCmd = this.plan[this.plan.length - 1]?.command;
    const item = { command: MAV_CMD.NAV_WAYPOINT, lat, lon, alt, param1: 0 };
    if (lastCmd === MAV_CMD.NAV_RETURN_TO_LAUNCH || lastCmd === MAV_CMD.NAV_LAND) this.plan.splice(this.plan.length - 1, 0, item);
    else this.plan.push(item);
    this.planChanged();
  }

  planChanged() {
    this.dirty = true;
    this.renderPlan();
  }

  missionPath() {
    const home = this.app.homeGeo();
    const pts = [[home.lat, home.lon]];
    let last = pts[0];
    for (const it of this.plan) {
      if (CMD_INFO[it.command]?.pos && it.lat) { last = [it.lat, it.lon]; pts.push(last); }
      if (it.command === MAV_CMD.NAV_RETURN_TO_LAUNCH) pts.push([home.lat, home.lon]);
    }
    return pts;
  }

  renderPlan(full = true) {
    const list = $('planList');
    if (full) list.innerHTML = '';
    this.wpLayer.clearLayers();
    const home = this.app.homeGeo();
    let n = 0;
    this.plan.forEach((it, i) => {
      const info = CMD_INFO[it.command] || { name: `CMD ${it.command}`, cls: '', pos: !!it.lat };
      n++;
      if (full) {
        const li = document.createElement('li');
        li.dataset.i = i;
        const opts = Object.entries(CMD_INFO).map(([c, v]) => `<option value="${c}" ${Number(c) === it.command ? 'selected' : ''}>${v.name}</option>`).join('');
        const altField = it.command !== MAV_CMD.NAV_RETURN_TO_LAUNCH && it.command !== MAV_CMD.DO_CHANGE_SPEED && it.command !== MAV_CMD.NAV_LAND
          ? `<label>Alt <input type="number" data-k="alt" value="${it.alt ?? 20}" step="1"/></label>` : '';
        const holdField = it.command === MAV_CMD.NAV_WAYPOINT || it.command === MAV_CMD.NAV_LOITER_TIME
          ? `<label>Hold <input type="number" data-k="param1" value="${it.param1 ?? 0}" min="0" step="1"/> s</label>` : '';
        const speedField = it.command === MAV_CMD.DO_CHANGE_SPEED ? `<label>Speed <input type="number" data-k="param2" value="${it.param2 ?? 5}" min="1" step="0.5"/> m/s</label>` : '';
        li.innerHTML = `<div class="num ${info.cls}">${n}</div>
          <div class="fields"><select data-k="command">${opts}</select>${altField}${holdField}${speedField}</div>
          <button class="del" title="Delete">✕</button>`;
        li.querySelector('.del').onclick = () => { this.plan.splice(i, 1); this.planChanged(); };
        li.querySelectorAll('[data-k]').forEach((el) => {
          el.onchange = () => {
            const k = el.dataset.k;
            it[k] = Number(el.value);
            if (k === 'command' && CMD_INFO[it.command]?.pos && !it.lat) { const c = this.map.getCenter(); it.lat = c.lat; it.lon = c.lng; }
            this.planChanged();
          };
        });
        list.appendChild(li);
      }
      const cur = this.missionCurrent === i;
      if (full) list.children[i]?.classList.toggle('current', cur);
      if (info.pos && it.lat) {
        const m = L.marker([it.lat, it.lon], {
          draggable: this.app.view === 'plan',
          icon: L.divIcon({ className: '', html: `<div class="wp-icon ${info.cls} ${cur ? 'cur' : ''}">${n}</div>`, iconSize: [26, 26], iconAnchor: [13, 13] }),
        });
        m.on('dragend', () => { const p = m.getLatLng(); it.lat = p.lat; it.lon = p.lng; this.planChanged(); });
        m.bindTooltip(`${n}: ${info.name}${it.alt !== undefined ? ` · ${it.alt} m` : ''}`);
        this.wpLayer.addLayer(m);
      }
    });
    if (!full) [...list.children].forEach((li, i) => li.classList.toggle('current', this.missionCurrent === i));
    const path = this.missionPath();
    this.missionLine.setLatLngs(path.length > 1 ? path : []);
    // stats
    let dist = 0;
    for (let i = 1; i < path.length; i++) dist += distance(path[i - 1][0], path[i - 1][1], path[i][0], path[i][1]);
    const speed = Number($('planSpeed').value) || 5;
    const hold = this.plan.reduce((s, it) => s + (it.param1 && CMD_INFO[it.command]?.pos ? it.param1 : 0), 0);
    const tt = dist / speed + hold + 20;
    $('planStats').textContent = this.plan.length
      ? `${this.plan.length} items · ${dist.toFixed(0)} m · ~${Math.floor(tt / 60)} min ${Math.round(tt % 60)} s`
      : 'No mission items yet.';
    this.app.onPlanChanged?.(this.plan, home);
  }

  missionForUpload() {
    const speed = Number($('planSpeed').value) || 5;
    const items = this.plan.map((it) => ({ ...it }));
    // set the cruise speed at the start of the mission
    const insertAt = items[0]?.command === MAV_CMD.NAV_TAKEOFF ? 1 : 0;
    items.splice(insertAt, 0, { command: MAV_CMD.DO_CHANGE_SPEED, param1: 1, param2: speed, param3: -1, frame: MAV_FRAME.MISSION });
    return items;
  }

  async upload() {
    if (!this.plan.length) { this.app.log('Mission is empty', 4); return false; }
    const bar = $('planProgress');
    bar.style.width = '10%';
    try {
      await this.app.vehicle.uploadMission(this.missionForUpload(), (p) => { bar.style.width = `${Math.round(p * 100)}%`; });
      bar.style.width = '100%';
      this.dirty = false;
      this.app.log(`Mission uploaded (${this.plan.length} items)`);
      setTimeout(() => { bar.style.width = '0'; }, 1500);
      return true;
    } catch (e) {
      bar.style.width = '0';
      this.app.log(e.message, 3);
      return false;
    }
  }

  async download() {
    try {
      const items = await this.app.vehicle.downloadMission();
      this.plan = items.filter((it) => it.command !== MAV_CMD.DO_CHANGE_SPEED);
      this.planChanged();
      this.app.log(`Mission downloaded (${this.plan.length} items)`);
    } catch (e) { this.app.log(e.message, 3); }
  }

  generateOrbit() {
    const c = this.map.getCenter();
    const r = 40, alt = Number($('planAlt').value) || 20;
    const home = this.app.homeGeo();
    const items = [{ command: MAV_CMD.NAV_TAKEOFF, alt }];
    for (let k = 0; k <= 12; k++) {
      const a = (k / 12) * Math.PI * 2;
      items.push({
        command: MAV_CMD.NAV_WAYPOINT, alt, param1: 0,
        lat: c.lat + (r * Math.cos(a)) / 111320, lon: c.lng + (r * Math.sin(a)) / (111320 * Math.cos((home.lat * Math.PI) / 180)),
      });
    }
    items.push({ command: MAV_CMD.NAV_RETURN_TO_LAUNCH });
    this.plan = items;
    this.planChanged();
  }

  generateSurvey() {
    const b = this.map.getBounds().pad(-0.3);
    const alt = Number($('planAlt').value) || 20;
    const spacingM = 18;
    const lat0 = b.getSouth(), lat1 = b.getNorth(), lon0 = b.getWest(), lon1 = b.getEast();
    const dLat = spacingM / 111320;
    const items = [{ command: MAV_CMD.NAV_TAKEOFF, alt }];
    let flip = false;
    for (let lat = lat0; lat <= lat1 && items.length < 60; lat += dLat) {
      const a = flip ? [lon1, lon0] : [lon0, lon1];
      items.push({ command: MAV_CMD.NAV_WAYPOINT, lat, lon: a[0], alt, param1: 0 });
      items.push({ command: MAV_CMD.NAV_WAYPOINT, lat, lon: a[1], alt, param1: 0 });
      flip = !flip;
    }
    items.push({ command: MAV_CMD.NAV_RETURN_TO_LAUNCH });
    this.plan = items;
    this.planChanged();
  }

  /** QGroundControl .plan (JSON) export */
  savePlan() {
    const home = this.app.homeGeo();
    const items = this.missionForUpload().map((it, i) => {
      const frame = it.frame ?? (CMD_INFO[it.command]?.pos || it.command === MAV_CMD.NAV_TAKEOFF ? MAV_FRAME.GLOBAL_RELATIVE_ALT : MAV_FRAME.MISSION);
      const pos = frame === MAV_FRAME.GLOBAL_RELATIVE_ALT;
      return {
        type: 'SimpleItem', autoContinue: true, command: it.command, doJumpId: i + 1, frame,
        params: [it.param1 ?? 0, it.param2 ?? 0, it.param3 ?? 0, null, pos ? (it.lat ?? 0) : 0, pos ? (it.lon ?? 0) : 0, pos ? (it.alt ?? 0) : 0],
        ...(pos ? { Altitude: it.alt ?? 0, AltitudeMode: 1, AMSLAltAboveTerrain: null } : {}),
      };
    });
    const plan = {
      fileType: 'Plan', version: 1, groundStation: 'Dronee',
      geoFence: { circles: [], polygons: [], version: 2 },
      rallyPoints: { points: [], version: 2 },
      mission: {
        cruiseSpeed: 15, hoverSpeed: Number($('planSpeed').value) || 5, firmwareType: 12, vehicleType: 2, version: 2,
        globalPlanAltitudeMode: 1, plannedHomePosition: [home.lat, home.lon, home.alt], items,
      },
    };
    const blob = new Blob([JSON.stringify(plan, null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'mission.plan';
    a.click();
    URL.revokeObjectURL(a.href);
  }

  loadPlan(text) {
    try {
      const p = JSON.parse(text);
      const items = [];
      let skipped = 0;
      for (const it of p.mission?.items || []) {
        if (it.type !== 'SimpleItem') { skipped++; continue; }
        if (it.command === MAV_CMD.DO_CHANGE_SPEED) { if (it.params[1] > 0) $('planSpeed').value = it.params[1]; continue; }
        const [p1, p2, p3, , lat, lon, alt] = it.params;
        items.push({ command: it.command, param1: p1 ?? 0, param2: p2 ?? 0, param3: p3 ?? 0, lat: lat || undefined, lon: lon || undefined, alt: alt ?? it.Altitude });
      }
      this.plan = items;
      this.planChanged();
      if (items.length) {
        const pts = items.filter((i) => i.lat).map((i) => [i.lat, i.lon]);
        if (pts.length) this.map.fitBounds(pts, { padding: [40, 40] });
      }
      this.app.log(`Loaded ${items.length} mission items${skipped ? ` (${skipped} complex items skipped)` : ''}`);
    } catch (e) {
      this.app.log(`Could not read plan: ${e.message}`, 3);
    }
  }

  // ================================================================== analyze
  initAnalyze() {
    $('inspList').onclick = (e) => {
      const li = e.target.closest('li');
      if (li) { this.selectedMsg = li.dataset.name; }
    };
  }

  pushHistory(t) {
    const h = this.history;
    const push = (arr, v) => { arr.push(v); if (arr.length > 300) arr.shift(); };
    push(h.roll, (t.roll * 180) / Math.PI);
    push(h.pitch, (t.pitch * 180) / Math.PI);
    push(h.alt, t.altRel);
    push(h.climb, t.climb);
    for (let i = 0; i < 4; i++) push(h.m[i], (t.motors?.[i] ?? 0) * 100);
  }

  updateAnalyze(t) {
    const h = this.history;
    drawChart($('chartAtt'), [
      { values: h.roll, color: '#ff6a13', label: 'Roll °' },
      { values: h.pitch, color: '#3ea6ff', label: 'Pitch °' },
    ]);
    drawChart($('chartAlt'), [
      { values: h.alt, color: '#34c759', label: 'Alt m' },
      { values: h.climb, color: '#ffcc00', label: 'Climb m/s' },
    ]);
    drawChart($('chartMot'), h.m.map((v, i) => ({ values: v, color: ['#ff453a', '#ff9f0a', '#30d158', '#64d2ff'][i], label: `Motor ${i + 1} %` })), { min: 0, max: 100 });

    const link = this.app.link;
    const list = $('inspList');
    const detail = $('inspDetail');
    if (link) {
      $('inspSrc').textContent = `— ${link.autopilot?.toUpperCase() || ''} sys ${link.target.sys}`;
      const entries = [...link.inspector.entries()].sort((a, b) => a[0].localeCompare(b[0]));
      const key = entries.map((e) => e[0]).join();
      if (key !== this.inspKey) {
        // rebuild only when the set of message types changes (keeps the list clickable)
        this.inspKey = key;
        list.innerHTML = entries.map(([n]) => `<li data-name="${n}"><span>${n}</span><span class="hz"></span></li>`).join('');
      }
      for (const li of list.children) {
        const v = link.inspector.get(li.dataset.name);
        li.classList.toggle('sel', li.dataset.name === this.selectedMsg);
        li.querySelector('.hz').textContent = `${v.hz.toFixed(1)} Hz`;
      }
      const sel = this.selectedMsg && link.inspector.get(this.selectedMsg);
      if (sel) {
        const obj = Object.fromEntries(Object.entries(sel.last).filter(([k]) => !k.startsWith('_')));
        detail.textContent = `${this.selectedMsg}  (#${sel.last._id}, sys ${sel.sys} comp ${sel.comp}, ${sel.count} received)\n\n` +
          Object.entries(obj).map(([k, v]) => `${k.padEnd(24)} ${Array.isArray(v) ? v.map((x) => (typeof x === 'number' ? +x.toFixed(4) : x)).join(', ') : typeof v === 'number' ? +v.toFixed(6) : v}`).join('\n');
      } else detail.textContent = 'Select a message on the left to see its fields.';
    } else {
      $('inspSrc').textContent = '— simulation (internal)';
      if (this.inspKey !== 'internal') { this.inspKey = 'internal'; list.innerHTML = '<li class="sel"><span>VEHICLE_STATE</span><span>live</span></li>'; }
      const show = { ...t, battery: undefined, rates: undefined, thrustDir: undefined };
      delete show.battery; delete show.rates; delete show.thrustDir;
      const fmt = (v) => (typeof v === 'number' ? +v.toFixed(5) : Array.isArray(v) ? v.map((x) => +Number(x).toFixed(3)).join(', ') : typeof v === 'object' && v ? JSON.stringify(v) : v);
      detail.textContent = Object.entries(show).map(([k, v]) => `${k.padEnd(18)} ${fmt(v)}`).join('\n') +
        `\n\nbattery            ${t.battery.voltage.toFixed(2)} V, ${t.battery.current.toFixed(1)} A, ${(t.battery.remaining * 100).toFixed(0)} %`;
    }
  }

  log(text, severity) {
    const el = $('consoleLog');
    const d = document.createElement('div');
    d.className = severity <= 3 ? 'bad' : severity === 4 ? 'warn' : '';
    d.textContent = `${new Date().toLocaleTimeString()}  ${text}`;
    el.appendChild(d);
    while (el.children.length > 400) el.firstChild.remove();
    el.scrollTop = el.scrollHeight;
  }

  // ================================================================== setup
  initSetup() {
    const input = this.app.input;
    $('inSource').value = input.preferred;
    $('inSource').onchange = (e) => { input.preferred = e.target.value; input.save(); };
    const fns = ['roll', 'pitch', 'throttle', 'yaw', 'mode', 'arm'];
    const chOpts = (sel, allowNone) => (allowNone ? '<option value="0">— none —</option>' : '') + Array.from({ length: 18 }, (_, i) => `<option value="${i + 1}" ${sel === i + 1 ? 'selected' : ''}>CH ${i + 1}</option>`).join('');
    $('rcMap').innerHTML = fns.map((f) => {
      const m = input.rcMap[f];
      const rev = f === 'mode' || f === 'arm' ? '<span></span>' : `<label><input type="checkbox" data-rev="${f}" ${m.rev ? 'checked' : ''}/> reverse</label>`;
      return `<span>${f === 'mode' ? 'Mode switch' : f === 'arm' ? 'Arm switch' : f[0].toUpperCase() + f.slice(1)}</span><select data-rc="${f}">${chOpts(m.ch, f === 'mode' || f === 'arm')}</select>${rev}`;
    }).join('');
    $('rcMap').onchange = (e) => {
      const f = e.target.dataset.rc || e.target.dataset.rev;
      if (!f) return;
      input.rcMap[f] = { ...input.rcMap[f] };
      if (e.target.dataset.rc) input.rcMap[f].ch = Number(e.target.value);
      else input.rcMap[f].rev = e.target.checked;
      input.save();
    };
    let cal = null;
    $('rcCal').onclick = () => { cal = { min: 2100, max: 900 }; this.app.log('RC calibration: move all sticks to their extremes, then press Done'); };
    $('rcCalDone').onclick = () => {
      if (cal && cal.max > cal.min) { input.rcMap.min = cal.min; input.rcMap.max = cal.max; input.rcMap.trim = Math.round((cal.min + cal.max) / 2); input.save(); this.app.log(`RC calibrated: ${cal.min}–${cal.max} µs`); }
      cal = null;
    };
    this.rcCal = () => cal;

    this.renderPadMap();
    input.onGamepad = () => this.renderPadMap();
  }

  renderPadMap() {
    const input = this.app.input;
    const gp = input.gamepad();
    const map = input.padMap;
    if (!gp || !map) { $('padMap').innerHTML = '<span class="hint">Connect a gamepad or USB RC transmitter and move a stick.</span>'; return; }
    const axOpts = (sel) => gp.axes.map((_, i) => `<option value="${i}" ${sel === i ? 'selected' : ''}>Axis ${i}</option>`).join('');
    $('padMap').innerHTML = ['roll', 'pitch', 'throttle', 'yaw'].map((f) => `<span>${f}</span><select data-ax="${f}">${axOpts(map[f].axis)}</select><label><input type="checkbox" data-axrev="${f}" ${map[f].rev ? 'checked' : ''}/> reverse</label>`).join('')
      + `<span class="hint" style="grid-column: span 3">${gp.id}</span>`;
    $('padMap').onchange = (e) => {
      const f = e.target.dataset.ax || e.target.dataset.axrev;
      if (!f) return;
      input.padMap[f] = { ...input.padMap[f] };
      if (e.target.dataset.ax) input.padMap[f].axis = Number(e.target.value);
      else input.padMap[f].rev = e.target.checked;
      input.save();
    };
  }

  updateSetup() {
    const input = this.app.input;
    const rc = input.rc;
    const cal = this.rcCal();
    const bars = [];
    for (let i = 1; i <= 8; i++) {
      const v = rc ? rc[`chan${i}_raw`] : 0;
      if (cal && v && v !== 65535) { cal.min = Math.min(cal.min, v); cal.max = Math.max(cal.max, v); }
      const pct = v && v !== 65535 ? Math.max(0, Math.min(100, ((v - 900) / 1200) * 100)) : 0;
      bars.push(`<div class="bar"><i style="width:${pct}%"></i><span>CH${i} ${v && v !== 65535 ? v : '—'}</span></div>`);
    }
    $('rcBars').innerHTML = bars.join('');
    const gp = input.gamepad();
    $('padBars').innerHTML = gp ? gp.axes.map((a, i) => `<div class="bar"><i style="width:${((a + 1) / 2) * 100}%"></i><span>Axis ${i} ${a.toFixed(2)}</span></div>`).join('') : '';
  }
}
