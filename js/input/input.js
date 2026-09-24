// Pilot input: keyboard, gamepad / USB RC transmitter (joystick mode),
// on-screen touch sticks and the RC receiver attached to a Pixhawk
// (RC_CHANNELS over MAVLink).
//
// Output convention (same as a Mode-2 transmitter):
//   roll  -1..1  (+ = right)
//   pitch -1..1  (+ = stick pulled back = nose up)
//   yaw   -1..1  (+ = right)
//   throttle 0..1

import { clamp } from '../core/math.js';

const STORAGE_KEY = 'dronee.input.v1';

export const DEFAULT_RC_MAP = {
  roll: { ch: 1, rev: false },
  pitch: { ch: 2, rev: false },
  throttle: { ch: 3, rev: false },
  yaw: { ch: 4, rev: false },
  mode: { ch: 5 },
  arm: { ch: 0 }, // 0 = disabled (use stick gesture / GCS)
  min: 1000,
  max: 2000,
  trim: 1500,
};

export const DEFAULT_PAD_MAP = {
  roll: { axis: 2, rev: false },
  pitch: { axis: 3, rev: false },
  throttle: { axis: 1, rev: true },
  yaw: { axis: 0, rev: false },
};
const RC_PAD_MAP = { // typical RC transmitter in USB joystick mode (AETR)
  roll: { axis: 0, rev: false },
  pitch: { axis: 1, rev: true },
  throttle: { axis: 2, rev: false },
  yaw: { axis: 3, rev: false },
};

export class InputManager {
  constructor() {
    this.keys = new Set();
    this.source = 'keyboard';
    this.preferred = 'auto';
    this.kbThrottle = 0;
    this.centeredThrottle = true; // altitude/position modes: throttle springs to 50 %
    this.sticks = { roll: 0, pitch: 0, yaw: 0, throttle: 0.5 };
    this.rc = null; // last RC_CHANNELS
    this.rcTime = 0;
    this.rcMap = { ...DEFAULT_RC_MAP };
    this.padMap = null;
    this.padIndex = null;
    this.touch = { left: { x: 0, y: 0, active: false }, right: { x: 0, y: 0, active: false }, used: false };
    this.modeSwitch = null; // 0,1,2 position of the RC mode switch
    this.armSwitch = null;
    this.load();

    window.addEventListener('keydown', (e) => {
      if (e.target && ['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName)) return;
      this.keys.add(e.code);
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(e.code)) e.preventDefault();
    });
    window.addEventListener('keyup', (e) => this.keys.delete(e.code));
    window.addEventListener('blur', () => this.keys.clear());
    window.addEventListener('gamepadconnected', (e) => {
      this.padIndex = e.gamepad.index;
      if (!this.padMap) this.padMap = e.gamepad.mapping === 'standard' ? { ...DEFAULT_PAD_MAP } : { ...RC_PAD_MAP };
      this.onGamepad?.(e.gamepad, true);
    });
    window.addEventListener('gamepaddisconnected', (e) => {
      if (this.padIndex === e.gamepad.index) this.padIndex = null;
      this.onGamepad?.(e.gamepad, false);
    });
  }

  load() {
    try {
      const s = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}');
      if (s.rcMap) this.rcMap = { ...DEFAULT_RC_MAP, ...s.rcMap };
      if (s.padMap) this.padMap = s.padMap;
      if (s.preferred) this.preferred = s.preferred;
    } catch { /* storage unavailable */ }
  }

  save() {
    try { localStorage.setItem(STORAGE_KEY, JSON.stringify({ rcMap: this.rcMap, padMap: this.padMap, preferred: this.preferred })); } catch { /* ignore */ }
  }

  setRcChannels(msg) {
    this.rc = msg;
    this.rcTime = performance.now();
  }

  get rcActive() { return this.rc && performance.now() - this.rcTime < 1000 && this.rc.chancount > 0; }

  gamepad() {
    if (this.padIndex === null || !navigator.getGamepads) return null;
    return navigator.getGamepads()[this.padIndex] || null;
  }

  /** Normalised RC channel value in -1..1 (or 0..1 for throttle). */
  rcValue(fn) {
    const m = this.rcMap[fn];
    const pwm = this.rc?.[`chan${m.ch}_raw`];
    if (!pwm || pwm === 65535) return fn === 'throttle' ? 0 : 0;
    const { min, max, trim } = this.rcMap;
    if (fn === 'throttle') {
      let v = (pwm - min) / (max - min);
      if (m.rev) v = 1 - v;
      return clamp(v, 0, 1);
    }
    let v = pwm >= trim ? (pwm - trim) / (max - trim) : (pwm - trim) / (trim - min);
    if (m.rev) v = -v;
    return clamp(v, -1, 1);
  }

  pickSource() {
    if (this.preferred !== 'auto') {
      if (this.preferred === 'rc' && !this.rcActive) return 'keyboard';
      if (this.preferred === 'gamepad' && !this.gamepad()) return 'keyboard';
      return this.preferred;
    }
    if (this.rcActive) return 'rc';
    if (this.gamepad()) return 'gamepad';
    if (this.touch.used) return 'touch';
    return 'keyboard';
  }

  update(dt) {
    this.source = this.pickSource();
    const s = this.sticks;
    switch (this.source) {
      case 'rc': {
        s.roll = this.rcValue('roll');
        s.pitch = this.rcValue('pitch');
        s.yaw = this.rcValue('yaw');
        s.throttle = this.rcValue('throttle');
        const modeCh = this.rcMap.mode.ch;
        const mpwm = modeCh ? this.rc[`chan${modeCh}_raw`] : 0;
        this.modeSwitch = mpwm && mpwm !== 65535 ? (mpwm < 1300 ? 0 : mpwm < 1700 ? 1 : 2) : null;
        const armCh = this.rcMap.arm.ch;
        const apwm = armCh ? this.rc[`chan${armCh}_raw`] : 0;
        this.armSwitch = apwm && apwm !== 65535 ? apwm > 1600 : null;
        break;
      }
      case 'gamepad': {
        const gp = this.gamepad();
        const map = this.padMap || DEFAULT_PAD_MAP;
        const ax = (m) => { const v = gp.axes[m.axis] ?? 0; return m.rev ? -v : v; };
        s.roll = clamp(ax(map.roll), -1, 1);
        s.pitch = clamp(ax(map.pitch), -1, 1);
        s.yaw = clamp(ax(map.yaw), -1, 1);
        s.throttle = clamp((ax(map.throttle) + 1) / 2, 0, 1);
        this.modeSwitch = null;
        this.armSwitch = null;
        // standard pads: A = arm toggle handled by app via buttons
        this.padButtons = gp.buttons.map((b) => b.pressed);
        break;
      }
      case 'touch': {
        const t = this.touch;
        s.roll = t.right.x;
        s.pitch = t.right.y;
        s.yaw = t.left.x;
        s.throttle = this.centeredThrottle ? 0.5 - t.left.y * 0.5 : clamp(this.kbThrottle - t.left.y * dt * 0.8, 0, 1);
        if (!this.centeredThrottle) this.kbThrottle = s.throttle;
        break;
      }
      default: {
        const k = this.keys;
        const axis = (neg, pos) => (k.has(pos) ? 1 : 0) - (k.has(neg) ? 1 : 0);
        const target = {
          roll: axis('ArrowLeft', 'ArrowRight') || axis('KeyJ', 'KeyL'),
          pitch: axis('ArrowUp', 'ArrowDown') || axis('KeyI', 'KeyK'),
          yaw: axis('KeyA', 'KeyD'),
        };
        const shift = k.has('ShiftLeft') || k.has('ShiftRight');
        const gain = shift ? 1 : 0.6;
        // smooth keyboard sticks so the vehicle does not jerk
        const rate = 1 - Math.exp(-dt * 7);
        for (const a of ['roll', 'pitch', 'yaw']) s[a] += (target[a] * gain - s[a]) * rate;
        const thr = axis('KeyS', 'KeyW');
        if (this.centeredThrottle) {
          s.throttle += (0.5 + thr * 0.5 * gain - s.throttle) * rate;
          this.kbThrottle = s.throttle;
        } else {
          this.kbThrottle = clamp(this.kbThrottle + thr * dt * 0.45, 0, 1);
          s.throttle = this.kbThrottle;
        }
        this.modeSwitch = null;
        this.armSwitch = null;
      }
    }
    return s;
  }

  /** Switch keyboard throttle behaviour (centred for alt/pos hold modes). */
  setThrottleCentered(centered) {
    if (this.centeredThrottle === centered) return;
    this.centeredThrottle = centered;
    this.kbThrottle = centered ? 0.5 : 0;
    if (this.source === 'keyboard' || this.source === 'touch') this.sticks.throttle = this.kbThrottle;
  }

  /** Attach draggable virtual sticks for touch screens. */
  attachTouchSticks(leftEl, rightEl) {
    const bind = (el, side) => {
      const knob = el.querySelector('.knob');
      let id = null;
      const setFrom = (e) => {
        const r = el.getBoundingClientRect();
        const x = clamp(((e.clientX - r.left) / r.width) * 2 - 1, -1, 1);
        const y = clamp(((e.clientY - r.top) / r.height) * 2 - 1, -1, 1);
        this.touch[side] = { x, y, active: true };
        knob.style.transform = `translate(${x * 50}%, ${y * 50}%)`;
      };
      el.addEventListener('pointerdown', (e) => { id = e.pointerId; el.setPointerCapture(id); this.touch.used = true; setFrom(e); });
      el.addEventListener('pointermove', (e) => { if (e.pointerId === id) setFrom(e); });
      const end = (e) => {
        if (e.pointerId !== id) return;
        id = null;
        this.touch[side] = { x: 0, y: 0, active: false };
        knob.style.transform = 'translate(0,0)';
      };
      el.addEventListener('pointerup', end);
      el.addEventListener('pointercancel', end);
    };
    bind(leftEl, 'left');
    bind(rightEl, 'right');
  }
}
