// Byte transports for MAVLink: Web Serial (Pixhawk over USB / telemetry
// radio, Chrome & Edge) and WebSocket (to the optional Node bridge, which
// in turn talks UDP to PX4 SITL / QGroundControl or to a serial port).

export class SerialTransport {
  static get supported() { return 'serial' in navigator; }

  constructor(baudRate = 115200) {
    this.baudRate = baudRate;
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.onData = null;
    this.onClose = null;
    this.label = 'USB serial';
    this.writeQueue = [];
    this.writing = false;
  }

  async open() {
    if (!SerialTransport.supported) throw new Error('Web Serial is not supported by this browser. Use Google Chrome or Microsoft Edge (desktop).');
    this.port = await navigator.serial.requestPort({});
    await this.port.open({ baudRate: this.baudRate, bufferSize: 65536 });
    const info = this.port.getInfo?.() || {};
    this.label = `USB ${info.usbVendorId ? info.usbVendorId.toString(16).padStart(4, '0') + ':' + info.usbProductId.toString(16).padStart(4, '0') : 'serial'} @ ${this.baudRate}`;
    this.writer = this.port.writable.getWriter();
    this.closed = false;
    this.readLoop();
    navigator.serial.addEventListener('disconnect', this._onDisconnect = (e) => { if (e.target === this.port) this.close(); });
  }

  async readLoop() {
    while (this.port && this.port.readable && !this.closed) {
      this.reader = this.port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          if (value && this.onData) this.onData(value);
        }
      } catch (err) {
        console.warn('Serial read error', err);
        if (this.closed) break;
      } finally {
        try { this.reader.releaseLock(); } catch { /* already released */ }
      }
      if (this.closed) break;
    }
    this.close();
  }

  write(bytes) {
    if (!this.writer || this.closed) return;
    this.writeQueue.push(bytes);
    if (!this.writing) this.flush();
  }

  async flush() {
    this.writing = true;
    try {
      while (this.writeQueue.length && !this.closed) {
        // coalesce small frames into one USB transfer
        let total = 0;
        for (const b of this.writeQueue) total += b.length;
        const buf = new Uint8Array(total);
        let o = 0;
        for (const b of this.writeQueue) { buf.set(b, o); o += b.length; }
        this.writeQueue = [];
        await this.writer.write(buf);
      }
    } catch (err) {
      console.warn('Serial write error', err);
    }
    this.writing = false;
  }

  async close() {
    if (this.closed) return;
    this.closed = true;
    try { await this.reader?.cancel(); } catch { /* ignore */ }
    try { this.writer?.releaseLock(); } catch { /* ignore */ }
    try { await this.port?.close(); } catch { /* ignore */ }
    if (this._onDisconnect) navigator.serial.removeEventListener('disconnect', this._onDisconnect);
    this.port = null;
    this.onClose?.();
  }
}

export class WebSocketTransport {
  constructor(url = 'ws://localhost:14560') {
    this.url = url;
    this.label = url;
    this.onData = null;
    this.onClose = null;
  }

  open() {
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(this.url);
      ws.binaryType = 'arraybuffer';
      ws.onopen = () => { this.ws = ws; resolve(); };
      ws.onerror = () => reject(new Error(`Cannot connect to ${this.url}. Is the bridge running? (see bridge/README)`));
      ws.onmessage = (e) => { if (this.onData && e.data instanceof ArrayBuffer) this.onData(new Uint8Array(e.data)); };
      ws.onclose = () => { this.ws = null; this.onClose?.(); };
    });
  }

  write(bytes) {
    if (this.ws && this.ws.readyState === 1) this.ws.send(bytes);
  }

  close() { this.ws?.close(); }
}
