#!/usr/bin/env node
// Dronee MAVLink bridge.
//
// The browser talks WebSocket; flight software talks UDP or serial. This
// bridge relays raw MAVLink bytes between them, so you can:
//   * connect the web simulator to PX4 SITL / ArduPilot SITL (UDP)
//   * use a Pixhawk from browsers without Web Serial (Firefox, Safari)
//   * run real QGroundControl at the same time as the web simulator
//
// Usage:
//   node server.js                                  # UDP 14550 <-> ws://localhost:14560
//   node server.js --serial /dev/ttyACM0 --baud 115200
//   node server.js --serial COM5 --qgc 127.0.0.1:14551   # also mirror to QGroundControl
//
// Options:
//   --ws <port>          WebSocket port for the browser (default 14560)
//   --udp <port>         local UDP port to listen on (default 14550)
//   --udp-remote h:p     where to send UDP (default: last sender)
//   --serial <path>      use a serial port instead of UDP (needs `npm i serialport`)
//   --baud <rate>        serial baud rate (default 115200)
//   --qgc h:p            mirror all vehicle traffic to QGroundControl over UDP

import dgram from 'node:dgram';
import { WebSocketServer } from 'ws';

const args = process.argv.slice(2);
const opt = (name, def) => { const i = args.indexOf(`--${name}`); return i >= 0 ? args[i + 1] : def; };
const wsPort = Number(opt('ws', 14560));
const udpPort = Number(opt('udp', 14550));
const serialPath = opt('serial', null);
const baud = Number(opt('baud', 115200));
const qgc = opt('qgc', null);
const parseHostPort = (s) => { const [h, p] = s.split(':'); return { host: h || '127.0.0.1', port: Number(p) }; };
let udpRemote = opt('udp-remote', null) ? parseHostPort(opt('udp-remote')) : null;

const clients = new Set();
const toBrowsers = (buf) => { for (const c of clients) if (c.readyState === 1) c.send(buf); };
let toVehicle = () => {};
let stats = { up: 0, down: 0 };

// ---------------------------------------------------------------- vehicle side
if (serialPath) {
  let SerialPort;
  try { ({ SerialPort } = await import('serialport')); } catch {
    console.error('The serialport package is not installed. Run:  npm install serialport');
    process.exit(1);
  }
  const port = new SerialPort({ path: serialPath, baudRate: baud });
  port.on('data', (d) => { stats.down += d.length; toBrowsers(d); toQgc(d); });
  port.on('error', (e) => console.error('Serial error:', e.message));
  port.on('open', () => console.log(`Serial ${serialPath} @ ${baud} open`));
  toVehicle = (buf) => port.write(buf);
} else {
  const udp = dgram.createSocket('udp4');
  udp.on('message', (msg, rinfo) => {
    if (qgcAddr && rinfo.port === qgcAddr.port && rinfo.address === qgcAddr.host) return;
    if (!udpRemote || !opt('udp-remote')) udpRemote = { host: rinfo.address, port: rinfo.port };
    stats.down += msg.length;
    toBrowsers(msg);
    toQgc(msg);
  });
  udp.bind(udpPort, () => console.log(`UDP listening on ${udpPort} (PX4 SITL sends here by default)`));
  toVehicle = (buf) => { if (udpRemote) udp.send(buf, udpRemote.port, udpRemote.host); };
}

// ---------------------------------------------------------------- optional QGroundControl mirror
const qgcAddr = qgc ? parseHostPort(qgc) : null;
const qgcSock = qgcAddr ? dgram.createSocket('udp4') : null;
function toQgc(buf) { if (qgcSock) qgcSock.send(buf, qgcAddr.port, qgcAddr.host); }
if (qgcSock) {
  qgcSock.on('message', (msg) => toVehicle(msg)); // commands from QGC go to the vehicle
  qgcSock.bind(0, () => console.log(`Mirroring to QGroundControl at ${qgcAddr.host}:${qgcAddr.port} (add a UDP link in QGC)`));
}

// ---------------------------------------------------------------- browser side
const wss = new WebSocketServer({ port: wsPort });
wss.on('connection', (ws, req) => {
  clients.add(ws);
  console.log(`Browser connected from ${req.socket.remoteAddress}`);
  ws.on('message', (data) => { const b = Buffer.from(data); stats.up += b.length; toVehicle(b); });
  ws.on('close', () => { clients.delete(ws); console.log('Browser disconnected'); });
});
console.log(`WebSocket server on ws://localhost:${wsPort}  →  in the simulator: Connect Pixhawk › Transport: WebSocket bridge`);
setInterval(() => {
  if (stats.up || stats.down) console.log(`traffic: browser→vehicle ${stats.up} B/s, vehicle→browser ${stats.down} B/s, ${clients.size} browser(s)`);
  stats = { up: 0, down: 0 };
}, 5000);
