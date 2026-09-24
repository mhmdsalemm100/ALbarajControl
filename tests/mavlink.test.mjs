// MAVLink codec tests against reference frames generated with pymavlink.
//   node tests/mavlink.test.mjs
import assert from 'node:assert/strict';
import { MavlinkCodec } from '../js/mavlink/mavlink.js';

const hex = (s) => Uint8Array.from(s.match(/../g).map((b) => parseInt(b, 16)));
const toHex = (u8) => Buffer.from(u8).toString('hex');

// ATTITUDE + HIL_ACTUATOR_CONTROLS as sent by a flight controller (sys 1, comp 1)
const FROM_FC = 'fd1c00000001011e0000d2040000cdcccc3dcdcc4cbe0000c03f0ad7233c0ad7a33c8fc2f53c2c53fd5100000101015d0000e8030000000000000000000000000000cdcccc3dcdcc4c3e9a99993ecdcccc3e000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000019089';
// COMMAND_LONG arm, as sent by a GCS (sys 255, comp 190, seq 0)
const ARM_CMD = 'fd20000000ffbe4c00000000803f000000000000000000000000000000000000000000000000900101019e4e';

// --- decode, fed in awkward chunks to exercise the stream parser
const rx = new MavlinkCodec();
const bytes = hex(FROM_FC);
const msgs = [];
for (let i = 0; i < bytes.length; i += 7) msgs.push(...rx.parse(bytes.subarray(i, i + 7)));
assert.equal(msgs.length, 2);
assert.equal(msgs[0]._name, 'ATTITUDE');
assert.equal(msgs[0].time_boot_ms, 1234);
assert.ok(Math.abs(msgs[0].yaw - 1.5) < 1e-6);
assert.equal(msgs[1]._name, 'HIL_ACTUATOR_CONTROLS');
assert.deepEqual(msgs[1].controls.slice(0, 4).map((x) => +x.toFixed(3)), [0.1, 0.2, 0.3, 0.4]);
assert.equal(msgs[1].mode, 1);
assert.equal(rx.stats.crcErrors, 0);
console.log('ok - decodes pymavlink frames (split across chunks)');

// --- encode must be byte-identical to pymavlink (incl. payload truncation)
const tx = new MavlinkCodec({ sysid: 255, compid: 190 });
tx.seq = 255; // next frame gets seq 0
const arm = tx.encode('COMMAND_LONG', { target_system: 1, target_component: 1, command: 400, confirmation: 0, param1: 1 });
assert.equal(toHex(arm), ARM_CMD);
console.log('ok - encodes COMMAND_LONG identical to pymavlink');

// --- round trip of HIL messages
const rt = new MavlinkCodec();
const f = tx.encode('HIL_SENSOR', { time_usec: 123456789, xacc: 0.1, zacc: -9.81, abs_pressure: 1005.5, fields_updated: 0x1fff });
const [s] = rt.parse(f);
assert.equal(s._name, 'HIL_SENSOR');
assert.equal(s.time_usec, 123456789);
assert.equal(s.fields_updated, 0x1fff);
assert.ok(Math.abs(s.zacc + 9.81) < 1e-5);
console.log('ok - HIL_SENSOR round trip');

// --- corrupted frame is rejected
const bad = Uint8Array.from(f);
bad[15] ^= 0xff;
assert.equal(new MavlinkCodec().parse(bad).length, 0);
console.log('ok - rejects CRC errors');
