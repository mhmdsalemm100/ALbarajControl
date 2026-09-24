# Dronee MAVLink bridge (optional)

In Chrome and Edge the simulator can talk to the Pixhawk directly over USB (Web Serial), so **most users don't need this bridge.** Use it when you want to:

* connect the web simulator to **PX4 SITL / ArduPilot SITL** (UDP),
* use a Pixhawk from **Firefox or Safari** (no Web Serial),
* run **real QGroundControl at the same time** as the web simulator.

```bash
cd bridge
npm install                 # installs "ws"; add "serialport" for serial mode
node server.js              # UDP 14550  <->  ws://localhost:14560
```

In the simulator, click **Connect Pixhawk**, set *Transport* to **WebSocket bridge**, and keep the URL `ws://localhost:14560`.

| Example | Command |
|---|---|
| PX4 SITL (`make px4_sitl none_iris`) | `node server.js --udp 14550` |
| Pixhawk on a serial port | `npm i serialport && node server.js --serial /dev/ttyACM0 --baud 115200` (Windows: `--serial COM5`) |
| Also mirror to QGroundControl | add `--qgc 127.0.0.1:14551`, then in QGC add a UDP comm link that listens on 14551 |

## Testing without hardware

`node ../tools/mock-pixhawk.mjs` starts a fake PX4 in HITL mode on the same WebSocket port. It answers arm, takeoff and mission upload, and it flies the simulated drone to 5 m using the simulated barometer.
