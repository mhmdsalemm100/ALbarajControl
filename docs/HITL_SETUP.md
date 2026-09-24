# Connecting your Pixhawk, receiver and remote controller

The simulator has three ways to use real hardware. All of them start the same way:

1. Plug the **RC receiver** into the Pixhawk (**RC IN** / **PPM** / **SBUS** port) and bind it to your transmitter.
2. Plug the **Pixhawk into the computer with USB**.
3. **Close QGroundControl / Mission Planner.** Only one program can open the USB port at a time.
4. Open the simulator in **Google Chrome or Microsoft Edge** (desktop). These browsers have *Web Serial*, which lets a web page talk to USB devices. Firefox and Safari don't have it, so use the [bridge](../bridge/README.md) with them.
5. Click **Connect Pixhawk**, pick a mode, click **Connect**, then pick the Pixhawk's port in the browser pop-up. It is usually called *PX4 FMU*, *ArduPilot*, *CubeOrange* or *STM32 Virtual ComPort*.

> ⚠️ **Take the propellers off** any real drone the Pixhawk is mounted on before you connect it to a simulator.

---

## Mode 1: RC controller (easiest, any firmware)

Your transmitter flies the **simulated** F450 through the receiver and the Pixhawk. This works with PX4 and ArduPilot, and needs no parameter changes.

* The page asks the Pixhawk to stream `RC_CHANNELS` at 20 Hz. The **Setup** tab shows the live channel bars.
* The default channel map is **AETR**: CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw. You can change the map and set reverses in **Setup → RC channel mapping**. Press **Calibrate**, move every stick to its limits, then press **Done**.
* **CH5 3-position switch** → Stabilized / Altitude / Position.
* **Arm:** hold throttle down + yaw right for 1 s. **Disarm:** hold throttle down + yaw left. You can also give an arm switch its own channel.
* The simulator's autopilot (PX4-style cascaded controller) does the stabilization, missions and RTL.

## Mode 2: Hardware-in-the-loop (PX4)

The **real PX4 flight stack on your Pixhawk flies the simulated drone**:

```
 Transmitter ──radio──► Receiver ──► Pixhawk (PX4 estimator + controllers)
                                          │  HIL_ACTUATOR_CONTROLS (motor outputs)
                                          ▼
                     Web page: F450 physics  ──► 3D view
                                          │  HIL_SENSOR 250 Hz (IMU, mag, baro)
                                          │  HIL_GPS 10 Hz
                                          ▼
                                       Pixhawk
```

### One-time Pixhawk setup (in QGroundControl)

1. Flash current **PX4** firmware (v1.13 or newer): *Vehicle Setup → Firmware*.
2. *Vehicle Setup → Airframe* → **Simulation (HIL) → HIL Quadcopter X** (`SYS_AUTOSTART = 1001`). Apply and restart.
3. *Vehicle Setup → Safety* → **HITL Enabled** (or set parameter `SYS_HITL = 1`). Reboot.
4. Calibrate the radio (*Vehicle Setup → Radio*) and set flight modes (*Flight Modes*).
5. Optional, to fly with a keyboard or gamepad instead of a transmitter: set `COM_RC_IN_MODE` to allow joystick input. The page sends `MANUAL_CONTROL` when no RC is detected.
6. **Close QGroundControl.**

### Flying

1. Connect in the simulator with **Hardware-in-the-loop (HITL, PX4)**.
2. Wait until PX4 says it is ready (messages appear at the top, and in *Analyze → Messages*). The EKF needs a few seconds of GPS.
3. Arm with the transmitter (or the **Arm** button), switch to Position mode, and take off. You can also use **Takeoff**, or plan a mission in **Plan**, press **Upload** and then **Start mission**. The mission is uploaded to PX4 with the standard MAVLink mission protocol.
4. Keep the browser tab **visible** while flying. Browsers slow down timers in background tabs, and PX4 would lose its sensor stream.

### Troubleshooting

| Symptom | Fix |
|---|---|
| "No heartbeat within 6 s" | Wrong port, the port is used by another program, or a telemetry radio with a different baud. Try 57600 for SiK radios. |
| No `HIL_ACTUATOR_CONTROLS` in *Analyze* | `SYS_HITL` is not 1, or the airframe is not a HIL airframe. Reboot after changing it. |
| "Preflight Fail: … mag / compass inconsistent" | The simulated magnetic field matches the configured **home** (KFUPM, Dhahran by default). Set your real location in *Setup → Home location*, or relax `EKF2_MAG_CHECK` / `COM_ARM_MAG_STR`. |
| Drone drifts or toilet-bowls | Wait for the EKF to settle before takeoff. Keep sensor noise at 1×. |
| Motors never spin | Check that *Analyze → MAVLink inspector* shows `HIL_ACTUATOR_CONTROLS`, and that the vehicle is armed (red **Flying/Armed** pill). |

## Mode 3: Telemetry only

This shows the live attitude and data of the real Pixhawk in the 3D view and the ground station. It's useful for bench tests: tilt the board and watch the F450 tilt. Mission upload and download, mode changes, arming and the MAVLink inspector all work against the real autopilot.

## ArduPilot

**RC controller** mode and **Telemetry only** mode work with ArduPilot. ArduPilot doesn't support `HIL_SENSOR`-based HITL. To run the ArduPilot flight code, use ArduPilot SITL together with the [bridge](../bridge/README.md).

## Using a USB transmitter or gamepad directly

Many transmitters (RadioMaster, FrSky, Jumper, TBS…) show up as a **USB joystick** when you plug them in. You can fly the simulator with them without a Pixhawk. Choose *Setup → Input source → Gamepad / USB RC transmitter* and map the axes. Xbox and PlayStation controllers work too: A = arm, B = land, X = takeoff, Y = return.
