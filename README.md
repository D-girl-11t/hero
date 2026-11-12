# Hero

Hero - A Hybrid AI Powered Personal Robotic Companion

**[▶ Interactive demo](https://d-girl-11t.github.io/hero/)**: drive Hero, switch on follow mode,
and watch every command travel from Linux on the Cortex-A55 through shared memory and the MU1
mailbox to the Cortex-M33, with the camera view, VFH path planning and the PWM signals.
(The demo is a browser model of the design, not a live feed from the robot.)

A two-wheel robot on the **NXP FRDM-IMX93**. Linux on the **Cortex-A55** handles
the camera, the NPU, speech and the web app. The motors are driven by the
**Cortex-M33** running FreeRTOS, which gets its commands from Linux over **RPMsg**.

```
Cortex-A55 · Linux                                    Cortex-M33 · FreeRTOS
┌──────────────────────────────┐                      ┌────────────────────────────┐
│ hero.py  web app :8080       │                      │ rpmsg_task  endpoint 30    │
│  ├ manual drive / talk       │  "drv 45 60"         │ control_task  every 10 ms  │
│  ├ follow me (Vision class,  │ ───────────────────► │  ├ ramp + safe reversal    │
│  │   follow.py, vfh.py)      │  shared DDR vrings   │  ├ 500 ms watchdog         │
│  └ collision guard           │  + MU1 doorbell      │  └ TPM3/TPM4 PWM, GPIO2    │
│ motor_link.py → /dev/ttyRPMSG30 ◄────────────────── │ "ok 45 60", "st…", "wd"    │
└──────────────────────────────┘                      └─────────────┬──────────────┘
USB camera → V4L2 → Ethos-U65 NPU (SSD-MobileNet)                   ▼
                                                      L298N → left and right motors
```

## What's here

| Path | What it does |
|---|---|
| `firmware/m33/` | M33 firmware: protocol, motor control, PWM/GPIO, FreeRTOS + RPMsg-Lite. [README](firmware/m33/README.md) |
| `linux/hero/hero.py` | The app running on Hero: web UI and API, drive with speed, talk (rules + local LLM), Vosk voice, camera stream, photos, NPU detection, guard and follow |
| `linux/hero/motor_link.py` | Motor backends: RPMsg to the M33 (with 200 ms keepalive), or Linux GPIO as a fallback |
| `linux/hero/vfh.py` | VFH obstacle avoidance: blocked/free angle sectors from detection boxes |
| `linux/hero/follow.py` | Picks and keeps one person, steers to them around obstacles; collision guard |
| `linux/systemd/` | `hero-m33.service` (loads the firmware via remoteproc), `hero.service` (the app) |
| `dts/` | Device tree patch: motor pins and TPM3/TPM4 go to the M33 |
| `docs/protocol.md` | The Linux ↔ M33 message format and timing rules |
| `docs/demo/` | Interactive browser model of the system |

## Wiring

| L298N | Header pin | i.MX93 pad | Driven by the M33 as |
|---|---|---|---|
| IN1 | 3 | GPIO_IO02 | GPIO2_IO02 (left reverse) |
| IN2 | 5 | GPIO_IO03 | GPIO2_IO03 (left forward) |
| IN3 | 7 | GPIO_IO04 | GPIO2_IO04 (right reverse) |
| IN4 | 8 | GPIO_IO14 | GPIO2_IO14 (right forward) |
| ENA | 32 | GPIO_IO12 | TPM3_CH2, 20 kHz PWM (left speed) |
| ENB | 33 | GPIO_IO13 | TPM4_CH2, 20 kHz PWM (right speed) |

Remove the ENA/ENB jumpers on the L298N before wiring pins 32/33. Those pads also
feed the board's RGB LED, so it glows with motor speed.

## Run the tests (no board needed)

```sh
make test
```

This builds the firmware's protocol and control code for the PC and runs its
35 checks, then runs the 22 Linux tests. The end-to-end tests start `fake_m33`,
the firmware logic on a pseudo-terminal, and drive it through the real app and
HTTP API: speed changes, timed moves, text commands, VFH follow around an obstacle, guard stops, and the watchdog stop.

Try the app on a PC against the fake M33:

```sh
make -C firmware/m33/test fake_m33
./firmware/m33/test/fake_m33 --pty -v            # prints a /dev/pts/N path
HERO_MOTOR_BACKEND=rpmsg HERO_RPMSG_DEV=/dev/pts/N python3 linux/hero/hero.py
# open http://localhost:8080
```

## Put it on the board

1. Device tree: `git apply dts/0001-hero-give-motor-pins-and-tpm3-tpm4-to-cm33.patch`
   in the linux-imx tree, rebuild `imx93-11x11-frdm.dtb`, copy it to the boot partition.
2. Firmware: build it with the MCUXpresso SDK ([steps](firmware/m33/README.md#build-for-the-board)).
3. Install: `./linux/install.sh root@192.168.1.230 path/to/hero_m33.elf`
4. Open the app: `ssh -f -N -L 8080:localhost:8080 root@192.168.1.230`, then http://localhost:8080

Without the M33 firmware the app falls back to Linux GPIO (full speed only, no watchdog).

## Using the app

- **Drive**: arrow buttons or keyboard arrows, a speed slider (20–100 %) and a run time. Space stops.
- **Talk to Hero**: "forward 3 seconds", "turn left", "speed 80", "follow me", "stop following", "status".
- **Follow me**: Hero follows the person it sees, keeps that person when others walk by,
  steers around obstacles, and stops about 0.8 m away.
- **Motor controller**: whether the RPMsg link to the M33 is up, firmware version,
  the duty on each motor, and how many times the watchdog stopped the robot.
- **Camera & path**: the 15 direction sectors (free or blocked), where the person is, and the path chosen.
