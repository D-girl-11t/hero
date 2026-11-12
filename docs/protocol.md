# Linux ↔ M33 motor protocol

Transport: RPMsg over shared DDR, with the MU1 mailbox as the doorbell.
The M33 announces endpoint **30** as `rpmsg-virtual-tty-channel`; Linux's
`imx_rpmsg_tty` driver binds it and creates **`/dev/ttyRPMSG30`**.

Messages are ASCII lines ending in `\n`, at most 63 characters. One RPMsg
message can hold part of a line or several lines; both sides reassemble lines.

## Commands (Linux → M33)

| Line | Meaning | Reply |
|---|---|---|
| `drv <L> <R>` | Target duty per side, −100..100 % (negative = reverse) | `ok <L> <R>` |
| `stop` | Both targets to 0 | `ok 0 0` |
| `ping` | Version check | `pong hero-m33 1.0.0` |
| `st` | Status | `st <L> <R> <watchdog_trips> <uptime_ms>` (duty on the pins now) |

Anything else gets `err <reason>`, for example `err drv needs two values -100..100`.

## Events (M33 → Linux)

| Line | Meaning |
|---|---|
| `wd` | No `drv`/`stop` for 500 ms while moving: the M33 stopped both motors |

## Timing rules

- While moving, Linux re-sends the current `drv` every 200 ms. A moving robot
  only stays moving while Linux keeps asking.
- While stopped, `ping`/`st` are enough; the watchdog only arms when a target is non-zero.
- The M33 ramps duty by at most 4 % every 10 ms (0 → 100 % in 250 ms), and
  holds a motor at 0 for 100 ms before reversing it.
- After `wd`, Linux treats the robot as stopped and does not resume on its own.
