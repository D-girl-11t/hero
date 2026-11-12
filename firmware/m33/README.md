# Hero M33 motor firmware

FreeRTOS + RPMsg-Lite firmware for the i.MX93's Cortex-M33. It owns the L298N
motor driver: PWM speed on ENA/ENB, direction on IN1–IN4, speed ramping, a
pause at zero before reversing, and a 500 ms command watchdog.

| File | What it is |
|---|---|
| `src/main.c` | Two tasks: `rpmsg_task` (commands in, replies out) and `control_task` (10 ms loop) |
| `src/protocol.c/.h` | The text protocol: parsing, line assembly, replies. No SDK calls |
| `src/control.c/.h` | Slew limit, safe reversal, watchdog. No SDK calls |
| `src/motor_hw.c/.h` | Pins and TPM PWM through the MCUXpresso SDK drivers |
| `src/hero_board.h` | Pin map, PWM frequency, RPMsg endpoint and name |
| `test/` | Host build: unit tests and `fake_m33`, the same logic over a pty |

## Test on a PC (no board needed)

```sh
make -C test          # builds test_firmware and fake_m33
./test/test_firmware  # 35 checks: parser, line splitting, ramp, reversal, watchdog
```

`fake_m33` runs `protocol.c` and `control.c` on the PC and talks over a
pseudo-terminal, so the Linux app can be tested against the real firmware logic:

```sh
./test/fake_m33 --pty -v      # prints e.g. /dev/pts/7
HERO_MOTOR_BACKEND=rpmsg HERO_RPMSG_DEV=/dev/pts/7 python3 ../../linux/hero/hero.py
```

## Build for the board

Needs the MCUXpresso SDK for the FRDM-IMX93 (or MCIMX93-EVK) and
`arm-none-eabi-gcc`. The firmware is built on top of the SDK's RPMsg-Lite
FreeRTOS example, which already provides the board files, `FreeRTOSConfig.h`,
the linker script and `rsc_table.c`:

1. Copy the SDK example `multicore_examples/rpmsg_lite_str_echo_rtos` (the
   `cm33` / remote variant) to a work folder.
2. Replace its `main_remote.c` with the files in `src/` and add `motor_hw.c`,
   `protocol.c` and `control.c` to the example's source list (`CMakeLists.txt`
   for SDK 2.x armgcc, or the example's `mcux_add_source` list for the newer
   west-based SDK).
3. Add the `fsl_rgpio` and `fsl_tpm` drivers to the example (they ship with the SDK).
4. Build the release target, e.g. `cd armgcc && ./build_release.sh`, and rename
   the ELF to `hero_m33.elf`.
5. Install and start it: `../../linux/install.sh root@<board> path/to/hero_m33.elf`.
   `hero-m33.service` loads it through remoteproc at boot.

Check on the board:

```sh
cat /sys/class/remoteproc/remoteproc0/state   # running
ls /dev/ttyRPMSG*                              # /dev/ttyRPMSG30
printf 'ping\n' > /dev/ttyRPMSG30; head -n1 /dev/ttyRPMSG30   # pong hero-m33 1.0.0
```

## Things to confirm on the hardware

- SDK names: the `IOMUXC_PAD_GPIO_IO12__TPM3_CH2`-style macros, the
  `kCLOCK_TPMx_ClockRoot_MuxOsc24M` constants and `RL_PLATFORM_IMX93_M33_A55_USER_LINK_ID`
  follow the i.MX93 SDK naming. Check them against your SDK version.
- Access: GPIO2, TPM3 and TPM4 sit in the wakeup domain. The default i.MX93
  boot image's TRDC setup must let the M33 use them; if a register write
  faults, grant access in the imx-boot / ELE TRDC configuration.
- Shared GPIO bank: Linux still owns the other GPIO2 pins. Output writes use
  the RGPIO set/clear registers on both sides, which don't collide.
- Start order: the firmware is loaded by Linux (remoteproc) after boot, so
  Linux's unused-clock cleanup has already run and won't stop the TPM clocks.
