/*
 * Motor control logic that runs on the M33: slew limiting, safe reversal and
 * the command watchdog. Pure C (no SDK), driven by a fixed-rate tick, so the
 * same code runs in the firmware and in the host tests.
 */
#ifndef HERO_CONTROL_H
#define HERO_CONTROL_H

#include <stdint.h>

#define CTRL_TICK_MS        10      /* control loop period */
#define CTRL_SLEW_PER_TICK  4       /* max duty change per tick: 0 -> 100 % in 250 ms */
#define CTRL_COAST_MS       100     /* hold at 0 before changing direction */
#define CTRL_WATCHDOG_MS    500     /* no command for this long while moving -> stop */

typedef struct {
    int8_t target;          /* requested duty, -100..100 */
    int8_t applied;         /* duty on the pins now */
    uint16_t coast_left_ms; /* time still to wait at 0 before reversing */
} motor_ctrl_t;

typedef struct {
    motor_ctrl_t left, right;
    uint32_t now_ms;
    uint32_t last_cmd_ms;
    uint32_t watchdog_trips;
    uint8_t tripped;        /* set by the tick that tripped; caller clears after reporting */
} control_t;

void control_init(control_t *c);

/* A valid drive/stop command arrived. */
void control_command(control_t *c, int8_t left, int8_t right);

/* Any valid line (ping, st) counts as a keepalive only while stopped:
 * a moving robot needs fresh drive commands. */
void control_keepalive(control_t *c);

/* Advance CTRL_TICK_MS. Returns 1 when the applied duty changed. */
int control_tick(control_t *c);

#endif
