#include "control.h"

static int sign(int v) { return (v > 0) - (v < 0); }

static void motor_init(motor_ctrl_t *m)
{
    m->target = 0;
    m->applied = 0;
    m->coast_left_ms = 0;
}

void control_init(control_t *c)
{
    motor_init(&c->left);
    motor_init(&c->right);
    c->now_ms = 0;
    c->last_cmd_ms = 0;
    c->watchdog_trips = 0;
    c->tripped = 0;
}

static int8_t clamp_duty(int v) { return (int8_t)(v > 100 ? 100 : v < -100 ? -100 : v); }

void control_command(control_t *c, int8_t left, int8_t right)
{
    c->left.target = clamp_duty(left);
    c->right.target = clamp_duty(right);
    c->last_cmd_ms = c->now_ms;
}

void control_keepalive(control_t *c)
{
    if (c->left.target == 0 && c->right.target == 0) c->last_cmd_ms = c->now_ms;
}

/* One motor, one tick. Never jumps straight from forward to reverse:
 * ramps to 0, waits CTRL_COAST_MS, then ramps the other way. */
static int motor_step(motor_ctrl_t *m)
{
    int a = m->applied, t = m->target;
    if (a == t) return 0;

    if (a != 0 && t != 0 && sign(a) != sign(t)) t = 0;    /* reversing: head for 0 first */

    if (a == 0 && t != 0 && m->coast_left_ms > 0) {        /* coasting at 0 */
        m->coast_left_ms = m->coast_left_ms > CTRL_TICK_MS ? m->coast_left_ms - CTRL_TICK_MS : 0;
        return 0;
    }

    int step = t - a;
    if (step > CTRL_SLEW_PER_TICK) step = CTRL_SLEW_PER_TICK;
    if (step < -CTRL_SLEW_PER_TICK) step = -CTRL_SLEW_PER_TICK;
    m->applied = (int8_t)(a + step);

    /* just reached 0 on the way to the other direction: start coasting */
    if (m->applied == 0 && m->target != 0 && sign(m->target) != sign(a))
        m->coast_left_ms = CTRL_COAST_MS;
    return 1;
}

int control_tick(control_t *c)
{
    c->now_ms += CTRL_TICK_MS;

    int moving = c->left.target != 0 || c->right.target != 0;
    if (moving && c->now_ms - c->last_cmd_ms >= CTRL_WATCHDOG_MS) {
        /* the A55 went quiet: stop now, no ramp */
        c->left.target = c->right.target = 0;
        c->left.applied = c->right.applied = 0;
        c->left.coast_left_ms = c->right.coast_left_ms = 0;
        c->watchdog_trips++;
        c->tripped = 1;
        return 1;
    }

    int changed = motor_step(&c->left);
    changed |= motor_step(&c->right);
    return changed;
}
