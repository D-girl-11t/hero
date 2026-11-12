/* Host tests for the M33's protocol and control logic: make -C firmware/m33/test */
#include <stdio.h>
#include <string.h>

#include "../src/control.h"
#include "../src/protocol.h"

static int fails, checks;
#define CHECK(cond) do { checks++; if (!(cond)) { fails++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static cmd_t P(const char *s) { return proto_parse(s, strlen(s)); }

static void test_parse(void)
{
    cmd_t c = P("drv 60 -45");
    CHECK(c.type == CMD_DRIVE && c.left == 60 && c.right == -45);
    c = P("  drv +100 -100\r");
    CHECK(c.type == CMD_DRIVE && c.left == 100 && c.right == -100);
    CHECK(P("stop").type == CMD_STOP);
    CHECK(P("ping").type == CMD_PING);
    CHECK(P("st").type == CMD_STATUS);
    CHECK(P("").type == CMD_NONE);
    CHECK(P("   ").type == CMD_NONE);
    CHECK(P("drv 101 0").type == CMD_ERROR);
    CHECK(P("drv 50").type == CMD_ERROR);
    CHECK(P("drv a b").type == CMD_ERROR);
    CHECK(P("drv - 5").type == CMD_ERROR);
    CHECK(P("drv 10 10 10").type == CMD_ERROR);
    CHECK(P("stopp").type == CMD_ERROR);
    CHECK(P("fly").type == CMD_ERROR);
    /* never reads past len */
    c = proto_parse("drv 12 34XXXX", 9);
    CHECK(c.type == CMD_DRIVE && c.left == 12 && c.right == 34);
}

static char got[8][PROTO_MAX_LINE];
static int ngot;
static void collect(const char *line, size_t len, void *ctx)
{
    (void)ctx;
    if (ngot < 8) { memcpy(got[ngot], line, len); got[ngot][len] = 0; ngot++; }
}

static void test_lines(void)
{
    line_buf_t lb;
    proto_line_init(&lb);
    ngot = 0;
    proto_line_feed(&lb, "drv 1", 5, collect, NULL);         /* split across messages */
    proto_line_feed(&lb, "0 20\nst\npi", 10, collect, NULL);  /* joined in one message */
    proto_line_feed(&lb, "ng\n", 3, collect, NULL);
    CHECK(ngot == 3);
    CHECK(strcmp(got[0], "drv 10 20") == 0);
    CHECK(strcmp(got[1], "st") == 0);
    CHECK(strcmp(got[2], "ping") == 0);

    char longline[200];
    memset(longline, 'x', sizeof longline);
    ngot = 0;
    proto_line_feed(&lb, longline, sizeof longline, collect, NULL);
    proto_line_feed(&lb, "\nstop\n", 6, collect, NULL);
    CHECK(ngot == 1 && strcmp(got[0], "stop") == 0);          /* oversize line dropped */
}

static void test_format(void)
{
    char b[PROTO_MAX_LINE];
    CHECK(proto_fmt_ok(b, sizeof b, -40, 75) == 10 && strcmp(b, "ok -40 75\n") == 0);
    proto_fmt_status(b, sizeof b, 10, -10, 3, 123456);
    CHECK(strcmp(b, "st 10 -10 3 123456\n") == 0);
    proto_fmt_wd(b, sizeof b);
    CHECK(strcmp(b, "wd\n") == 0);
    char tiny[4];
    CHECK(proto_fmt_ok(tiny, sizeof tiny, 100, 100) == 3 && tiny[3] == 0);
}

static int ticks_until(control_t *c, int left, int right, int max)
{
    for (int i = 1; i <= max; i++) {
        control_tick(c);
        if (c->left.applied == left && c->right.applied == right) return i;
    }
    return -1;
}

static void test_ramp(void)
{
    control_t c;
    control_init(&c);
    control_command(&c, 100, 50);
    int t = ticks_until(&c, 100, 50, 100);
    CHECK(t == 25);                                   /* 4 % per 10 ms tick */
    control_init(&c);
    control_command(&c, 3, -2);
    CHECK(ticks_until(&c, 3, -2, 5) == 1);            /* small steps land exactly */
}

static void test_reverse(void)
{
    control_t c;
    control_init(&c);
    control_command(&c, 40, 40);
    ticks_until(&c, 40, 40, 50);
    control_command(&c, -40, 40);
    int saw_zero = 0, saw_negative_too_early = 0, ticks_at_zero = 0;
    for (int i = 0; i < 60; i++) {
        if (i % 20 == 0) control_command(&c, -40, 40);    /* keep the watchdog fed */
        control_tick(&c);
        if (c.left.applied == 0) { saw_zero = 1; ticks_at_zero++; }
        if (c.left.applied < 0 && ticks_at_zero < CTRL_COAST_MS / CTRL_TICK_MS) saw_negative_too_early = 1;
    }
    CHECK(saw_zero);
    CHECK(!saw_negative_too_early);                   /* held at 0 for the coast time */
    CHECK(c.left.applied == -40);
    CHECK(c.right.applied == 40);
}

static void test_watchdog(void)
{
    control_t c;
    control_init(&c);
    control_command(&c, 60, 60);
    for (int i = 0; i < 40; i++) {                     /* 400 ms, refreshed every 200 ms */
        if (i % 20 == 0) control_command(&c, 60, 60);
        control_tick(&c);
    }
    CHECK(c.watchdog_trips == 0 && c.left.applied == 60);

    int trip_tick = -1;
    for (int i = 1; i <= 100 && trip_tick < 0; i++) {   /* now go quiet */
        control_tick(&c);
        if (c.tripped) trip_tick = i;
    }
    CHECK(c.watchdog_trips == 1);
    CHECK(c.left.applied == 0 && c.right.applied == 0 && c.left.target == 0);
    /* last command was at 200 ms into the run; quiet since then -> trips at 700 ms */
    CHECK(trip_tick == 30);

    /* stopped robot with only pings must not trip */
    c.tripped = 0;
    for (int i = 0; i < 200; i++) { if (i % 30 == 0) control_keepalive(&c); control_tick(&c); }
    CHECK(c.watchdog_trips == 1);
}

int main(void)
{
    test_parse();
    test_lines();
    test_format();
    test_ramp();
    test_reverse();
    test_watchdog();
    printf("%d checks, %d failed\n", checks, fails);
    return fails != 0;
}
