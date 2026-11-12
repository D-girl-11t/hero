/*
 * m33_sim: the M33 firmware's protocol and control code running on a PC,
 * talking over a pseudo-terminal instead of RPMsg. Lets the Linux app and its
 * tests run without the board:
 *
 *   ./m33_sim --pty            creates a pty, prints its path, serves it
 *   ./m33_sim --fd N           serves an already-open fd (used by the tests)
 *
 * Every 10 ms it runs control_tick() exactly like control_task on the M33.
 */
#define _XOPEN_SOURCE 600
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "../src/control.h"
#include "../src/protocol.h"

#define VERSION "hero-m33 1.0.0 (simulator)"

static int fd = -1;
static control_t ctrl;
static int verbose;

static void send_str(const char *s, size_t n)
{
    if (n && write(fd, s, n) < 0) perror("write");
}

static void on_line(const char *line, size_t len, void *ctx)
{
    (void)ctx;
    char out[PROTO_MAX_LINE];
    size_t n = 0;
    cmd_t c = proto_parse(line, len);
    switch (c.type) {
    case CMD_DRIVE:  control_command(&ctrl, c.left, c.right); n = proto_fmt_ok(out, sizeof out, ctrl.left.target, ctrl.right.target); break;
    case CMD_STOP:   control_command(&ctrl, 0, 0); n = proto_fmt_ok(out, sizeof out, 0, 0); break;
    case CMD_PING:   control_keepalive(&ctrl); n = proto_fmt_pong(out, sizeof out, VERSION); break;
    case CMD_STATUS: control_keepalive(&ctrl); n = proto_fmt_status(out, sizeof out, ctrl.left.applied, ctrl.right.applied, ctrl.watchdog_trips, ctrl.now_ms); break;
    case CMD_ERROR:  n = proto_fmt_err(out, sizeof out, c.error); break;
    case CMD_NONE:   break;
    }
    if (verbose) fprintf(stderr, "m33_sim <- %.*s   -> %.*s", (int)len, line, (int)n, n ? out : "\n");
    send_str(out, n);
}

static long ms_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--fd") && i + 1 < argc) fd = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pty")) {
            fd = posix_openpt(O_RDWR | O_NOCTTY);
            if (fd < 0 || grantpt(fd) || unlockpt(fd)) { perror("pty"); return 1; }
            struct termios t;
            int s = open(ptsname(fd), O_RDWR | O_NOCTTY);
            tcgetattr(s, &t); cfmakeraw(&t); tcsetattr(s, TCSANOW, &t); close(s);
            printf("%s\n", ptsname(fd));
            fflush(stdout);
        } else if (!strcmp(argv[i], "-v")) verbose = 1;
    }
    if (fd < 0) { fprintf(stderr, "usage: %s --pty | --fd N [-v]\n", argv[0]); return 2; }

    control_init(&ctrl);
    line_buf_t lb;
    proto_line_init(&lb);
    long next = ms_now() + CTRL_TICK_MS;

    for (;;) {
        long wait = next - ms_now();
        struct pollfd p = { fd, POLLIN, 0 };
        int r = poll(&p, 1, wait > 0 ? (int)wait : 0);
        if (r > 0 && (p.revents & POLLIN)) {
            char buf[256];
            ssize_t n = read(fd, buf, sizeof buf);
            if (n > 0) proto_line_feed(&lb, buf, (size_t)n, on_line, NULL);
            else if (n == 0 || (errno != EAGAIN && errno != EIO)) return 0;
        } else if (r > 0 && (p.revents & (POLLHUP | POLLERR))) {
            usleep(10000);   /* pty with no reader yet */
        }
        while (ms_now() >= next) {
            next += CTRL_TICK_MS;
            control_tick(&ctrl);
            if (ctrl.tripped) {
                ctrl.tripped = 0;
                char out[8];
                send_str(out, proto_fmt_wd(out, sizeof out));
                if (verbose) fprintf(stderr, "m33_sim: watchdog stop\n");
            }
        }
    }
}
