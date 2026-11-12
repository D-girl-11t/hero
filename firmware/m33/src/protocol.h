/*
 * Hero motor protocol: one text line per RPMsg message.
 *
 *   A55 -> M33                     M33 -> A55
 *   drv <L> <R>   (-100..100 %)    ok <L> <R>          target accepted
 *   stop                           ok 0 0
 *   ping                           pong <version>
 *   st                             st <L> <R> <trips> <uptime_ms>   applied duty
 *                                  wd                  watchdog stopped the motors (event)
 *                                  err <reason>
 *
 * Pure C with no SDK calls, so it builds and is tested on the host too.
 */
#ifndef HERO_PROTOCOL_H
#define HERO_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define PROTO_MAX_LINE 64

typedef enum {
    CMD_NONE = 0,   /* empty line */
    CMD_DRIVE,
    CMD_STOP,
    CMD_PING,
    CMD_STATUS,
    CMD_ERROR       /* unparseable: reason in cmd.error */
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    int8_t left;            /* -100..100, CMD_DRIVE only */
    int8_t right;
    const char *error;      /* static string, CMD_ERROR only */
} cmd_t;

/* Parse one line (no newline needed). Never reads past len. */
cmd_t proto_parse(const char *line, size_t len);

/* Collects bytes into lines. Messages may split or join lines, so feed every
 * received buffer here; on_line runs once per complete line. */
typedef void (*proto_line_cb)(const char *line, size_t len, void *ctx);
typedef struct {
    char buf[PROTO_MAX_LINE];
    size_t used;
    uint8_t overflow;       /* current line too long: drop it */
} line_buf_t;

void proto_line_init(line_buf_t *lb);
void proto_line_feed(line_buf_t *lb, const char *data, size_t len, proto_line_cb on_line, void *ctx);

/* Replies. Return the length written (excluding the NUL), always NUL-terminated. */
size_t proto_fmt_ok(char *out, size_t n, int left, int right);
size_t proto_fmt_pong(char *out, size_t n, const char *version);
size_t proto_fmt_status(char *out, size_t n, int left, int right, uint32_t trips, uint32_t uptime_ms);
size_t proto_fmt_wd(char *out, size_t n);
size_t proto_fmt_err(char *out, size_t n, const char *reason);

#endif
