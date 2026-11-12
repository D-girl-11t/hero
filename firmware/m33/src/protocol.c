#include "protocol.h"

#include <stdio.h>
#include <string.h>

static int is_space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

/* Next whitespace-separated token. */
static size_t next_token(const char *s, size_t len, size_t *pos, const char **tok)
{
    while (*pos < len && is_space(s[*pos])) (*pos)++;
    size_t start = *pos;
    while (*pos < len && !is_space(s[*pos])) (*pos)++;
    *tok = s + start;
    return *pos - start;
}

static int tok_is(const char *tok, size_t n, const char *word)
{
    return strlen(word) == n && memcmp(tok, word, n) == 0;
}

/* Signed decimal in -100..100. Returns 0 on success. */
static int parse_duty(const char *tok, size_t n, int8_t *out)
{
    if (n == 0 || n > 4) return -1;
    size_t i = 0;
    int neg = 0, v = 0;
    if (tok[0] == '-' || tok[0] == '+') { neg = tok[0] == '-'; i = 1; }
    if (i == n) return -1;
    for (; i < n; i++) {
        if (tok[i] < '0' || tok[i] > '9') return -1;
        v = v * 10 + (tok[i] - '0');
    }
    if (v > 100) return -1;
    *out = (int8_t)(neg ? -v : v);
    return 0;
}

cmd_t proto_parse(const char *line, size_t len)
{
    cmd_t c = { CMD_NONE, 0, 0, NULL };
    size_t pos = 0;
    const char *tok, *extra;
    size_t n = next_token(line, len, &pos, &tok);
    if (n == 0) return c;

    if (tok_is(tok, n, "drv")) {
        const char *a, *b;
        size_t na = next_token(line, len, &pos, &a);
        size_t nb = next_token(line, len, &pos, &b);
        if (parse_duty(a, na, &c.left) || parse_duty(b, nb, &c.right)) {
            c.type = CMD_ERROR; c.error = "drv needs two values -100..100"; return c;
        }
        c.type = CMD_DRIVE;
    } else if (tok_is(tok, n, "stop")) {
        c.type = CMD_STOP;
    } else if (tok_is(tok, n, "ping")) {
        c.type = CMD_PING;
    } else if (tok_is(tok, n, "st")) {
        c.type = CMD_STATUS;
    } else {
        c.type = CMD_ERROR; c.error = "unknown command"; return c;
    }
    if (next_token(line, len, &pos, &extra) != 0) {
        c.type = CMD_ERROR; c.error = "unexpected text after command";
        c.left = c.right = 0;
    }
    return c;
}

void proto_line_init(line_buf_t *lb)
{
    lb->used = 0;
    lb->overflow = 0;
}

void proto_line_feed(line_buf_t *lb, const char *data, size_t len, proto_line_cb on_line, void *ctx)
{
    for (size_t i = 0; i < len; i++) {
        char ch = data[i];
        if (ch == '\n' || ch == '\0') {
            if (!lb->overflow && on_line) on_line(lb->buf, lb->used, ctx);
            lb->used = 0;
            lb->overflow = 0;
            continue;
        }
        if (lb->overflow) continue;
        if (lb->used >= sizeof lb->buf - 1) {     /* too long: drop until next newline */
            lb->overflow = 1;
            lb->used = 0;
            continue;
        }
        lb->buf[lb->used++] = ch;
    }
}

static size_t clamp_len(int r, size_t n) { return r < 0 ? 0 : ((size_t)r >= n ? n - 1 : (size_t)r); }

size_t proto_fmt_ok(char *out, size_t n, int left, int right)
{ return clamp_len(snprintf(out, n, "ok %d %d\n", left, right), n); }

size_t proto_fmt_pong(char *out, size_t n, const char *version)
{ return clamp_len(snprintf(out, n, "pong %s\n", version), n); }

size_t proto_fmt_status(char *out, size_t n, int left, int right, uint32_t trips, uint32_t uptime_ms)
{ return clamp_len(snprintf(out, n, "st %d %d %lu %lu\n", left, right, (unsigned long)trips, (unsigned long)uptime_ms), n); }

size_t proto_fmt_wd(char *out, size_t n)
{ return clamp_len(snprintf(out, n, "wd\n"), n); }

size_t proto_fmt_err(char *out, size_t n, const char *reason)
{ return clamp_len(snprintf(out, n, "err %s\n", reason), n); }
