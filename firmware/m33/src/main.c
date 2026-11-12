/*
 * Hero motor firmware for the i.MX93 Cortex-M33 (FreeRTOS + RPMsg-Lite).
 *
 *   rpmsg_task    receives command lines from Linux, replies, sends "wd" events
 *   control_task  every 10 ms: slew limit, safe reversal, watchdog, update pins
 *
 * Linux loads this with remoteproc; Linux sees it as /dev/ttyRPMSG30.
 */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "board.h"
#include "fsl_debug_console.h"
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "rsc_table.h"

#include "hero_board.h"
#include "control.h"
#include "motor_hw.h"
#include "protocol.h"

#define RPMSG_LITE_LINK_ID      (RL_PLATFORM_IMX93_M33_A55_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE   ((void *)(char *)VDEV0_VRING_BASE)

static struct rpmsg_lite_instance *s_rpmsg;
static struct rpmsg_lite_endpoint *s_ept;
static rpmsg_queue_handle s_queue;
static volatile uint32_t s_remote = RL_ADDR_ANY;  /* Linux endpoint, learned from first message */

static control_t s_ctrl;
static SemaphoreHandle_t s_ctrl_lock;

static void reply(const char *msg, size_t len)
{
    if (s_remote != RL_ADDR_ANY && len)
        (void)rpmsg_lite_send(s_rpmsg, s_ept, s_remote, (char *)msg, len, RL_BLOCK);
}

/* Called for every complete line received from Linux. */
static void on_line(const char *line, size_t len, void *ctx)
{
    (void)ctx;
    char out[PROTO_MAX_LINE];
    size_t n = 0;
    cmd_t c = proto_parse(line, len);

    xSemaphoreTake(s_ctrl_lock, portMAX_DELAY);
    switch (c.type) {
    case CMD_DRIVE:
        control_command(&s_ctrl, c.left, c.right);
        n = proto_fmt_ok(out, sizeof out, s_ctrl.left.target, s_ctrl.right.target);
        break;
    case CMD_STOP:
        control_command(&s_ctrl, 0, 0);
        n = proto_fmt_ok(out, sizeof out, 0, 0);
        break;
    case CMD_PING:
        control_keepalive(&s_ctrl);
        n = proto_fmt_pong(out, sizeof out, HERO_FW_VERSION);
        break;
    case CMD_STATUS:
        control_keepalive(&s_ctrl);
        n = proto_fmt_status(out, sizeof out, s_ctrl.left.applied, s_ctrl.right.applied,
                             s_ctrl.watchdog_trips, s_ctrl.now_ms);
        break;
    case CMD_ERROR:
        n = proto_fmt_err(out, sizeof out, c.error);
        break;
    case CMD_NONE:
        break;
    }
    xSemaphoreGive(s_ctrl_lock);
    reply(out, n);
}

static void rpmsg_task(void *arg)
{
    (void)arg;
    line_buf_t lb;
    proto_line_init(&lb);

    s_rpmsg = rpmsg_lite_remote_init(RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
    rpmsg_lite_wait_for_link_up(s_rpmsg, RL_BLOCK);

    s_queue = rpmsg_queue_create(s_rpmsg);
    s_ept = rpmsg_lite_create_ept(s_rpmsg, HERO_EPT_ADDR, rpmsg_queue_rx_cb, s_queue);
    (void)rpmsg_ns_announce(s_rpmsg, s_ept, HERO_NS_NAME, (uint32_t)RL_NS_CREATE);
    PRINTF("hero-m33: endpoint %u announced as \"%s\"\r\n", HERO_EPT_ADDR, HERO_NS_NAME);

    for (;;) {
        uint32_t src;
        char *buf;
        uint32_t len;
        if (rpmsg_queue_recv_nocopy(s_rpmsg, s_queue, &src, &buf, &len, RL_BLOCK) != RL_SUCCESS)
            continue;
        s_remote = src;
        proto_line_feed(&lb, buf, len, on_line, NULL);
        (void)rpmsg_queue_nocopy_free(s_rpmsg, buf);
    }
}

static void control_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CTRL_TICK_MS));

        xSemaphoreTake(s_ctrl_lock, portMAX_DELAY);
        int changed = control_tick(&s_ctrl);
        int8_t l = s_ctrl.left.applied, r = s_ctrl.right.applied;
        int tripped = s_ctrl.tripped;
        s_ctrl.tripped = 0;
        xSemaphoreGive(s_ctrl_lock);

        if (changed) motor_hw_apply(l, r);
        if (tripped) {
            char out[8];
            PRINTF("hero-m33: watchdog stop\r\n");
            reply(out, proto_fmt_wd(out, sizeof out));
        }
    }
}

int main(void)
{
    BOARD_InitHardware();            /* clocks, debug UART (from the SDK board files) */
    copyResourceTable();             /* tell remoteproc where our vrings are */

    control_init(&s_ctrl);
    motor_hw_init();
    motor_hw_apply(0, 0);
    s_ctrl_lock = xSemaphoreCreateMutex();

    PRINTF("\r\n%s starting\r\n", HERO_FW_VERSION);
    xTaskCreate(control_task, "control", 512, NULL, tskIDLE_PRIORITY + 3, NULL);   /* highest: keeps PWM updates on time */
    xTaskCreate(rpmsg_task, "rpmsg", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    vTaskStartScheduler();
    for (;;) { }
}
