/*
 * pmac_link.c - RPMsg endpoint receiving PMAC telemetry from Linux.
 *
 * Follows the SDK's i.MX93 rpmsg_lite_str_echo_rtos example: the user link
 * (VDEV1 vrings), one endpoint, a name-service announcement. Two differences:
 *
 *  - The channel is "rpmsg-raw", not the example's tty channel. Linux's
 *    rpmsg_char driver binds to that name and gives a /dev/rpmsgN where each
 *    write() is exactly one message, so a binary struct arrives intact.
 *
 *  - No rpmsg_queue and no receive task. The endpoint callback runs in the MU
 *    interrupt (rpmsg-lite's env_isr), copies the 52-byte message into a
 *    snapshot and releases the buffer. That is all the work there is, and it
 *    saves the queue's heap and a task's stack.
 *
 * The MU interrupt runs at RPMSG_LITE_MU_IRQ_PRIORITY (3), numerically above
 * configMAX_SYSCALL_INTERRUPT_PRIORITY (2), so the ...FROM_ISR calls below are
 * legal - the priority-0 freeze in STATUS.md section 6 cannot happen here.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "board.h"
#include "fsl_debug_console.h"
#include "rpmsg_lite.h"
#include "rpmsg_ns.h"

#include "pmac_link.h"

#define PMAC_LINK_ID         (RL_PLATFORM_IMX93_M33_A55_USER_LINK_ID)
#define PMAC_LINK_SHMEM      (VDEV1_VRING_BASE)
/*
 * Wait between link-up and the name-service announcement. The SDK example
 * waits 1 s for the same reason: announce too early and Linux has not yet
 * registered its name-service handler, so the channel never appears.
 */
#define PMAC_LINK_NS_DELAY_MS 1000U

/* Written only in the MU interrupt, read under a critical section. */
static pmac_telem_t      s_latest;
static TickType_t        s_rx_tick;
static bool              s_have;
static pmac_link_stats_t s_stats;

static int32_t pmac_rx_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    const pmac_telem_t *m = (const pmac_telem_t *)payload;
    UBaseType_t         saved;

    (void)src;
    (void)priv;

    if ((payload_len != sizeof(pmac_telem_t)) || (m->magic != PMAC_TELEM_MAGIC) ||
        (m->version != PMAC_TELEM_VERSION))
    {
        s_stats.bad++;
        return RL_RELEASE;
    }

    saved = taskENTER_CRITICAL_FROM_ISR();
    if (s_have && (m->seq != s_latest.seq + 1U))
    {
        s_stats.gaps += m->seq - s_latest.seq - 1U;
    }
    (void)memcpy(&s_latest, m, sizeof(s_latest));
    s_rx_tick = xTaskGetTickCountFromISR();
    s_have    = true;
    s_stats.rx++;
    taskEXIT_CRITICAL_FROM_ISR(saved);

    return RL_RELEASE;
}

static void pmac_link_task(void *pvParameters)
{
    struct rpmsg_lite_instance *rl;
    struct rpmsg_lite_endpoint *ept;

    (void)pvParameters;

    rl = rpmsg_lite_remote_init((void *)PMAC_LINK_SHMEM, PMAC_LINK_ID, RL_NO_FLAGS);
    if (rl == RL_NULL)
    {
        PRINTF("RPMSG: WARNING init failed - no motor controller telemetry\r\n");
        vTaskDelete(NULL);
    }

    /* Blocks until Linux has set up the vrings - immediately after a
     * remoteproc start, never if Linux has no rpmsg support loaded. */
    (void)rpmsg_lite_wait_for_link_up(rl, RL_BLOCK);

    ept = rpmsg_lite_create_ept(rl, PMAC_TELEM_EPT_ADDR, pmac_rx_cb, RL_NULL);
    if (ept == RL_NULL)
    {
        PRINTF("RPMSG: WARNING endpoint %u not created\r\n", (unsigned int)PMAC_TELEM_EPT_ADDR);
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(PMAC_LINK_NS_DELAY_MS));
    if (rpmsg_ns_announce(rl, ept, PMAC_TELEM_CHANNEL, (uint32_t)RL_NS_CREATE) != RL_SUCCESS)
    {
        PRINTF("RPMSG: WARNING name service announce failed\r\n");
        vTaskDelete(NULL);
    }
    s_stats.announced = true;

    /* Everything from here happens in pmac_rx_cb. */
    vTaskDelete(NULL);
}

bool pmac_link_start(void)
{
    /* +200 words: rpmsg-lite's init path, plus PRINTF on the failure paths. */
    return xTaskCreate(pmac_link_task, "pmac_link", configMINIMAL_STACK_SIZE + 200, NULL,
                       tskIDLE_PRIORITY + 1U, NULL) == pdPASS;
}

uint32_t pmac_link_latest(pmac_telem_t *out)
{
    bool       have;
    TickType_t at;

    taskENTER_CRITICAL();
    have = s_have;
    at   = s_rx_tick;
    if (have)
    {
        *out = s_latest;
    }
    taskEXIT_CRITICAL();

    if (!have)
    {
        return UINT32_MAX;
    }
    return (uint32_t)((xTaskGetTickCount() - at) * portTICK_PERIOD_MS);
}

void pmac_link_stats(pmac_link_stats_t *out)
{
    taskENTER_CRITICAL();
    *out = s_stats;
    taskEXIT_CRITICAL();
}
