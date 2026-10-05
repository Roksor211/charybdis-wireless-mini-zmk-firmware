/* SPDX-License-Identifier: MIT */
#include <stdint.h>
#include <zephyr/input/input.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

/* Process sensor motion outside the shared system workqueue. */
static struct k_work_q charybdis_sensor_work_q;
K_THREAD_STACK_DEFINE(charybdis_sensor_stack, 2048);

static int charybdis_sensor_work_q_init(void)
{
    static const struct k_work_queue_config config = {
        .name = "Charybdis sensor",
    };

    k_work_queue_start(&charybdis_sensor_work_q,
                       charybdis_sensor_stack,
                       K_THREAD_STACK_SIZEOF(charybdis_sensor_stack),
                       K_PRIO_PREEMPT(5),
                       &config);
    return 0;
}

SYS_INIT(charybdis_sensor_work_q_init, POST_KERNEL,
         CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int charybdis_sensor_work_submit(struct k_work *work)
{
    return k_work_submit_to_queue(&charybdis_sensor_work_q, work);
}

/* Additional 22.5-degree correction for the single PMW3610 sensor.
 * Screen coordinates: X points right, Y points down.
 */
static int charybdis_report_rotated(const struct device *dev, uint16_t code,
                                  int32_t value, bool sync, k_timeout_t timeout)
{
    static int32_t x, y;
    static int32_t remainder_x, remainder_y;

    if (code == INPUT_REL_X) {
        x += value;
    } else if (code == INPUT_REL_Y) {
        y += value;
    } else {
        return input_report_rel(dev, code, value, sync, timeout);
    }

    if (!sync) {
        return 0;
    }

    /* Additional 22.5-degree counterclockwise screen correction. */
    int64_t scaled_x = (int64_t)x * 30274 + (int64_t)y * 12540 + remainder_x;
    int64_t scaled_y = (int64_t)y * 30274 - (int64_t)x * 12540 + remainder_y;

    int32_t rotated_x = scaled_x / 32768;
    int32_t rotated_y = scaled_y / 32768;

    remainder_x = scaled_x - (int64_t)rotated_x * 32768;
    remainder_y = scaled_y - (int64_t)rotated_y * 32768;

    x = 0;
    y = 0;

    int ret = input_report_rel(dev, INPUT_REL_X, rotated_x, false, timeout);
    if (ret < 0) {
        return ret;
    }

    return input_report_rel(dev, INPUT_REL_Y, rotated_y, true, timeout);
}

/* Compile the pinned driver with rotation and a dedicated sensor queue. */
#define input_report_rel charybdis_report_rotated
#define k_work_submit charybdis_sensor_work_submit
#include "input_pmw3610.c"
#undef k_work_submit
#undef input_report_rel