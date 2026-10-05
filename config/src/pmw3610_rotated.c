/* SPDX-License-Identifier: MIT */
#include <stdint.h>
#include <zephyr/input/input.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

static atomic_t diag_requested, diag_started, diag_finished;
static atomic_t diag_phase, diag_errors;
static struct k_work *diag_motion_work;
static k_work_handler_t diag_original_handler;

static void charybdis_motion_work_handler(struct k_work *work)
{
    struct pmw3610_data *data =
        CONTAINER_OF(work, struct pmw3610_data, motion_work);
    const struct pmw3610_config *cfg = data->dev->config;

    atomic_inc(&diag_started);
    atomic_set(&diag_phase, 1);

    diag_original_handler(work);

    atomic_inc(&diag_finished);
    atomic_set(&diag_phase, 0);

    /* The motion line may still be active without a new interrupt edge. */
    int motion = gpio_pin_get_dt(&cfg->motion_gpio);
    if (motion > 0) {
        /* Limit retries if the signal stays active persistently. */
        k_sleep(K_MSEC(1));

        int ret = charybdis_sensor_work_submit(work);
        if (ret < 0) {
            atomic_inc(&diag_errors);
        }
    } else if (motion < 0) {
        atomic_inc(&diag_errors);
    }
}

static void charybdis_motion_work_init(struct k_work *work,
                                      k_work_handler_t handler)
{
    diag_motion_work = work;
    diag_original_handler = handler;
    k_work_init(work, charybdis_motion_work_handler);
}

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
    atomic_inc(&diag_requested);
    return k_work_submit_to_queue(&charybdis_sensor_work_q, work);
}

/* Additional 22.5-degree counterclockwise screen correction. */
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

    int64_t scaled_x = (int64_t)x * 30274
                     + (int64_t)y * 12540 + remainder_x;
    int64_t scaled_y = (int64_t)y * 30274
                     - (int64_t)x * 12540 + remainder_y;

    int32_t rotated_x = scaled_x / 32768;
    int32_t rotated_y = scaled_y / 32768;

    remainder_x = scaled_x - (int64_t)rotated_x * 32768;
    remainder_y = scaled_y - (int64_t)rotated_y * 32768;

    x = 0;
    y = 0;

    atomic_set(&diag_phase, 2);
    int ret = input_report_rel(dev, INPUT_REL_X, rotated_x, false, timeout);
    if (ret < 0) {
        atomic_inc(&diag_errors);
        return ret;
    }

    atomic_set(&diag_phase, 3);
    ret = input_report_rel(dev, INPUT_REL_Y, rotated_y, true, timeout);
    if (ret < 0) {
        atomic_inc(&diag_errors);
    }

    atomic_set(&diag_phase, 4);
    return ret;
}

/* Compile the pinned driver with rotation and instrumented motion work. */
#define input_report_rel charybdis_report_rotated
#define k_work_submit charybdis_sensor_work_submit
#define k_work_init charybdis_motion_work_init
#include "input_pmw3610.c"
#undef k_work_init
#undef k_work_submit
#undef input_report_rel

static void charybdis_motion_work_handler(struct k_work *work)
{
    atomic_inc(&diag_started);
    atomic_set(&diag_phase, 1);

    diag_original_handler(work);

    atomic_inc(&diag_finished);
    atomic_set(&diag_phase, 0);
}

static void charybdis_diag_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (true) {
        k_sleep(K_SECONDS(5));

        int motion = -1;
        if (diag_motion_work != NULL) {
            struct pmw3610_data *data =
                CONTAINER_OF(diag_motion_work,
                             struct pmw3610_data, motion_work);
            const struct pmw3610_config *cfg = data->dev->config;
            motion = gpio_pin_get_dt(&cfg->motion_gpio);
        }

        printk("TB_DIAG t=%lld req=%ld start=%ld done=%ld "
               "phase=%ld motion=%d errors=%ld\n",
               (long long)k_uptime_get(),
               (long)atomic_get(&diag_requested),
               (long)atomic_get(&diag_started),
               (long)atomic_get(&diag_finished),
               (long)atomic_get(&diag_phase),
               motion,
               (long)atomic_get(&diag_errors));
    }
}

K_THREAD_DEFINE(charybdis_diag_tid, 2048, charybdis_diag_thread,
                NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, 0);