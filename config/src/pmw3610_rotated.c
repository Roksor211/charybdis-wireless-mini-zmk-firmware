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

static void charybdis_motion_work_handler(struct k_work *work);

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

/* Accumulate movement; all accesses run on the sensor workqueue. */
static const struct device *pending_motion_dev;
static int32_t pending_x, pending_y;
static int32_t rotation_remainder_x, rotation_remainder_y;

static void charybdis_flush_motion(struct k_work *work)
{
    ARG_UNUSED(work);

    if (pending_motion_dev == NULL ||
        (pending_x == 0 && pending_y == 0)) {
        return;
    }

    int64_t scaled_x = (int64_t)pending_x * 30274
                     + (int64_t)pending_y * 12540
                     + rotation_remainder_x;
    int64_t scaled_y = (int64_t)pending_y * 30274
                     - (int64_t)pending_x * 12540
                     + rotation_remainder_y;

    int32_t rotated_x = scaled_x / 32768;
    int32_t rotated_y = scaled_y / 32768;

    rotation_remainder_x =
        scaled_x - (int64_t)rotated_x * 32768;
    rotation_remainder_y =
        scaled_y - (int64_t)rotated_y * 32768;

    pending_x = 0;
    pending_y = 0;

    if (rotated_x == 0 && rotated_y == 0) {
        return;
    }

    atomic_set(&diag_phase, 2);
    int ret = input_report_rel(pending_motion_dev, INPUT_REL_X,
                               rotated_x, false, K_FOREVER);
    if (ret < 0) {
        atomic_inc(&diag_errors);
        atomic_set(&diag_phase, 0);
        return;
    }

    atomic_set(&diag_phase, 3);
    ret = input_report_rel(pending_motion_dev, INPUT_REL_Y,
                           rotated_y, true, K_FOREVER);
    if (ret < 0) {
        atomic_inc(&diag_errors);
    }

    atomic_set(&diag_phase, 0);
}

K_WORK_DELAYABLE_DEFINE(charybdis_motion_flush_work,
                        charybdis_flush_motion);

static int charybdis_report_rotated(const struct device *dev,
                                  uint16_t code, int32_t value,
                                  bool sync, k_timeout_t timeout)
{
    if (code == INPUT_REL_X) {
        pending_x += value;
    } else if (code == INPUT_REL_Y) {
        pending_y += value;
    } else {
        return input_report_rel(dev, code, value, sync, timeout);
    }

    pending_motion_dev = dev;

    if (!sync) {
        return 0;
    }

    /* Keep the first deadline; later samples join the same batch. */
    int ret = k_work_schedule_for_queue(
        &charybdis_sensor_work_q,
        &charybdis_motion_flush_work,
        K_MSEC(12));

    if (ret < 0) {
        atomic_inc(&diag_errors);
        return ret;
    }

    return 0;
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