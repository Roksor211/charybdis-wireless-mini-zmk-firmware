/* SPDX-License-Identifier: MIT */
#include <stdint.h>
#include <zephyr/input/input.h>

/* Additional 45-degree correction for the single PMW3610 sensor.
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

    /* Rotate counterclockwise on screen, retaining fractional movement. */
    int64_t scaled_x = ((int64_t)x + y) * 23170 + remainder_x;
    int64_t scaled_y = ((int64_t)y - x) * 23170 + remainder_y;

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

/* Compile the pinned Zephyr driver with the report function above. */
#define input_report_rel charybdis_report_rotated
#include "input_pmw3610.c"
#undef input_report_rel