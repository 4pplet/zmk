/*
 * Copyright (c) 2025 Stefan Sundin (4pplet)
 *
 * SPDX-License-Identifier: MIT
 *
 * WHKB Pro2 (Rev B) Power Control & Sleep Guard
 *
 * This module is specific to the WHKB Pro2 hardware which has:
 * - ADXL362 accelerometer for wake-from-sleep (GPIO-powered via P0.28)
 * - Deep sleep support requiring a working wakeup source
 *
 * Functions:
 * 1. Power-cycles the ADXL362 on P0.28 during init to ensure a clean
 *    power-on reset (DCDC stays running during warm reboot)
 * 2. Provides a sleep guard that prevents deep sleep if no wakeup source
 *    is available (avoids bricking the keyboard)
 *
 * The power init runs at POST_KERNEL priority to execute before the ADXL362
 * driver which runs at APPLICATION priority.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_REGISTER(whkb_pro2_power, CONFIG_LOG_DEFAULT_LEVEL);

/* P0.28 controls ADXL362 power (active high) */
#define ADXL_POWER_NODE DT_NODELABEL(gpio0)
#define ADXL_POWER_PIN  28

/*
 * Sleep guard: prevent deep sleep if no wakeup source available.
 * Generates fake key events to reset activity timer.
 */
#if IS_ENABLED(CONFIG_ZMK_SLEEP)

#define SLEEP_GUARD_INTERVAL_SEC 30

static bool has_wakeup_source(void)
{
    const struct device *dev = device_get_binding("wakeup_trigger");
    return (dev != NULL && device_is_ready(dev));
}

static void sleep_guard_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(sleep_guard_work, sleep_guard_work_handler);

static void sleep_guard_work_handler(struct k_work *work)
{
    if (!has_wakeup_source()) {
        raise_zmk_position_state_changed((struct zmk_position_state_changed){
            .source = 0, .position = 0, .state = false, .timestamp = k_uptime_get()
        });
    }
    k_work_schedule(&sleep_guard_work, K_SECONDS(SLEEP_GUARD_INTERVAL_SEC));
}

#endif /* CONFIG_ZMK_SLEEP */

static int whkb_pro2_power_init(void)
{
    const struct device *gpio0 = DEVICE_DT_GET(ADXL_POWER_NODE);

    if (!device_is_ready(gpio0)) {
        LOG_ERR("GPIO0 not ready");
        return -ENODEV;
    }

    /* Configure P0.28 as output */
    int ret = gpio_pin_configure(gpio0, ADXL_POWER_PIN, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure P0.28: %d", ret);
        return ret;
    }

    /* Power cycle: OFF 100ms, ON 20ms */
    gpio_pin_set(gpio0, ADXL_POWER_PIN, 0);
    k_sleep(K_MSEC(100));
    gpio_pin_set(gpio0, ADXL_POWER_PIN, 1);
    k_sleep(K_MSEC(20));

#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    k_work_schedule(&sleep_guard_work, K_SECONDS(SLEEP_GUARD_INTERVAL_SEC));
#endif

    return 0;
}

/* Run at POST_KERNEL priority 50, before APPLICATION (90) where ADXL362 driver runs */
SYS_INIT(whkb_pro2_power_init, POST_KERNEL, 50);
