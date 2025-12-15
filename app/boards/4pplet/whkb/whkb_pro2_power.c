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
 * Sleep guard: periodically check if wakeup source is available.
 * If not, generate a fake key event to reset the sleep timer.
 * This prevents entering deep sleep without a way to wake up.
 *
 * Check interval should be less than CONFIG_ZMK_IDLE_SLEEP_TIMEOUT.
 */
#if IS_ENABLED(CONFIG_ZMK_SLEEP)

#define SLEEP_GUARD_INTERVAL_SEC 30
static bool wakeup_warning_logged = false;

static bool has_wakeup_source(void)
{
    const struct device *wakeup_dev = device_get_binding("wakeup_trigger");
    return (wakeup_dev != NULL && device_is_ready(wakeup_dev));
}

static void sleep_guard_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(sleep_guard_work, sleep_guard_work_handler);

static void sleep_guard_work_handler(struct k_work *work)
{
    if (!has_wakeup_source()) {
        if (!wakeup_warning_logged) {
            LOG_WRN("No wakeup source - sleep guard active");
            wakeup_warning_logged = true;
        }
        /*
         * Generate a fake position event to reset the activity timer.
         * This prevents the system from entering sleep.
         */
        raise_zmk_position_state_changed((struct zmk_position_state_changed){
            .source = 0,
            .position = 0,
            .state = false,  /* Key release - less intrusive */
            .timestamp = k_uptime_get()
        });
    } else {
        if (wakeup_warning_logged) {
            LOG_INF("Wakeup source now available - sleep guard inactive");
            wakeup_warning_logged = false;
        }
    }

    /* Reschedule */
    k_work_schedule(&sleep_guard_work, K_SECONDS(SLEEP_GUARD_INTERVAL_SEC));
}

static void start_sleep_guard(void)
{
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

    /*
     * Power cycle sequence:
     * 1. Drive low to cut power (in case it was already high)
     * 2. Wait for capacitors to discharge and chip to fully power down
     * 3. Drive high to restore power
     * 4. Wait for ADXL362 startup time before driver init
     */
    LOG_INF("ADXL362 power cycle: OFF");
    gpio_pin_set(gpio0, ADXL_POWER_PIN, 0);
    k_sleep(K_MSEC(50));  /* Allow full discharge */

    LOG_INF("ADXL362 power cycle: ON");
    gpio_pin_set(gpio0, ADXL_POWER_PIN, 1);
    k_sleep(K_MSEC(10));  /* ADXL362 needs ~5ms after power-on */

    LOG_INF("ADXL362 power cycle complete");

#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    start_sleep_guard();
#endif

    return 0;
}

/* Run at POST_KERNEL priority 50, before APPLICATION (90) where ADXL362 driver runs */
SYS_INIT(whkb_pro2_power_init, POST_KERNEL, 50);
