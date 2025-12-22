/*
 * Copyright (c) 2025 Stefan Sundin (4pplet)
 * SPDX-License-Identifier: MIT
 *
 * WHKB Pro2: Power-cycles ADXL362 (P0.28) at boot for clean POR,
 * and provides sleep guard to prevent deep sleep without wakeup source.
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

/* Get wakeup_trigger device from devicetree if it exists */
#define WAKEUP_TRIGGER_NODE DT_NODELABEL(wakeup_trigger)
#if DT_NODE_EXISTS(WAKEUP_TRIGGER_NODE)
static const struct device *wakeup_dev = DEVICE_DT_GET_OR_NULL(WAKEUP_TRIGGER_NODE);
#else
static const struct device *wakeup_dev = NULL;
#endif

static bool has_wakeup_source(void)
{
    return (wakeup_dev != NULL && device_is_ready(wakeup_dev));
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
