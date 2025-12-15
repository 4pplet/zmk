/*
 * Copyright (c) 2022 Kan-Ru Chen
 * Copyright (c) 2025 Stefan Sundin (4pplet)
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_gpio_topre

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(kscan_topre, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/activity.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* Store device reference for activity listener (single instance) */
static const struct device *topre_dev;

#define SEL_PINS 6

struct kscan_gpio_topre_config {
    struct gpio_dt_spec bits[SEL_PINS];
    struct gpio_dt_spec power;
    struct gpio_dt_spec key;
    struct gpio_dt_spec hys;
    struct gpio_dt_spec strobe;
    struct gpio_dt_spec row_enable_low;  /* JP: Z2 ~Enable (row 0-7) */
    struct gpio_dt_spec row_enable_high; /* JP: Z3 ~Enable (row 8-15) */
    bool is_jp;
    uint8_t matrix_rows;
    uint8_t matrix_cols;
    uint16_t matrix_relax_us;
    uint16_t adc_read_settle_us;
    uint16_t active_polling_interval_ms;
    uint16_t idle_polling_interval_ms;
    uint16_t sleep_polling_interval_ms;
};

struct kscan_gpio_topre_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_timer poll_timer;
    struct k_work poll;
    bool matrix_state[16 * 8]; /* Max size for JP (16x8) */
};

static int kscan_gpio_topre_configure(const struct device *dev, kscan_callback_t callback) {
    LOG_DBG("KSCAN API configure");
    struct kscan_gpio_topre_data *data = dev->data;
    if (!callback) {
        return -EINVAL;
    }
    data->callback = callback;
    LOG_DBG("Configured KSCAN");
    return 0;
}

static int kscan_gpio_topre_enable(const struct device *dev) {
    LOG_DBG("KSCAN API enable");
    struct kscan_gpio_topre_data *data = dev->data;
    const struct kscan_gpio_topre_config *cfg = dev->config;
    k_timer_start(&data->poll_timer, K_MSEC(cfg->active_polling_interval_ms),
                  K_MSEC(cfg->active_polling_interval_ms));
    return 0;
}

static int kscan_gpio_topre_disable(const struct device *dev) {
    LOG_DBG("KSCAN API disable");
    struct kscan_gpio_topre_data *data = dev->data;
    k_timer_stop(&data->poll_timer);
    return 0;
}

static void kscan_gpio_topre_update_polling(const struct device *dev, enum zmk_activity_state state) {
    struct kscan_gpio_topre_data *data = dev->data;
    const struct kscan_gpio_topre_config *cfg = dev->config;
    uint16_t interval_ms;

    switch (state) {
    case ZMK_ACTIVITY_IDLE:
        interval_ms = cfg->idle_polling_interval_ms;
        break;
    case ZMK_ACTIVITY_ACTIVE:
    default:
        interval_ms = cfg->active_polling_interval_ms;
        break;
    }

    LOG_INF("Topre polling interval: %dms (%s)",
            interval_ms, state == ZMK_ACTIVITY_ACTIVE ? "active" : "idle");
    k_timer_start(&data->poll_timer, K_MSEC(interval_ms), K_MSEC(interval_ms));
}

static void kscan_gpio_topre_timer_handler(struct k_timer *timer) {
    struct kscan_gpio_topre_data *data =
        CONTAINER_OF(timer, struct kscan_gpio_topre_data, poll_timer);
    k_work_submit(&data->poll);
}

/* Static buffer to reduce stack usage */
static bool matrix_read[16 * 8];

/* Debug: track scan cycles */
static uint32_t scan_cycle_count = 0;

static void kscan_gpio_topre_work_handler(struct k_work *work) {
    struct kscan_gpio_topre_data *data = CONTAINER_OF(work, struct kscan_gpio_topre_data, poll);
    const struct device *dev = data->dev;
    const struct kscan_gpio_topre_config *cfg = dev->config;
    const int matrix_rows = cfg->matrix_rows;
    const int matrix_cols = cfg->matrix_cols;

    scan_cycle_count++;
    /* Log every 1000 scans (~8 seconds at 8ms interval) */
    if ((scan_cycle_count % 1000) == 0) {
        LOG_INF("Topre scan cycle %u", scan_cycle_count);
    }

    /* Power on everything - use raw gpio to match original driver */
    gpio_pin_configure_dt(&cfg->key, GPIO_INPUT);
    gpio_pin_set(cfg->strobe.port, cfg->strobe.pin, 1);
    gpio_pin_set(cfg->power.port, cfg->power.pin, 1);

    /* For JP, enable both row multiplexers (active low) */
    if (cfg->is_jp) {
        gpio_pin_set(cfg->row_enable_low.port, cfg->row_enable_low.pin, 0);
        gpio_pin_set(cfg->row_enable_high.port, cfg->row_enable_high.pin, 0);
    }

    /* Topre controller board needs time to be operational.
     * Original whkb-zmk-config driver uses 5ms. */
    k_sleep(K_MSEC(5));

    for (int r = 0; r < matrix_rows; ++r) {
        /* For JP, select which multiplexer to use based on row */
        if (cfg->is_jp) {
            if (r < 8) {
                /* Row 0-7: Enable Z2, disable Z3 */
                gpio_pin_set(cfg->row_enable_low.port, cfg->row_enable_low.pin, 0);
                gpio_pin_set(cfg->row_enable_high.port, cfg->row_enable_high.pin, 1);
            } else {
                /* Row 8-15: Enable Z3, disable Z2 */
                gpio_pin_set(cfg->row_enable_low.port, cfg->row_enable_low.pin, 1);
                gpio_pin_set(cfg->row_enable_high.port, cfg->row_enable_high.pin, 0);
            }
        }

        for (int c = 0; c < matrix_cols; ++c) {
            /* Set row selection bits (use lower 3 bits for JP rows 8-15)
             * Note: Use raw pin set to match original driver behavior */
            int row_bits = cfg->is_jp ? (r & 0x07) : r;
            gpio_pin_set(cfg->bits[0].port, cfg->bits[0].pin, row_bits & BIT(0));
            gpio_pin_set(cfg->bits[1].port, cfg->bits[1].pin, row_bits & BIT(1));
            gpio_pin_set(cfg->bits[2].port, cfg->bits[2].pin, row_bits & BIT(2));
            gpio_pin_set(cfg->bits[3].port, cfg->bits[3].pin, c & BIT(0));
            gpio_pin_set(cfg->bits[4].port, cfg->bits[4].pin, c & BIT(1));
            gpio_pin_set(cfg->bits[5].port, cfg->bits[5].pin, c & BIT(2));

            int cell = (r * matrix_cols) + c;
            const bool prev = data->matrix_state[cell];
            /* Hysteresis for Topre EC sensing:
             * Match original whkb-zmk-config behavior: HYS follows previous state.
             * This provides hysteresis - different thresholds for press vs release. */
            gpio_pin_set(cfg->hys.port, cfg->hys.pin, prev);

            /* Wait for multiplexer outputs to settle and HYS to take effect */
            k_busy_wait(cfg->matrix_relax_us);

            const unsigned int lock = irq_lock();
            /* Strobe sequence - matches original whkb-zmk-config driver:
             * 1. Pull strobe LOW to trigger sensing
             * 2. Immediately pull strobe HIGH
             * 3. Wait for signal to settle (capacitor charge/discharge)
             * 4. Read KEY state
             *
             * Note: Use raw gpio operations to match original driver behavior. */
            gpio_pin_set(cfg->strobe.port, cfg->strobe.pin, 0);  /* Trigger sensing */
            gpio_pin_set(cfg->strobe.port, cfg->strobe.pin, 1);  /* Release strobe */
            k_busy_wait(cfg->adc_read_settle_us);  /* Wait for sensing to settle */
            const bool pressed = gpio_pin_get(cfg->key.port, cfg->key.pin);
            irq_unlock(lock);
            /* Reset HYS after reading */
            gpio_pin_set(cfg->hys.port, cfg->hys.pin, 0);

#if IS_ENABLED(CONFIG_ZMK_KSCAN_GPIO_TOPRE_DEBUG)
            /* Log raw KEY state for first few scan cycles to diagnose sensing issues */
            static int scan_count = 0;
            if (scan_count < 3 && r == 0 && c == 0) {
                scan_count++;
            }
            if (scan_count <= 2) {
                LOG_INF("scan%d r%d c%d: bits=%d%d%d%d%d%d KEY=%d hys=%d",
                        scan_count, r, c,
                        (row_bits & BIT(0)) ? 1 : 0,
                        (row_bits & BIT(1)) ? 1 : 0,
                        (row_bits & BIT(2)) ? 1 : 0,
                        (c & BIT(0)) ? 1 : 0,
                        (c & BIT(1)) ? 1 : 0,
                        (c & BIT(2)) ? 1 : 0,
                        pressed, prev);
            }
#endif

            matrix_read[cell] = pressed;
        }
    }

    /* Set all gpio pins to low and power off the controller board to avoid
     * current leakage. Use raw gpio to match original driver. */
    for (int i = 0; i < SEL_PINS; ++i) {
        gpio_pin_set(cfg->bits[i].port, cfg->bits[i].pin, 0);
    }
    gpio_pin_configure_dt(&cfg->key, GPIO_DISCONNECTED);
    gpio_pin_set(cfg->power.port, cfg->power.pin, 0);
    gpio_pin_set(cfg->strobe.port, cfg->strobe.pin, 0);

    if (cfg->is_jp) {
        gpio_pin_set(cfg->row_enable_low.port, cfg->row_enable_low.pin, 1);
        gpio_pin_set(cfg->row_enable_high.port, cfg->row_enable_high.pin, 1);
    }

    for (int r = 0; r < matrix_rows; ++r) {
        for (int c = 0; c < matrix_cols; ++c) {
            int cell = (r * matrix_cols) + c;
            if (data->matrix_state[cell] != matrix_read[cell]) {
                data->matrix_state[cell] = matrix_read[cell];
#if IS_ENABLED(CONFIG_ZMK_KSCAN_GPIO_TOPRE_DEBUG)
                LOG_INF("Key %s at row=%d col=%d (cell=%d)",
                        matrix_read[cell] ? "PRESS" : "RELEASE", r, c, cell);
#endif
                data->callback(data->dev, r, c, matrix_read[cell]);
            }
        }
    }
}

static int kscan_gpio_topre_init(const struct device *dev) {
    LOG_DBG("KSCAN init");
    struct kscan_gpio_topre_data *data = dev->data;
    const struct kscan_gpio_topre_config *cfg = dev->config;
    int err;

    for (int i = 0; i < SEL_PINS; ++i) {
        if (!gpio_is_ready_dt(&cfg->bits[i])) {
            LOG_ERR("GPIO port for bit %d is not ready", i);
            return -ENODEV;
        }
        err = gpio_pin_configure_dt(&cfg->bits[i], GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR("Failed to configure bit %d pin: %d", i, err);
            return err;
        }
    }

    if (!gpio_is_ready_dt(&cfg->power)) {
        LOG_ERR("GPIO port for power is not ready");
        return -ENODEV;
    }
    err = gpio_pin_configure_dt(&cfg->power, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure power pin: %d", err);
        return err;
    }

    if (!gpio_is_ready_dt(&cfg->key)) {
        LOG_ERR("GPIO port for key is not ready");
        return -ENODEV;
    }
    err = gpio_pin_configure_dt(&cfg->key, GPIO_DISCONNECTED);
    if (err) {
        LOG_ERR("Failed to configure key pin: %d", err);
        return err;
    }

    if (!gpio_is_ready_dt(&cfg->hys)) {
        LOG_ERR("GPIO port for hys is not ready");
        return -ENODEV;
    }
    err = gpio_pin_configure_dt(&cfg->hys, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure hys pin: %d", err);
        return err;
    }

    if (!gpio_is_ready_dt(&cfg->strobe)) {
        LOG_ERR("GPIO port for strobe is not ready");
        return -ENODEV;
    }
    err = gpio_pin_configure_dt(&cfg->strobe, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure strobe pin: %d", err);
        return err;
    }

    /* Initialize JP-specific pins if present */
    if (cfg->is_jp) {
        if (!gpio_is_ready_dt(&cfg->row_enable_low)) {
            LOG_ERR("GPIO port for row_enable_low is not ready");
            return -ENODEV;
        }
        err = gpio_pin_configure_dt(&cfg->row_enable_low, GPIO_OUTPUT_HIGH); /* Inactive (active low) */
        if (err) {
            LOG_ERR("Failed to configure row_enable_low pin: %d", err);
            return err;
        }

        if (!gpio_is_ready_dt(&cfg->row_enable_high)) {
            LOG_ERR("GPIO port for row_enable_high is not ready");
            return -ENODEV;
        }
        err = gpio_pin_configure_dt(&cfg->row_enable_high, GPIO_OUTPUT_HIGH); /* Inactive (active low) */
        if (err) {
            LOG_ERR("Failed to configure row_enable_high pin: %d", err);
            return err;
        }
    }

    data->dev = dev;
    topre_dev = dev;  /* Store for activity listener */

    k_timer_init(&data->poll_timer, kscan_gpio_topre_timer_handler, NULL);
    k_work_init(&data->poll, kscan_gpio_topre_work_handler);

    /* Startup delay before first scan to let power rails stabilize */
    k_sleep(K_MSEC(100));

    return 0;
}

static DEVICE_API(kscan, kscan_gpio_topre_api) = {
    .config = kscan_gpio_topre_configure,
    .enable_callback = kscan_gpio_topre_enable,
    .disable_callback = kscan_gpio_topre_disable,
};

/* Helper macros for optional JP pins */
#define COND_JP_PIN(n, prop) \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prop), \
                (GPIO_DT_SPEC_INST_GET(n, prop)), \
                ({0}))

#define IS_JP(n) DT_INST_NODE_HAS_PROP(n, row_enable_low_gpios)

#define KSCAN_GPIO_TOPRE_INST(n)                                                                   \
    static struct kscan_gpio_topre_data kscan_gpio_topre_data_##n;                                 \
                                                                                                   \
    static const struct kscan_gpio_topre_config kscan_gpio_topre_config_##n = {                    \
        .bits =                                                                                    \
            {                                                                                      \
                GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 3),                                         \
                GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 4),                                         \
                GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 5),                                         \
                GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 6),                                         \
                GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 7),                                         \
                GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 8),                                         \
            },                                                                                     \
        .power = GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 0),                                        \
        .key = GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 1),                                          \
        .hys = GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 2),                                          \
        .strobe = GPIO_DT_SPEC_INST_GET_BY_IDX(n, gpios, 9),                                       \
        .row_enable_low = COND_JP_PIN(n, row_enable_low_gpios),                                    \
        .row_enable_high = COND_JP_PIN(n, row_enable_high_gpios),                                  \
        .is_jp = IS_JP(n),                                                                         \
        .matrix_rows = DT_INST_PROP_OR(n, matrix_rows, 8),                                         \
        .matrix_cols = DT_INST_PROP_OR(n, matrix_cols, 8),                                         \
        .matrix_relax_us = DT_INST_PROP(n, matrix_relax_us),                                       \
        .adc_read_settle_us = DT_INST_PROP(n, adc_read_settle_us),                                 \
        .active_polling_interval_ms = DT_INST_PROP(n, active_polling_interval_ms),                 \
        .idle_polling_interval_ms = DT_INST_PROP(n, idle_polling_interval_ms),                     \
        .sleep_polling_interval_ms = DT_INST_PROP(n, sleep_polling_interval_ms),                   \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, kscan_gpio_topre_init, NULL, &kscan_gpio_topre_data_##n,              \
                          &kscan_gpio_topre_config_##n, POST_KERNEL,                               \
                          CONFIG_APPLICATION_INIT_PRIORITY, &kscan_gpio_topre_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_GPIO_TOPRE_INST)

/* Activity state listener - adjust polling rate based on activity */
static int topre_activity_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (!ev || !topre_dev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* Only handle active/idle transitions - sleep is handled by sys_poweroff */
    if (ev->state == ZMK_ACTIVITY_ACTIVE || ev->state == ZMK_ACTIVITY_IDLE) {
        kscan_gpio_topre_update_polling(topre_dev, ev->state);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(topre_activity, topre_activity_listener);
ZMK_SUBSCRIPTION(topre_activity, zmk_activity_state_changed);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
