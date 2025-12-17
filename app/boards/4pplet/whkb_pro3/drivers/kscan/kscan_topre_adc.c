/*
 * Copyright (c) 2025 Stefan Sundin (4pplet)
 * SPDX-License-Identifier: MIT
 *
 * WHKB Pro3 Topre ADC-based kscan driver
 *
 * Uses ADC to read analog signal from OpAmp instead of comparator.
 * This allows for adjustable actuation thresholds.
 *
 * Sensing sequence per key:
 * 1. Select column via COL_A/B/C and U1_EN or U2_EN
 * 2. Enable OpAmp
 * 3. Discharge sample capacitor
 * 4. Drive row high
 * 5. Read ADC value
 * 6. Compare against threshold
 * 7. Disable OpAmp, drive row low
 */

#define DT_DRV_COMPAT zmk_kscan_topre_adc

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(kscan_topre_adc, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/activity.h>

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define MATRIX_ROWS 4
#define MATRIX_COLS 15

struct kscan_topre_adc_config {
    struct gpio_dt_spec rows[MATRIX_ROWS];
    struct gpio_dt_spec col_sel[3];  /* COL_A, COL_B, COL_C */
    struct gpio_dt_spec u1_en;
    struct gpio_dt_spec u2_en;
    struct gpio_dt_spec opamp_en;
    struct gpio_dt_spec discharge;
    struct gpio_dt_spec power;
    const struct adc_dt_spec adc_channel;
    uint16_t actuation_threshold_mv;
    uint16_t row_drive_us;
    uint16_t row_settle_us;
    uint16_t opamp_settle_us;
    uint16_t discharge_us;
    uint16_t active_polling_interval_ms;
    uint16_t idle_polling_interval_ms;
};

struct kscan_topre_adc_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_timer poll_timer;
    struct k_work poll;
    bool matrix_state[MATRIX_ROWS * MATRIX_COLS];
    int16_t adc_buffer;
    struct adc_sequence adc_seq;
};

static const struct device *topre_adc_dev;

static int kscan_topre_adc_configure(const struct device *dev, kscan_callback_t callback) {
    struct kscan_topre_adc_data *data = dev->data;
    if (!callback) {
        return -EINVAL;
    }
    data->callback = callback;
    return 0;
}

static int kscan_topre_adc_enable(const struct device *dev) {
    struct kscan_topre_adc_data *data = dev->data;
    const struct kscan_topre_adc_config *cfg = dev->config;
    k_timer_start(&data->poll_timer, K_MSEC(cfg->active_polling_interval_ms),
                  K_MSEC(cfg->active_polling_interval_ms));
    return 0;
}

static int kscan_topre_adc_disable(const struct device *dev) {
    struct kscan_topre_adc_data *data = dev->data;
    k_timer_stop(&data->poll_timer);
    return 0;
}

static void kscan_topre_adc_timer_handler(struct k_timer *timer) {
    struct kscan_topre_adc_data *data =
        CONTAINER_OF(timer, struct kscan_topre_adc_data, poll_timer);
    k_work_submit(&data->poll);
}

static inline void select_column(const struct kscan_topre_adc_config *cfg, uint8_t col) {
    uint8_t mux_addr;

    if (col < 8) {
        /* U1: columns 0-7 */
        mux_addr = col;
        gpio_pin_set_dt(&cfg->u1_en, 1);  /* Enable U1 (active low in DT) */
        gpio_pin_set_dt(&cfg->u2_en, 0);  /* Disable U2 */
    } else {
        /* U2: columns 8-14 */
        mux_addr = col - 8;
        gpio_pin_set_dt(&cfg->u1_en, 0);  /* Disable U1 */
        gpio_pin_set_dt(&cfg->u2_en, 1);  /* Enable U2 */
    }

    /* Set 3-bit column address */
    gpio_pin_set_dt(&cfg->col_sel[0], mux_addr & 0x01);
    gpio_pin_set_dt(&cfg->col_sel[1], mux_addr & 0x02);
    gpio_pin_set_dt(&cfg->col_sel[2], mux_addr & 0x04);
}

static inline void deselect_columns(const struct kscan_topre_adc_config *cfg) {
    gpio_pin_set_dt(&cfg->u1_en, 0);
    gpio_pin_set_dt(&cfg->u2_en, 0);
}

static inline int read_adc_mv(struct kscan_topre_adc_data *data,
                               const struct kscan_topre_adc_config *cfg) {
    int ret = adc_read(cfg->adc_channel.dev, &data->adc_seq);
    if (ret < 0) {
        return ret;
    }

    int32_t val_mv = data->adc_buffer;
    ret = adc_raw_to_millivolts_dt(&cfg->adc_channel, &val_mv);
    if (ret < 0) {
        return ret;
    }

    return (int)val_mv;
}

static bool matrix_read[MATRIX_ROWS * MATRIX_COLS];

static void kscan_topre_adc_work_handler(struct k_work *work) {
    struct kscan_topre_adc_data *data =
        CONTAINER_OF(work, struct kscan_topre_adc_data, poll);
    const struct device *dev = data->dev;
    const struct kscan_topre_adc_config *cfg = dev->config;

    /* Power on switch board */
    gpio_pin_set_dt(&cfg->power, 1);
    k_sleep(K_MSEC(5));

    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
        select_column(cfg, col);

        for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
            int cell = (row * MATRIX_COLS) + col;

            /* Sensing sequence */
            gpio_pin_set_dt(&cfg->opamp_en, 1);
            k_busy_wait(cfg->opamp_settle_us);

            /* Discharge capacitor */
            gpio_pin_set_dt(&cfg->discharge, 1);
            k_busy_wait(cfg->discharge_us);
            gpio_pin_set_dt(&cfg->discharge, 0);

            /* Drive row */
            gpio_pin_set_dt(&cfg->rows[row], 1);
            k_busy_wait(cfg->row_drive_us);

            /* Read ADC */
            int mv = read_adc_mv(data, cfg);

            /* Release row */
            gpio_pin_set_dt(&cfg->rows[row], 0);
            gpio_pin_set_dt(&cfg->opamp_en, 0);

            /* Higher ADC value = key pressed (per HHKB Classic reference) */
            matrix_read[cell] = (mv >= 0 && mv > cfg->actuation_threshold_mv);

            k_busy_wait(cfg->row_settle_us);
        }
    }

    deselect_columns(cfg);

    /* Power off switch board */
    gpio_pin_set_dt(&cfg->power, 0);

    /* Report changes */
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < MATRIX_COLS; col++) {
            int cell = (row * MATRIX_COLS) + col;
            if (data->matrix_state[cell] != matrix_read[cell]) {
                data->matrix_state[cell] = matrix_read[cell];
                data->callback(data->dev, row, col, matrix_read[cell]);
            }
        }
    }
}

static int kscan_topre_adc_init(const struct device *dev) {
    struct kscan_topre_adc_data *data = dev->data;
    const struct kscan_topre_adc_config *cfg = dev->config;
    int ret;

    data->dev = dev;
    topre_adc_dev = dev;

    /* Configure row pins */
    for (int i = 0; i < MATRIX_ROWS; i++) {
        if (!gpio_is_ready_dt(&cfg->rows[i])) {
            LOG_ERR("Row %d GPIO not ready", i);
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&cfg->rows[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) return ret;
    }

    /* Configure column select pins */
    for (int i = 0; i < 3; i++) {
        if (!gpio_is_ready_dt(&cfg->col_sel[i])) {
            LOG_ERR("Col select %d GPIO not ready", i);
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(&cfg->col_sel[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) return ret;
    }

    /* Configure mux enable pins */
    if (!gpio_is_ready_dt(&cfg->u1_en) || !gpio_is_ready_dt(&cfg->u2_en)) {
        LOG_ERR("Mux enable GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&cfg->u1_en, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return ret;
    ret = gpio_pin_configure_dt(&cfg->u2_en, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return ret;

    /* Configure OpAmp control pins */
    if (!gpio_is_ready_dt(&cfg->opamp_en) || !gpio_is_ready_dt(&cfg->discharge)) {
        LOG_ERR("OpAmp control GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&cfg->opamp_en, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return ret;
    ret = gpio_pin_configure_dt(&cfg->discharge, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return ret;

    /* Configure power pin */
    if (!gpio_is_ready_dt(&cfg->power)) {
        LOG_ERR("Power GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&cfg->power, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) return ret;

    /* Configure ADC */
    if (!adc_is_ready_dt(&cfg->adc_channel)) {
        LOG_ERR("ADC not ready");
        return -ENODEV;
    }
    ret = adc_channel_setup_dt(&cfg->adc_channel);
    if (ret < 0) {
        LOG_ERR("ADC channel setup failed: %d", ret);
        return ret;
    }

    /* Initialize ADC sequence */
    data->adc_seq = (struct adc_sequence){
        .buffer = &data->adc_buffer,
        .buffer_size = sizeof(data->adc_buffer),
    };
    ret = adc_sequence_init_dt(&cfg->adc_channel, &data->adc_seq);
    if (ret < 0) {
        LOG_ERR("ADC sequence init failed: %d", ret);
        return ret;
    }

    k_timer_init(&data->poll_timer, kscan_topre_adc_timer_handler, NULL);
    k_work_init(&data->poll, kscan_topre_adc_work_handler);

    LOG_INF("WHKB Pro3 Topre ADC kscan initialized (threshold: %dmV)",
            cfg->actuation_threshold_mv);
    return 0;
}

static DEVICE_API(kscan, kscan_topre_adc_api) = {
    .config = kscan_topre_adc_configure,
    .enable_callback = kscan_topre_adc_enable,
    .disable_callback = kscan_topre_adc_disable,
};

#define KSCAN_TOPRE_ADC_INST(n)                                                                    \
    static struct kscan_topre_adc_data kscan_topre_adc_data_##n;                                   \
                                                                                                   \
    static const struct kscan_topre_adc_config kscan_topre_adc_config_##n = {                      \
        .rows = {                                                                                  \
            GPIO_DT_SPEC_INST_GET_BY_IDX(n, row_gpios, 0),                                         \
            GPIO_DT_SPEC_INST_GET_BY_IDX(n, row_gpios, 1),                                         \
            GPIO_DT_SPEC_INST_GET_BY_IDX(n, row_gpios, 2),                                         \
            GPIO_DT_SPEC_INST_GET_BY_IDX(n, row_gpios, 3),                                         \
        },                                                                                         \
        .col_sel = {                                                                               \
            GPIO_DT_SPEC_INST_GET_BY_IDX(n, col_sel_gpios, 0),                                     \
            GPIO_DT_SPEC_INST_GET_BY_IDX(n, col_sel_gpios, 1),                                     \
            GPIO_DT_SPEC_INST_GET_BY_IDX(n, col_sel_gpios, 2),                                     \
        },                                                                                         \
        .u1_en = GPIO_DT_SPEC_INST_GET(n, u1_en_gpios),                                            \
        .u2_en = GPIO_DT_SPEC_INST_GET(n, u2_en_gpios),                                            \
        .opamp_en = GPIO_DT_SPEC_INST_GET(n, opamp_en_gpios),                                      \
        .discharge = GPIO_DT_SPEC_INST_GET(n, discharge_gpios),                                    \
        .power = GPIO_DT_SPEC_INST_GET(n, power_gpios),                                            \
        .adc_channel = ADC_DT_SPEC_INST_GET(n),                                                    \
        .actuation_threshold_mv = DT_INST_PROP(n, actuation_threshold_mv),                         \
        .row_drive_us = DT_INST_PROP(n, row_drive_us),                                             \
        .row_settle_us = DT_INST_PROP(n, row_settle_us),                                           \
        .opamp_settle_us = DT_INST_PROP(n, opamp_settle_us),                                       \
        .discharge_us = DT_INST_PROP(n, discharge_us),                                             \
        .active_polling_interval_ms = DT_INST_PROP_OR(n, active_polling_interval_ms, 8),           \
        .idle_polling_interval_ms = DT_INST_PROP_OR(n, idle_polling_interval_ms, 100),             \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, kscan_topre_adc_init, NULL, &kscan_topre_adc_data_##n,                \
                          &kscan_topre_adc_config_##n, POST_KERNEL,                                \
                          CONFIG_APPLICATION_INIT_PRIORITY, &kscan_topre_adc_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_TOPRE_ADC_INST)

/* Activity state listener - adjust polling rate */
static int topre_adc_activity_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (!ev || !topre_adc_dev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct kscan_topre_adc_data *data = topre_adc_dev->data;
    const struct kscan_topre_adc_config *cfg = topre_adc_dev->config;
    uint16_t interval_ms;

    switch (ev->state) {
    case ZMK_ACTIVITY_IDLE:
        interval_ms = cfg->idle_polling_interval_ms;
        break;
    case ZMK_ACTIVITY_ACTIVE:
    default:
        interval_ms = cfg->active_polling_interval_ms;
        break;
    }

    k_timer_start(&data->poll_timer, K_MSEC(interval_ms), K_MSEC(interval_ms));
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(topre_adc_activity, topre_adc_activity_listener);
ZMK_SUBSCRIPTION(topre_adc_activity, zmk_activity_state_changed);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
