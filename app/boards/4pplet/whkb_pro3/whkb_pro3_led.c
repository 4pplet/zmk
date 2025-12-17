/*
 * Copyright (c) 2025 Stefan Sundin (4pplet)
 *
 * SPDX-License-Identifier: MIT
 *
 * WHKB Pro3 LED Indicator Driver (2 LEDs)
 *
 * LED Layout: [LEFT] [RIGHT]
 *             (led1)  (led2)
 *
 * LED Behavior:
 * - Left LED (Profile/Pairing):
 *     Blink pattern indicates profile number on change
 *     Continuous blink when profile is open (advertising)
 * - Right LED (Battery):
 *     Solid: Charging (USB connected + battery charging)
 *     Blinking: Low battery warning (below threshold, not on USB)
 *     Off: Normal operation or fully charged
 * - Sleep: All LEDs off
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/activity.h>

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zmk/usb.h>
#include <zmk/events/usb_conn_state_changed.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
#include <zmk/battery.h>
#include <zmk/events/battery_state_changed.h>
#endif

#define LED_LEFT_NODE DT_ALIAS(led1)   /* Profile/Pairing indicator */
#define LED_RIGHT_NODE DT_ALIAS(led2)  /* Battery indicator */

/* Timing (ms) */
#define SHORT_BLINK_ON      150
#define SHORT_BLINK_OFF     200
#define LONG_BLINK_ON       500
#define LONG_BLINK_OFF      200
#define PAIRING_BLINK       300
#define LOW_BATT_BLINK      500
#define LOW_BATT_THRESHOLD  15

#if !DT_NODE_HAS_STATUS(LED_LEFT_NODE, okay) || \
    !DT_NODE_HAS_STATUS(LED_RIGHT_NODE, okay)
#error "led1, led2 aliases required for Pro3"
#endif

static const struct gpio_dt_spec led_left = GPIO_DT_SPEC_GET(LED_LEFT_NODE, gpios);
static const struct gpio_dt_spec led_right = GPIO_DT_SPEC_GET(LED_RIGHT_NODE, gpios);

/* State */
static uint8_t profile_idx;
static bool profile_open;
static bool usb_powered;
static bool is_asleep;

/* Profile blink sequence state */
static uint8_t blink_count;
static bool blink_is_long;
static bool blink_led_on;
static bool sequence_active;

/* Other indicators */
static bool pairing_active;
static bool battery_led_on;
static bool low_battery;

static void led_left_set(bool on) {
    gpio_pin_configure_dt(&led_left, on ? GPIO_OUTPUT_HIGH : GPIO_DISCONNECTED);
}

static void led_right_set(bool on) {
    gpio_pin_configure_dt(&led_right, on ? GPIO_OUTPUT_HIGH : GPIO_DISCONNECTED);
}

static void all_leds_set(bool on) {
    led_left_set(on);
    led_right_set(on);
}

/* Work items */
static void profile_tick(struct k_work *work);
static void pairing_tick(struct k_work *work);
static void battery_led_tick(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(profile_work, profile_tick);
K_WORK_DELAYABLE_DEFINE(pairing_work, pairing_tick);
K_WORK_DELAYABLE_DEFINE(battery_led_work, battery_led_tick);

/*
 * Profile blink sequence on left LED
 * Profiles 0-2: 1-3 short blinks
 * Profiles 3-4: 1-2 long blinks
 */
static void start_profile_sequence(void) {
    if (is_asleep) return;

    k_work_cancel_delayable(&profile_work);
    k_work_cancel_delayable(&pairing_work);

    if (profile_idx <= 2) {
        blink_count = profile_idx + 1;
        blink_is_long = false;
    } else {
        blink_count = profile_idx - 2;
        blink_is_long = true;
    }

    blink_led_on = true;
    sequence_active = true;
    led_left_set(true);

    uint16_t on_time = blink_is_long ? LONG_BLINK_ON : SHORT_BLINK_ON;
    k_work_schedule(&profile_work, K_MSEC(on_time));
}

static void profile_tick(struct k_work *work) {
    if (is_asleep || !sequence_active) {
        led_left_set(false);
        sequence_active = false;
        /* Resume pairing indicator if needed */
        if (profile_open && !is_asleep) {
            k_work_schedule(&pairing_work, K_MSEC(PAIRING_BLINK));
        }
        return;
    }

    uint16_t on_time = blink_is_long ? LONG_BLINK_ON : SHORT_BLINK_ON;
    uint16_t off_time = blink_is_long ? LONG_BLINK_OFF : SHORT_BLINK_OFF;

    if (blink_led_on) {
        blink_led_on = false;
        led_left_set(false);
        blink_count--;

        if (blink_count == 0) {
            sequence_active = false;
            /* Resume pairing indicator if needed */
            if (profile_open && !is_asleep) {
                k_work_schedule(&pairing_work, K_MSEC(PAIRING_BLINK));
            }
            return;
        }

        k_work_schedule(&profile_work, K_MSEC(off_time));
    } else {
        blink_led_on = true;
        led_left_set(true);
        k_work_schedule(&profile_work, K_MSEC(on_time));
    }
}

/*
 * Pairing indicator on left LED - blinks while profile is open
 */
static bool should_show_pairing(void) {
    if (!profile_open) return false;
    if (sequence_active) return false;  /* Don't interrupt profile sequence */
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    if (usb_powered) return false;
#endif
    return true;
}

static void start_pairing_indicator(void) {
    if (is_asleep || !should_show_pairing()) return;

    pairing_active = true;
    led_left_set(true);
    k_work_schedule(&pairing_work, K_MSEC(PAIRING_BLINK));
}

static void stop_pairing_indicator(void) {
    k_work_cancel_delayable(&pairing_work);
    if (!sequence_active) {
        led_left_set(false);
    }
    pairing_active = false;
}

static void pairing_tick(struct k_work *work) {
    if (is_asleep || !should_show_pairing()) {
        led_left_set(false);
        pairing_active = false;
        return;
    }

    pairing_active = !pairing_active;
    led_left_set(pairing_active);
    k_work_schedule(&pairing_work, K_MSEC(PAIRING_BLINK));
}

/*
 * Battery indicator on right LED
 */
static void update_battery_indicator(void);

static void start_battery_indicator(void) {
    if (is_asleep) return;
    k_work_schedule(&battery_led_work, K_NO_WAIT);
}

static void stop_battery_indicator(void) {
    k_work_cancel_delayable(&battery_led_work);
    led_right_set(false);
    battery_led_on = false;
}

static void battery_led_tick(struct k_work *work) {
    if (is_asleep) {
        led_right_set(false);
        battery_led_on = false;
        return;
    }

    if (usb_powered) {
        /* USB connected: solid on */
        led_right_set(true);
        battery_led_on = true;
        k_work_schedule(&battery_led_work, K_MSEC(LOW_BATT_BLINK));
    } else if (low_battery) {
        /* Low battery: blink */
        battery_led_on = !battery_led_on;
        led_right_set(battery_led_on);
        k_work_schedule(&battery_led_work, K_MSEC(LOW_BATT_BLINK));
    } else {
        /* Normal: off */
        led_right_set(false);
        battery_led_on = false;
    }
}

static void update_battery_indicator(void) {
    if (is_asleep) return;

    if (usb_powered || low_battery) {
        start_battery_indicator();
    } else {
        stop_battery_indicator();
    }
}

/*
 * Event listeners
 */
static int on_ble_profile(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev = as_zmk_ble_active_profile_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    profile_idx = ev->index;
#if IS_ENABLED(CONFIG_ZMK_BLE)
    profile_open = zmk_ble_active_profile_is_open();
#endif

    start_profile_sequence();

    return ZMK_EV_EVENT_BUBBLE;
}

static int on_activity(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    if (ev->state == ZMK_ACTIVITY_SLEEP) {
        is_asleep = true;
        k_work_cancel_delayable(&profile_work);
        k_work_cancel_delayable(&pairing_work);
        k_work_cancel_delayable(&battery_led_work);
        all_leds_set(false);
        sequence_active = false;
    } else if (ev->state == ZMK_ACTIVITY_ACTIVE) {
        is_asleep = false;
        if (profile_open) {
            start_pairing_indicator();
        }
        update_battery_indicator();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
static int on_usb(const zmk_event_t *eh) {
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    bool was_powered = usb_powered;
    usb_powered = (ev->conn_state != ZMK_USB_CONN_NONE);

    if (is_asleep) return ZMK_EV_EVENT_BUBBLE;

    if (usb_powered && !was_powered) {
        update_battery_indicator();
        stop_pairing_indicator();
    } else if (!usb_powered && was_powered) {
        update_battery_indicator();
        if (profile_open) {
            start_pairing_indicator();
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
static int on_battery(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    uint8_t level = ev->state_of_charge;
    bool was_low = low_battery;

    low_battery = (level <= LOW_BATT_THRESHOLD);

    if (low_battery != was_low) {
        update_battery_indicator();
    }

    return ZMK_EV_EVENT_BUBBLE;
}
#endif

/*
 * Init
 */
static int whkb_pro3_led_init(void) {
    all_leds_set(false);

#if IS_ENABLED(CONFIG_ZMK_BLE)
    profile_idx = zmk_ble_active_profile_index();
    profile_open = zmk_ble_active_profile_is_open();
#endif

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    usb_powered = zmk_usb_is_powered();
#endif

    is_asleep = (zmk_activity_get_state() == ZMK_ACTIVITY_SLEEP);

    if (profile_open && !is_asleep) {
        start_pairing_indicator();
    }

    if (!is_asleep) {
        update_battery_indicator();
    }

    return 0;
}

SYS_INIT(whkb_pro3_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* Event subscriptions */
ZMK_LISTENER(whkb_pro3_led_ble, on_ble_profile);
ZMK_LISTENER(whkb_pro3_led_activity, on_activity);

#if IS_ENABLED(CONFIG_ZMK_BLE)
ZMK_SUBSCRIPTION(whkb_pro3_led_ble, zmk_ble_active_profile_changed);
#endif
ZMK_SUBSCRIPTION(whkb_pro3_led_activity, zmk_activity_state_changed);

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
ZMK_LISTENER(whkb_pro3_led_usb, on_usb);
ZMK_SUBSCRIPTION(whkb_pro3_led_usb, zmk_usb_conn_state_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
ZMK_LISTENER(whkb_pro3_led_battery, on_battery);
ZMK_SUBSCRIPTION(whkb_pro3_led_battery, zmk_battery_state_changed);
#endif
