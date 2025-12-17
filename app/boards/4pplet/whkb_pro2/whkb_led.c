/*
 * Copyright (c) 2025 Stefan Sundin (4pplet)
 *
 * SPDX-License-Identifier: MIT
 *
 * WHKB LED Indicator Driver
 *
 * LED Layout: [LEFT] [MIDDLE] [RIGHT]
 *             (led3)  (led1)   (led2)
 *
 * LED Behavior:
 * - Left LED (Profile): Blink pattern indicates profile number
 *     Profile 0: 1 short blink
 *     Profile 1: 2 short blinks
 *     Profile 2: 3 short blinks
 *     Profile 3: 1 long blink
 *     Profile 4: 2 long blinks
 * - Middle LED (Pairing): Blinks when profile is open (advertising)
 * - Right LED (Battery): Battery status indicator
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

/* LED nodes: left (led3), middle (led1), right (led2) */
#define LED_LEFT_NODE DT_ALIAS(led3)    /* Profile indicator */
#define LED_MIDDLE_NODE DT_ALIAS(led1)  /* Pairing indicator */
#define LED_RIGHT_NODE DT_ALIAS(led2)   /* Battery indicator */
#define CHG_NODE DT_ALIAS(chg_status)

/* Timing (ms) */
#define SHORT_BLINK_ON      150
#define SHORT_BLINK_OFF     200
#define LONG_BLINK_ON       500
#define LONG_BLINK_OFF      200
#define PAIRING_BLINK       300   /* Pairing LED blink interval */
#define LOW_BATT_BLINK      500   /* Low battery LED blink interval */
#define LOW_BATT_THRESHOLD  15

#if !DT_NODE_HAS_STATUS(LED_LEFT_NODE, okay) || \
    !DT_NODE_HAS_STATUS(LED_MIDDLE_NODE, okay) || \
    !DT_NODE_HAS_STATUS(LED_RIGHT_NODE, okay)
#error "led1, led2, led3 aliases required"
#endif

static const struct gpio_dt_spec led_left = GPIO_DT_SPEC_GET(LED_LEFT_NODE, gpios);
static const struct gpio_dt_spec led_middle = GPIO_DT_SPEC_GET(LED_MIDDLE_NODE, gpios);
static const struct gpio_dt_spec led_right = GPIO_DT_SPEC_GET(LED_RIGHT_NODE, gpios);

#if DT_NODE_HAS_STATUS(CHG_NODE, okay)
static const struct gpio_dt_spec chg_pin = GPIO_DT_SPEC_GET(CHG_NODE, gpios);
#define HAS_CHG_PIN 1
#else
#define HAS_CHG_PIN 0
#endif

/* State */
static uint8_t profile_idx;
static bool profile_open;
static bool usb_powered;
static bool is_asleep;

/* Profile blink sequence state */
static uint8_t blink_count;       /* Number of blinks remaining */
static bool blink_is_long;        /* Long vs short blink */
static bool blink_led_on;         /* Current LED state in sequence */
static bool sequence_active;      /* Profile sequence running */

/* Other indicators */
static bool pairing_led_on;
static bool battery_led_on;
static bool low_battery;          /* Battery below threshold */

static void led_left_set(bool on) {
    gpio_pin_configure_dt(&led_left, on ? GPIO_OUTPUT_HIGH : GPIO_DISCONNECTED);
}

static void led_middle_set(bool on) {
    gpio_pin_configure_dt(&led_middle, on ? GPIO_OUTPUT_HIGH : GPIO_DISCONNECTED);
}

static void led_right_set(bool on) {
    gpio_pin_configure_dt(&led_right, on ? GPIO_OUTPUT_HIGH : GPIO_DISCONNECTED);
}

static void all_leds_set(bool on) {
    led_left_set(on);
    led_middle_set(on);
    led_right_set(on);
}

static bool is_charging(void) {
#if HAS_CHG_PIN
    if (!device_is_ready(chg_pin.port)) {
        return false;
    }
    return gpio_pin_get_dt(&chg_pin) > 0;
#else
    return false;
#endif
}

/* Work items for each indicator */
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

    if (profile_idx <= 2) {
        blink_count = profile_idx + 1;  /* 1, 2, or 3 blinks */
        blink_is_long = false;
    } else {
        blink_count = profile_idx - 2;  /* 1 or 2 blinks */
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
        return;
    }

    uint16_t on_time = blink_is_long ? LONG_BLINK_ON : SHORT_BLINK_ON;
    uint16_t off_time = blink_is_long ? LONG_BLINK_OFF : SHORT_BLINK_OFF;

    if (blink_led_on) {
        /* LED was on, turn it off */
        blink_led_on = false;
        led_left_set(false);
        blink_count--;

        if (blink_count == 0) {
            /* Sequence complete */
            sequence_active = false;
            return;
        }

        k_work_schedule(&profile_work, K_MSEC(off_time));
    } else {
        /* LED was off, turn it on for next blink */
        blink_led_on = true;
        led_left_set(true);
        k_work_schedule(&profile_work, K_MSEC(on_time));
    }
}

/*
 * Pairing indicator on middle LED - blinks while profile is open (BLE only)
 */
static bool should_show_pairing(void) {
    /* Only show pairing indicator when on BLE and profile is open */
    if (!profile_open) return false;
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    /* Don't show pairing indicator when USB is connected */
    if (usb_powered) return false;
#endif
    return true;
}

static void start_pairing_indicator(void) {
    if (is_asleep || !should_show_pairing()) return;

    pairing_led_on = true;
    led_middle_set(true);
    k_work_schedule(&pairing_work, K_MSEC(PAIRING_BLINK));
}

static void stop_pairing_indicator(void) {
    k_work_cancel_delayable(&pairing_work);
    led_middle_set(false);
    pairing_led_on = false;
}

static void pairing_tick(struct k_work *work) {
    if (is_asleep || !should_show_pairing()) {
        led_middle_set(false);
        pairing_led_on = false;
        return;
    }

    pairing_led_on = !pairing_led_on;
    led_middle_set(pairing_led_on);
    k_work_schedule(&pairing_work, K_MSEC(PAIRING_BLINK));
}

/*
 * Battery indicator on right LED
 * - Solid: USB connected and charging
 * - Blinking: Low battery (not on USB)
 * - Off: Normal operation or fully charged
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

    if (usb_powered && is_charging()) {
        /* Charging: solid on */
        led_right_set(true);
        battery_led_on = true;
        /* Keep checking in case charging stops */
        k_work_schedule(&battery_led_work, K_MSEC(LOW_BATT_BLINK));
    } else if (usb_powered) {
        /* USB connected but not charging (full): off */
        led_right_set(false);
        battery_led_on = false;
        /* Keep checking in case charging resumes */
        k_work_schedule(&battery_led_work, K_MSEC(LOW_BATT_BLINK));
    } else if (low_battery) {
        /* Low battery on BLE: blink */
        battery_led_on = !battery_led_on;
        led_right_set(battery_led_on);
        k_work_schedule(&battery_led_work, K_MSEC(LOW_BATT_BLINK));
    } else {
        /* Normal operation: off */
        led_right_set(false);
        battery_led_on = false;
        /* Don't reschedule - will be restarted when state changes */
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

    if (profile_open) {
        start_pairing_indicator();
    } else {
        stop_pairing_indicator();
    }

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
        /* Restart indicators */
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
        /* USB just connected - update battery indicator, stop pairing */
        update_battery_indicator();
        stop_pairing_indicator();
    } else if (!usb_powered && was_powered) {
        /* USB just disconnected - update battery indicator, maybe start pairing */
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

    /* Update low battery state */
    low_battery = (level <= LOW_BATT_THRESHOLD);

    /* Update indicator if state changed */
    if (low_battery != was_low) {
        update_battery_indicator();
    }

    return ZMK_EV_EVENT_BUBBLE;
}
#endif

/*
 * Init
 */
static int whkb_led_init(void) {
    all_leds_set(false);

#if HAS_CHG_PIN
    if (device_is_ready(chg_pin.port)) {
        gpio_pin_configure_dt(&chg_pin, GPIO_INPUT);
    }
#endif

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

SYS_INIT(whkb_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* Event subscriptions */
ZMK_LISTENER(whkb_led_ble, on_ble_profile);
ZMK_LISTENER(whkb_led_activity, on_activity);

#if IS_ENABLED(CONFIG_ZMK_BLE)
ZMK_SUBSCRIPTION(whkb_led_ble, zmk_ble_active_profile_changed);
#endif
ZMK_SUBSCRIPTION(whkb_led_activity, zmk_activity_state_changed);

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
ZMK_LISTENER(whkb_led_usb, on_usb);
ZMK_SUBSCRIPTION(whkb_led_usb, zmk_usb_conn_state_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
ZMK_LISTENER(whkb_led_battery, on_battery);
ZMK_SUBSCRIPTION(whkb_led_battery, zmk_battery_state_changed);
#endif
