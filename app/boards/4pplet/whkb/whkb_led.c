/*
 * Copyright (c) 2025 Stefan Sundin (4pplet)
 *
 * SPDX-License-Identifier: MIT
 *
 * WHKB LED Indicator Driver
 *
 * LED Behavior:
 * - Profile switch: Shows profile number (1-5) as binary on LEDs for 3 seconds
 * - Pairing mode: Blinks profile pattern when profile is open (advertising)
 * - Charging: LED3 pulses when USB connected and charging
 * - Low battery: All LEDs flash 3x when battery falls below threshold
 * - Sleep: All LEDs off
 *
 * Profile LED patterns (binary: LED3=4, LED2=2, LED1=1):
 *   Profile 1: LED1 (001)
 *   Profile 2: LED2 (010)
 *   Profile 3: LED1+LED2 (011)
 *   Profile 4: LED3 (100)
 *   Profile 5: LED1+LED3 (101)
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

/* LED nodes */
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)
#define CHG_NODE DT_ALIAS(chg_status)

/* Timing (ms) */
#define PROFILE_TIMEOUT     3000
#define BLINK_INTERVAL      250     /* 250ms = 4Hz blink rate */
#define LOW_BATT_THRESHOLD  15

#if !DT_NODE_HAS_STATUS(LED1_NODE, okay) || \
    !DT_NODE_HAS_STATUS(LED2_NODE, okay) || \
    !DT_NODE_HAS_STATUS(LED3_NODE, okay)
#error "led1, led2, led3 aliases required"
#endif

static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(LED1_NODE, gpios),
    GPIO_DT_SPEC_GET(LED2_NODE, gpios),
    GPIO_DT_SPEC_GET(LED3_NODE, gpios),
};

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
static bool blink_on;
static uint8_t low_batt_flashes;
static uint8_t last_batt_level = 100;

/* Animation mode */
enum led_mode {
    LED_OFF,
    LED_PROFILE_SOLID,  /* Show profile for timeout period */
    LED_PROFILE_BLINK,  /* Blink profile (pairing) */
    LED_CHARGING,       /* Pulse LED3 */
    LED_LOW_BATTERY,    /* Flash all LEDs */
};
static enum led_mode current_mode = LED_OFF;
static int64_t mode_start_time;

/*
 * Set LEDs by bitmask: bit0=LED1, bit1=LED2, bit2=LED3
 */
static void leds_set(uint8_t mask) {
    for (int i = 0; i < 3; i++) {
        if (mask & (1 << i)) {
            gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_HIGH);
        } else {
            gpio_pin_configure_dt(&leds[i], GPIO_DISCONNECTED);
        }
    }
}

/* Profile index (0-4) to LED bitmask (1-5 in binary) */
static uint8_t profile_to_mask(uint8_t idx) {
    return (idx + 1) & 0x07;
}

static bool is_charging(void) {
#if HAS_CHG_PIN
    if (!device_is_ready(chg_pin.port)) return false;
    return gpio_pin_get_dt(&chg_pin) > 0;
#else
    return false;
#endif
}

/*
 * Single animation tick handler
 */
static void led_tick(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(led_work, led_tick);

static void led_tick(struct k_work *work) {
    if (is_asleep) {
        leds_set(0);
        return;
    }

    int64_t elapsed = k_uptime_get() - mode_start_time;

    switch (current_mode) {
    case LED_PROFILE_SOLID:
        if (elapsed >= PROFILE_TIMEOUT) {
            /* Timeout - check what to do next */
            if (profile_open) {
                current_mode = LED_PROFILE_BLINK;
                mode_start_time = k_uptime_get();
                blink_on = true;
            } else if (usb_powered && is_charging()) {
                current_mode = LED_CHARGING;
                mode_start_time = k_uptime_get();
                blink_on = true;
            } else {
                current_mode = LED_OFF;
                leds_set(0);
                return;
            }
        } else {
            leds_set(profile_to_mask(profile_idx));
            k_work_schedule(&led_work, K_MSEC(PROFILE_TIMEOUT - elapsed));
            return;
        }
        break;

    case LED_PROFILE_BLINK:
        /* Re-check pairing state in case event hasn't fired yet */
#if IS_ENABLED(CONFIG_ZMK_BLE)
        profile_open = zmk_ble_active_profile_is_open();
#endif
        if (!profile_open) {
            /* No longer pairing - device connected */
            if (usb_powered && is_charging()) {
                current_mode = LED_CHARGING;
                blink_on = true;
            } else {
                current_mode = LED_OFF;
                leds_set(0);
                return;
            }
        } else {
            blink_on = !blink_on;
            leds_set(blink_on ? profile_to_mask(profile_idx) : 0);
        }
        break;

    case LED_CHARGING:
        if (!usb_powered) {
            current_mode = LED_OFF;
            leds_set(0);
            return;
        }
        blink_on = !blink_on;
        leds_set(blink_on && is_charging() ? 0x04 : 0); /* LED3 only */
        break;

    case LED_LOW_BATTERY:
        blink_on = !blink_on;
        if (blink_on) {
            leds_set(0x07); /* All on */
            low_batt_flashes++;
        } else {
            leds_set(0);
        }
        if (low_batt_flashes >= 3) {
            current_mode = LED_OFF;
            leds_set(0);
            return;
        }
        break;

    case LED_OFF:
    default:
        leds_set(0);
        return;
    }

    k_work_schedule(&led_work, K_MSEC(BLINK_INTERVAL));
}

static void start_mode(enum led_mode mode) {
    if (is_asleep) return;

    k_work_cancel_delayable(&led_work);
    current_mode = mode;
    mode_start_time = k_uptime_get();
    blink_on = true;
    low_batt_flashes = 0;

    /* Immediate first tick */
    if (mode == LED_PROFILE_SOLID) {
        leds_set(profile_to_mask(profile_idx));
        k_work_schedule(&led_work, K_MSEC(PROFILE_TIMEOUT));
    } else {
        k_work_schedule(&led_work, K_NO_WAIT);
    }
}

/*
 * Event listeners
 */
static int on_ble_profile(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev = as_zmk_ble_active_profile_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    profile_idx = ev->index;
    profile_open = zmk_ble_active_profile_is_open();
    start_mode(LED_PROFILE_SOLID);

    return ZMK_EV_EVENT_BUBBLE;
}

static int on_activity(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    if (ev->state == ZMK_ACTIVITY_SLEEP) {
        is_asleep = true;
        k_work_cancel_delayable(&led_work);
        leds_set(0);
    } else if (ev->state == ZMK_ACTIVITY_ACTIVE) {
        is_asleep = false;
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

    /* Start charging indicator if USB just connected and not already animating */
    if (usb_powered && !was_powered && current_mode == LED_OFF) {
        if (is_charging()) {
            start_mode(LED_CHARGING);
        }
    } else if (!usb_powered && current_mode == LED_CHARGING) {
        current_mode = LED_OFF;
        leds_set(0);
    }

    return ZMK_EV_EVENT_BUBBLE;
}
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
static int on_battery(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (!ev) return ZMK_EV_EVENT_BUBBLE;

    uint8_t level = ev->state_of_charge;

    /* Trigger low battery warning on crossing threshold */
    if (level <= LOW_BATT_THRESHOLD &&
        last_batt_level > LOW_BATT_THRESHOLD &&
        !is_asleep && !usb_powered) {
        start_mode(LED_LOW_BATTERY);
    }

    last_batt_level = level;
    return ZMK_EV_EVENT_BUBBLE;
}
#endif

/*
 * Init
 */
static int whkb_led_init(void) {
    leds_set(0);

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

    LOG_INF("WHKB LED init");
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
