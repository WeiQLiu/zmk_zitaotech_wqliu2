/*
 * TrackPoint HID over I2C Driver (Zephyr Input Subsystem)
 * Interrupt-driven version (Successfully Restored to Old-Version Speed & Curve)
 * Copyright (c) 2025 ZitaoTech
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_trackpoint

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zmk/hid.h>

#include "custom_led.h"

LOG_MODULE_REGISTER(trackpoint, LOG_LEVEL_DBG);

/* ========= ⭐ TrackPoint 专用 Work Queue ========= */
#define TP_WORKQ_STACK_SIZE 2048
#define TP_WORKQ_PRIORITY 5

static struct k_mutex trackpoint_i2c_mutex;

K_THREAD_STACK_DEFINE(tp_workq_stack, TP_WORKQ_STACK_SIZE);
static struct k_work_q tp_workq;

/* ========= ⭐ 完美复刻：旧版本专属指数加速参数 ========= */
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
#define TP_EXP_BASE 1.04f
#define TP_SPEED_SCALE 0.14f
#define TP_MAX_MULT 3.0f
#endif

/* ========= Motion GPIO ========= */
#define MOTION_GPIO_NODE DT_NODELABEL(gpio0)
#define MOTION_GPIO_PIN 14
#define MOTION_GPIO_FLAGS (GPIO_ACTIVE_LOW | GPIO_PULL_UP)

/* ========= TrackPoint 常量 ========= */
#define TRACKPOINT_I2C_ADDR 0x15
#define TRACKPOINT_PACKET_LEN 7
#define TRACKPOINT_MAGIC_BYTE0 0x50

#define SLOW_KEY_MULTIPLIER 0.5f

/* ========= Watch Dog ========= */
static uint32_t last_activity_time = 0;
#define TRACKPOINT_WDT_TIMEOUT 200

/* ========= 全局状态 ========= */
static bool scroll_key_pressed = false;
static bool arrow_key_pressed = false;
static bool slow_key_pressed = false;

/* ==== HID indicators ==== */
static zmk_hid_indicators_t current_indicators;
#define HID_INDICATORS_CAPS_LOCK (1 << 1)

static int hid_indicators_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev) {
        current_indicators = ev->indicators;
    }
    return ZMK_EV_EVENT_BUBBLE;
}
ZMK_LISTENER(a320_hid_listener, hid_indicators_listener);
ZMK_SUBSCRIPTION(a320_hid_listener, zmk_hid_indicators_changed);

/* ========= 按键监听：34(Arrow), 36(Slow), 61(Space) ========= */
static int special_key_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev)
        return 0;

    if (ev->position == 34) {
        arrow_key_pressed = ev->state;
        LOG_INF("arrow position=34 %s", arrow_key_pressed ? "PRESSED" : "RELEASED");
    }

    if (ev->position == 61) {
        scroll_key_pressed = ev->state;
        LOG_INF("scroll position=61 %s", scroll_key_pressed ? "PRESSED" : "RELEASED");
    }

    if (ev->position == 36) {
        slow_key_pressed = ev->state;
        LOG_INF("slow_key position=36 %s", slow_key_pressed ? "PRESSED" : "RELEASED");
    }

    return 0;
}
ZMK_LISTENER(trackpoint_special_key_listener, special_key_listener_cb);
ZMK_SUBSCRIPTION(trackpoint_special_key_listener, zmk_position_state_changed);

struct trackpoint_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec motion_gpio;
};

struct trackpoint_data {
    const struct device *dev;
    struct k_work work;
    struct gpio_callback motion_cb_data;
    struct k_work_delayable enable_irq_work;
    uint32_t last_packet_time;
};

/* ========= ⭐ 完美复刻：旧版本的纯正指数加速算法 ========= */
#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
static inline float trackpoint_exponential_factor(int8_t dx, int8_t dy, uint32_t delta_ms) {
    if (delta_ms == 0) {
        delta_ms = 1;
    }

    float dist = fabsf(dx) + fabsf(dy);
    if (dist < 1.0f) {
        return 1.0f;
    }

    float speed = dist / (float)delta_ms;
    float mult = powf(TP_EXP_BASE, speed / TP_SPEED_SCALE);

    if (mult > TP_MAX_MULT) {
        mult = TP_MAX_MULT;
    }

    return mult;
}
#endif

/* ========= 读取数据包 ========= */
static int trackpoint_read_packet(const struct device *dev, int8_t *dx, int8_t *dy) {
    const struct trackpoint_config *cfg = dev->config;
    uint8_t buf[TRACKPOINT_PACKET_LEN] = {0};
    int ret;

    k_mutex_lock(&trackpoint_i2c_mutex, K_FOREVER);
    ret = i2c_read_dt(&cfg->i2c, buf, TRACKPOINT_PACKET_LEN);
    k_mutex_unlock(&trackpoint_i2c_mutex);

    if (ret < 0)
        return ret;

    if (buf[0] != TRACKPOINT_MAGIC_BYTE0)
        return -EIO;

    *dx = (int8_t)buf[2];
    *dy = (int8_t)buf[3];
    return 0;
}

/* ========= 核心中断 Work 回调函数 ========= */
static void trackpoint_work_cb(struct k_work *work) {
    struct trackpoint_data *data = CONTAINER_OF(work, struct trackpoint_data, work);
    const struct device *dev = data->dev;
    uint32_t now = k_uptime_get_32();

    /* ========= WATCHDOG 喂狗 ========= */
    if (now - last_activity_time > TRACKPOINT_WDT_TIMEOUT) {
        LOG_WRN("TrackPoint watchdog recovery");
        last_activity_time = now;
        return;
    }

    int8_t dx = 0, dy = 0;
    int ret = trackpoint_read_packet(dev, &dx, &dy);
    if (ret != 0) {
        LOG_WRN("TrackPoint I2C read failed (soft recover)");
        return;
    }

    last_activity_time = now;
    bool capslock = current_indicators & HID_INDICATORS_CAPS_LOCK;

    /* ========================================================================= */
    /* 1. 方向键模式 (按住 34 号键触发) —— 同样采用旧版无限制线性缩放逻辑修复      */
    /* ========================================================================= */
    if (arrow_key_pressed) {
        int16_t move_x = 0;
        int16_t move_y = 0;

        if (abs(dx) >= 2) {
            move_x = dx / 16; 
            if (move_x == 0) move_x = (dx > 0) ? 1 : -1;
        }
        if (abs(dy) >= 2) {
            move_y = dy / 16;
            if (move_y == 0) move_y = (dy > 0) ? 1 : -1;
        }

        if (move_x > 0) {
            input_report_key(dev, KEY_RIGHT, 1, true, K_NO_WAIT);
            input_report_key(dev, KEY_RIGHT, 0, true, K_NO_WAIT);
        } else if (move_x < 0) {
            input_report_key(dev, KEY_LEFT, 1, true, K_NO_WAIT);
            input_report_key(dev, KEY_LEFT, 0, true, K_NO_WAIT);
        }

        if (move_y > 0) {
            input_report_key(dev, KEY_DOWN, 1, true, K_NO_WAIT);
            input_report_key(dev, KEY_DOWN, 0, true, K_NO_WAIT);
        } else if (move_y < 0) {
            input_report_key(dev, KEY_UP, 1, true, K_NO_WAIT);
            input_report_key(dev, KEY_UP, 0, true, K_NO_WAIT);
        }

        if (move_x != 0 || move_y != 0) {
            k_sleep(K_MSEC(40)); 
        }

    /* ========================================================================= */
    /* 2. 滚轮模式 (按住 61 号 Space 或开启大写锁定) —— ⭐ 100% 完整移植旧版逻辑    */
    /* ========================================================================= */
    } else if (scroll_key_pressed || capslock) {
        int16_t scroll_x = 0;
        int16_t scroll_y = 0;

        // 1. 先处理 X 轴 (水平滚动) 旧版原版无轴锁定限制
        if (abs(dx) >= 2) {
            scroll_x = -dx / 32; 
            if (scroll_x == 0) scroll_x = (dx > 0) ? -1 : 1;
        }

        // 2. 再处理 Y 轴 (垂直滚动) 旧版原版无轴锁定限制
        if (abs(dy) >= 2) {
            scroll_y = -dy / 16; 
            if (scroll_y == 0) scroll_y = (dy > 0) ? -1 : 1;
        }

        // 直接上报相对滚轮事件
        input_report_rel(dev, INPUT_REL_HWHEEL, scroll_x, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_WHEEL, -scroll_y, true, K_NO_WAIT);

        // 维持旧版最终改定的 30ms 顺滑节奏发包
        if (scroll_x != 0 || scroll_y != 0) {
            k_sleep(K_MSEC(30)); 
        }

    /* ========================================================================= */
    /* 3. 正常鼠标移动模式 —— ⭐ 100% 完整移植旧版放大速度与旧版指数曲线参数      */
    /* ========================================================================= */
    } else {
        uint8_t tp_led_brt = custom_led_get_last_valid_brightness();
        // 旧版基础因子计算
        float tp_factor = 0.6f + 0.02f * tp_led_brt;

#ifdef CONFIG_TRACKPOINT_EXPONENTIAL
        uint32_t delta = now - data->last_packet_time;
        // 使用上面完全复刻旧版的 powf 指数加速算法
        float exp_mult = trackpoint_exponential_factor(dx, dy, delta);
#else
        float exp_mult = 1.0f;
#endif

        // 36号按键按下速度直接减半
        float slow_mult = slow_key_pressed ? SLOW_KEY_MULTIPLIER : 1.0f;

        // 完全采用旧版公式：5/2 * dx * 0.5 * tp_factor * exp_mult * slow_mult
        float fx = 2.5f * dx * 0.5f * tp_factor * exp_mult * slow_mult;
        float fy = 2.5f * dy * 0.5f * tp_factor * exp_mult * slow_mult;

        input_report_rel(dev, INPUT_REL_X, -(int)fx, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, -(int)fy, true, K_NO_WAIT);
    }

    data->last_packet_time = now;
}

/* ========= GPIO 中断接收服务 ========= */
static void motion_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct trackpoint_data *data = CONTAINER_OF(cb, struct trackpoint_data, motion_cb_data);
    last_activity_time = k_uptime_get_32();
    
    k_work_submit_to_queue(&tp_workq, &data->work);
}

static void trackpoint_enable_irq_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct trackpoint_data *data = CONTAINER_OF(dwork, struct trackpoint_data, enable_irq_work);
    const struct device *dev = data->dev;
    const struct trackpoint_config *cfg = dev->config;

    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    LOG_INF("TrackPoint IRQ enabled (delayed)");
}

/* ========= 初始化函数 ========= */
static int trackpoint_init(const struct device *dev) {
    const struct trackpoint_config *cfg = dev->config;
    struct trackpoint_data *data = dev->data;
    if (!i2c_is_ready_dt(&cfg->i2c))
        return -ENODEV;
    if (!gpio_is_ready_dt(&cfg->motion_gpio))
        return -ENODEV;

    k_mutex_init(&trackpoint_i2c_mutex);

    data->dev = dev;
    data->last_packet_time = k_uptime_get_32();

    k_work_init(&data->work, trackpoint_work_cb);

    k_work_queue_start(&tp_workq, tp_workq_stack, K_THREAD_STACK_SIZEOF(tp_workq_stack),
                       TP_WORKQ_PRIORITY, NULL);

    gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);

    gpio_init_callback(&data->motion_cb_data, motion_isr, BIT(cfg->motion_gpio.pin));
    gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb_data);

    k_work_init_delayable(&data->enable_irq_work, trackpoint_enable_irq_work_cb);
    k_work_schedule(&data->enable_irq_work, K_MSEC(200));

    LOG_INF("TrackPoint Driver Initialized (Old Pure Config Mode)");
    return 0;
}

#define TRACKPOINT_DEFINE(inst)                                                                    \
    static struct trackpoint_data trackpoint_data_##inst;                                          \
    static const struct trackpoint_config trackpoint_config_##inst = {                             \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = {.port = DEVICE_DT_GET(MOTION_GPIO_NODE),                                    \
                        .pin = MOTION_GPIO_PIN,                                                     \
                        .dt_flags = MOTION_GPIO_FLAGS},                                             \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, trackpoint_init, NULL, &trackpoint_data_##inst,                    \
                          &trackpoint_config_##inst, POST_KERNEL, 70, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPOINT_DEFINE);
