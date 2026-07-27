/*
 * key.c — MultiButton 状态机按键驱动实现
 *
 * 状态机 (每 tick = 5ms):
 *   IDLE        → DEBOUNCE      (检测到引脚有效电平)
 *   DEBOUNCE    → PRESSED       (持续 3 tick = 15ms 确认按下, 触发 PRESS_DOWN)
 *   DEBOUNCE    → IDLE          (消抖期间无效 — 视为毛刺)
 *   PRESSED     → WAIT_DOUBLE   (短按释放, 触发 PRESS_UP, 等待可能的第二次点击)
 *   PRESSED     → LONG          (按住 ≥ 400 tick = 2s, 触发 LONG_PRESS_START)
 *   PRESSED     → IDLE          (长按后释放 — 仅 PRESS_UP, 不产生单击)
 *   LONG        → IDLE          (释放, 触发 PRESS_UP)
 *   LONG        → (repeat)      (每 REPEAT_TICKS 触发一次 LONG_PRESS_HOLD)
 *   WAIT_DOUBLE → DEBOUNCE      (400ms 内再次按下)
 *   WAIT_DOUBLE → IDLE          (超时 → 触发 SINGLE_CLICK)
 *
 * 双击流程: 第二次按下 -> 消抖 -> 释放时触发 DOUBLE_CLICK 回到 IDLE。
 */

#include "key.h"
#include "clock.h"   /* 使用 tick_ms 做 5ms 限速 */

/* ---- 时序参数 (以 5ms tick 为单位) ---- */
#define TICK_MS          5U      /* 每次 tick 间隔 5ms                   */
#define DEBOUNCE_TICKS   3U      /* 消抖时间 3 × 5ms = 15ms             */
#define DOUBLE_CLICK_MS  400U    /* 首次释放后等待第二击的真实时间窗口 */
#define LONG_TICKS       400U    /* 长按阈值 400 × 5ms = 2000ms (2s)    */
#define REPEAT_TICKS     20U     /* 长按保持重复间隔 20 × 5ms = 100ms   */

/* ---- 状态枚举 ---- */
enum {
    STATE_IDLE = 0,       /* 空闲, 等待按下 */
    STATE_DEBOUNCE,       /* 消抖中 */
    STATE_PRESSED,        /* 已确认按下 */
    STATE_LONG,           /* 长按保持 */
    STATE_WAIT_DOUBLE     /* 单击释放后等待双击 */
};

/* ---- 内部变量 ---- */
static uint32_t     last_tick_ms;  /* 上次 tick 时间戳 (用于限速) */
static Button      *head;          /* 按键链表头指针 */

/* 触发事件: 设置 event 字段, 若注册了回调则调用 */
static void emit_event(Button *btn, PressEvent event)
{
    btn->event = (uint8_t)event;
    if (btn->cb[event] != NULL) {
        btn->cb[event](btn->cb_arg);
    }
}

/* ---- 引脚读取函数 (按键为低电平有效: 按下 = 0) ---- */

static uint8_t key1_pin_read(void)
{
    return (DL_GPIO_readPins(key_PORT, key_KEY1_PIN) == 0U) ? 1U : 0U;
}

static uint8_t key2_pin_read(void)
{
    return (DL_GPIO_readPins(key_PORT, key_KEY2_PIN) == 0U) ? 1U : 0U;
}

/* ---- 全局按键实例 ---- */

Button btn_key1 = {
    .ticks       = 0,
    .wait_double_ms = 0U,
    .state       = STATE_IDLE,
    .debounce_cnt = 0,
    .is_double   = 0,
    .active_level = 1,      /* pin_read 按下时返回 1 (实际为低电平有效, 读取时已取反) */
    .repeat       = 0,
    .event        = 0,
    .pin_read     = key1_pin_read,
    .cb           = { NULL },
    .cb_arg       = NULL,
    .next         = NULL,
};

Button btn_key2 = {
    .ticks       = 0,
    .wait_double_ms = 0U,
    .state       = STATE_IDLE,
    .debounce_cnt = 0,
    .is_double   = 0,
    .active_level = 1,
    .repeat       = 0,
    .event        = 0,
    .pin_read     = key2_pin_read,
    .cb           = { NULL },
    .cb_arg       = NULL,
    .next         = NULL,
};

/* ================================================================
 *  公开 API 实现
 * ================================================================ */

/* 初始化: 将两个按键链接成链表, 记录起始时间 */
void key_init(void)
{
    btn_key1.next = &btn_key2;
    btn_key2.next = NULL;
    head          = &btn_key1;
    last_tick_ms  = tick_ms;
}

/*
 * 按键状态机轮询 — 需在主循环中反复调用。
 * 内部通过 tick_ms 做 5ms 限速。
 */
void key_ticks(void)
{
    Button *btn;
    uint8_t level;

    /* 限速: 不到 5ms 直接返回 */
    if ((uint32_t)(tick_ms - last_tick_ms) < TICK_MS) {
        return;
    }
    last_tick_ms = (uint32_t)tick_ms;

    /* 遍历链表中的每个按键 */
    for (btn = head; btn != NULL; btn = btn->next) {
        level = btn->pin_read();
        btn->ticks++;

        switch (btn->state) {

        /* -------------------------------------------------------- */
        case STATE_IDLE:            /* 等待按下 */
            if (level == btn->active_level) {
                btn->state        = STATE_DEBOUNCE;
                btn->debounce_cnt = 0;
                btn->ticks        = 0;
                btn->is_double    = 0;   /* 清除双击标记 */
            }
            break;

        /* -------------------------------------------------------- */
        case STATE_DEBOUNCE:        /* 消抖 */
            if (level != btn->active_level) {
                /* 弹跳 — 双击序列回 WAIT_DOUBLE (重置窗口计时),
                 * 否则回空闲。ticks 必须归零, 否则抖动会消耗双击窗口。 */
                btn->state = btn->is_double ? STATE_WAIT_DOUBLE : STATE_IDLE;
                btn->ticks = 0;
                if (btn->is_double) {
                    btn->wait_double_ms = (uint32_t)tick_ms;
                }
                break;
            }
            btn->debounce_cnt++;
            if (btn->debounce_cnt >= DEBOUNCE_TICKS) {
                /* 消抖确认: 按键已稳定按下 */
                btn->state = STATE_PRESSED;
                btn->ticks = 0;
                emit_event(btn, PRESS_DOWN);
            }
            break;

        /* -------------------------------------------------------- */
        case STATE_PRESSED:         /* 已按下, 监测释放或长按 */
            if (level != btn->active_level) {
                /* 释放 */
                emit_event(btn, PRESS_UP);

                if (btn->ticks >= LONG_TICKS) {
                    /* 长按后释放 — 不产生单击/双击, 清理状态 */
                    btn->state     = STATE_IDLE;
                    btn->is_double = 0;
                } else if (btn->is_double) {
                    /* 双击中的第二次释放 — 触发双击事件 */
                    emit_event(btn, DOUBLE_CLICK);
                    btn->state     = STATE_IDLE;
                    btn->is_double = 0;
                } else {
                    /* 短按释放 — 进入等待双击窗口 */
                    btn->state          = STATE_WAIT_DOUBLE;
                    btn->ticks          = 0;
                    btn->wait_double_ms = (uint32_t)tick_ms;
                }
            } else {
                /* 仍然按住 */
                if (btn->ticks == LONG_TICKS) {
                    /* 达到长按阈值 */
                    btn->state = STATE_LONG;
                    emit_event(btn, LONG_PRESS_START);
                    btn->repeat = REPEAT_TICKS;
                }
            }
            break;

        /* -------------------------------------------------------- */
        case STATE_LONG:            /* 长按保持 */
            if (level != btn->active_level) {
                emit_event(btn, PRESS_UP);
                btn->state     = STATE_IDLE;
                btn->is_double = 0;
            } else {
                btn->repeat--;
                if (btn->repeat == 0U) {
                    emit_event(btn, LONG_PRESS_HOLD);
                    btn->repeat = REPEAT_TICKS;
                }
            }
            break;

        /* -------------------------------------------------------- */
        case STATE_WAIT_DOUBLE:     /* 等待双击 */
            if ((uint32_t)(tick_ms - btn->wait_double_ms) >= DOUBLE_CLICK_MS) {
                /* 时间窗口已过，即使本次采样恰好读到按下也不能算双击。 */
                emit_event(btn, SINGLE_CLICK);
                btn->state     = STATE_IDLE;
                btn->is_double = 0;
            } else if (level == btn->active_level) {
                /* 在窗口内再次按下 → 双击序列 */
                btn->state        = STATE_DEBOUNCE;
                btn->debounce_cnt = 0;
                btn->ticks        = 0;
                btn->is_double    = 1;
            }
            break;

        } /* switch(state) */
    }     /* for each button */
}

/* 注册事件回调 */
void button_attach(Button *handle, PressEvent event, BtnCallback cb, void *arg)
{
    if (handle != NULL && event < NUMBER_OF_EVENT) {
        handle->cb[event] = cb;
        handle->cb_arg    = arg;
    }
}

/* 启动按键检测 (重置状态到空闲) */
void button_start(Button *handle)
{
    if (handle != NULL) {
        handle->state        = STATE_IDLE;
        handle->ticks        = 0;
        handle->wait_double_ms = 0U;
        handle->debounce_cnt = 0;
        handle->is_double    = 0;
        handle->repeat       = 0;
    }
}

/* 停止按键检测 (重置状态到空闲) */
void button_stop(Button *handle)
{
    if (handle != NULL) {
        handle->state        = STATE_IDLE;
        handle->ticks        = 0;
        handle->wait_double_ms = 0U;
        handle->debounce_cnt = 0;
        handle->is_double    = 0;
        handle->repeat       = 0;
    }
}
