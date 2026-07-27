/*
 * key.h — 基于 MultiButton 的事件驱动按键驱动 (MSPM0G3507)
 *
 * 通过 5ms 定时轮询驱动状态机, 支持每个按键独立消抖、单击、双击、
 * 长按检测。事件通过回调分发, 主循环无需阻塞等待按键释放。
 */

#ifndef __KEY_H
#define __KEY_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ---- 按键事件类型 (MultiButton 兼容) ---- */
typedef enum {
    PRESS_DOWN = 0,      /* 按下 */
    PRESS_UP,            /* 释放 */
    PRESS_REPEAT,        /* 持续连发 */
    SINGLE_CLICK,        /* 单击 */
    DOUBLE_CLICK,        /* 双击 */
    LONG_PRESS_START,    /* 长按开始 (按住 2s 触发) */
    LONG_PRESS_HOLD,     /* 长按保持 (每 100ms 重复) */
    NUMBER_OF_EVENT
} PressEvent;

/* 按键回调函数指针 */
typedef void (*BtnCallback)(void *arg);

/* ---- 按键控制块 ---- */
typedef struct Button {
    uint16_t ticks;                  /* 当前状态累计 tick 数 */
    uint32_t wait_double_ms;         /* 首次释放后开始等待第二击的真实时间戳 */
    uint8_t  state        : 3;       /* 状态机当前状态 */
    uint8_t  debounce_cnt : 3;       /* 消抖计数器 */
    uint8_t  is_double    : 1;       /* 双击序列标记 (第二次按下时置 1) */
    uint8_t  active_level : 1;       /* 按下有效电平 (1=高有效, 0=低有效) */
    uint8_t  repeat;                 /* 长按连发倒计时 */
    uint8_t  event;                  /* 最近一次触发的事件 */
    uint8_t  (*pin_read)(void);      /* 读取引脚电平的函数指针 */
    BtnCallback cb[NUMBER_OF_EVENT]; /* 事件回调表 */
    void       *cb_arg;              /* 回调参数 */
    struct Button *next;             /* 链表下一节点 */
} Button;

/* ---- 全局按键实例 ---- */
extern Button btn_key1;   /* PB23 — 导航键: 单击下一页 / 双击上一页 / 长按归零 */
extern Button btn_key2;   /* PB27 — 动作键: 单击执行页面特定功能 */

/* ---- 公开 API ---- */
void key_init(void);                    /* 初始化按键驱动 (链接链表)       */
void key_ticks(void);                   /* 状态机轮询 (需在主循环调用)    */
void button_attach(Button *handle, PressEvent event, BtnCallback cb, void *arg);
                                        /* 注册事件回调                    */
void button_start(Button *handle);      /* 启动按键检测                    */
void button_stop(Button *handle);       /* 停止按键检测                    */

#endif /* __KEY_H */
