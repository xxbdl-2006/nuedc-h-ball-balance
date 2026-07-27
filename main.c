/*
 * main.c — NUEDC 2026 MSPM0G3507 IMU 
 *
 * OLED 页面:
 *   第 0 页: BNO08X/BNO085 I2C/SHTP 数据流 (PB2/PB3)
 *   第 1 页: JY61P/WIT UART 角度数据流 (PB15/PB16)
 *   第 2 页: 8 路灰度传感器数字输入测试
 *   第 3 页: TB6612 电机低速启动与编码器测试
 *   第 4 页: JDY31 蓝牙 UART0 收发测试 (PA10/PA11)
 *
 * 按键操作:
 *   KEY1 (PB23): 单击→下一页 | 双击→上一页 | 长按2s→页面归零/重置
 *   KEY2 (PB27): 单击→当前页面特定动作 (BNO归零/电机切档/蓝牙发数)
 *
 * UART0 接收三 PID 调参命令，并可按需向 FireWater 输出 20Hz 波形。
 */

#include "ti_msp_dl_config.h"

#include "bno08x_uart_rvc.h"
#include "bsp_hc05.h"
#include "buzzer.h"
#include "clock.h"
#include "interrupt.h"
#include "key.h"
#include "oled_display.h"
#include "serial_debug.h"
#include "timer.h"
#include "wit.h"
#include "motor.h"
#include "encoder.h"
#include "pid_control.h"
/* ================================================================
 *  按键回调函数 — 由 key_ticks() 状态机事件驱动触发
 * ================================================================ */

/* KEY1 单击 → 切换到下一页 */
static void on_key1_click(void *arg)
{
    (void)arg;
    oled_display_set_last_key(1);
    oled_display_switch_page();
   // Buzzer_Beep(10);
}

/* KEY1 双击 → 切换回上一页 */
static void on_key1_double(void *arg)
{
    (void)arg;
    oled_display_set_last_key(1);
    oled_display_prev_page();
   // Buzzer_Beep(10);
}

/* KEY1 长按 2 秒 → 根据当前页面执行归零/重置动作 */
static void on_key1_long(void *arg)
{
    (void)arg;
    oled_display_set_last_key(2);
   // Buzzer_Beep(40);

    switch (oled_display_get_page()) {
    case PAGE_BNO08X:
        serial_debug_bno08x_action();   /* 重置 BNO08X 航向角 (或重新探测) */
        break;
    case PAGE_JY61P:
        WIT_RequestZero();              /* JY61P 角度归零 */
        break;
    case PAGE_MOTOR:
        /* 预留电机页面长按动作。 */
        break;
    case PAGE_BLUETOOTH:
        serial_debug_bluetooth_ping();  /* 发送蓝牙测试数据帧 */
        break;
    default:
        break;
    }
}

/* KEY2 单击 — 暂未使用 (保留占位) */
static void on_key2_click(void *arg)
{
    /* 预留: 可用于扩展 KEY2 功能 */
    (void)arg;
}

/* ================================================================
 *  主函数
 * ================================================================ */

int main(void)
{
    /* ---- 硬件 / SysConfig 初始化 ---- */
    SYSCFG_DL_init();   /* 由 SysConfig 工具生成的全部外设初始化代码 */ 
    SysTick_Init();     /* 1ms 系统滴答定时器, 驱动 tick_ms 全局变量 */

    
    /* ---- 各外设/模块初始化 (顺序有依赖: OLED→按键→传感器→串口) ---- */
    oled_display_init();    /* OLED 显示屏初始化, 显示启动画面 */
    Buzzer_Off();           /* 确保蜂鸣器初始为关闭状态 */
    Bluetooth_Init();       /* 蓝牙模块 UART 中断与缓冲区初始化 */
    BNO08X_Init();          /* BNO08X 姿态传感器 I2C 初始化 */
    WIT_Init();             /* JY61P 姿态传感器 UART 初始化 */
    Interrupt_Init();       /* 全局中断配置 (NVIC 分组等) */
    motor_init();            /* 电机初始化 */    
    encoder_init();          /* 编码器初始化 */
    serial_debug_init();    /* 串口调试输出, 发送启动消息 */
    pid_control_init();      /* 角度环 + 左右轮速度环初始化 */
    timer_init();            /* 最后启用 50ms 编码器与 PID 控制中断 */
    
    /* ---- 按键驱动注册 (将回调挂载到按键状态机) ---- */
    key_init();
    button_attach(&btn_key1, SINGLE_CLICK,     on_key1_click,  NULL);
    button_attach(&btn_key1, DOUBLE_CLICK,      on_key1_double, NULL);
    button_attach(&btn_key1, LONG_PRESS_START,  on_key1_long,   NULL);
    button_attach(&btn_key2, SINGLE_CLICK,      on_key2_click,  NULL);

    oled_display_refresh(); /* 默认显示第 0 页 (BNO08X) */
    // set_right_PWM(400); 
    // set_left_PWM(400);
    /* ---- 主循环: 非阻塞轮询调度 ---- */
    while (1) {
        key_ticks();            /* 按键状态机 (5ms 轮询, 消抖+事件分发) */
        Bluetooth_Poll();       /* 处理蓝牙 UART 软件接收队列           */
        BNO08X_Update();         /* BNO08X 自动更新(50ms)  */
        oled_display_tick();    /* OLED 定时刷新 (100ms)                 */
           
    }
}
