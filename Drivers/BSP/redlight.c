#include "redlight.h"
#include "timer.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "stdlib.h"
#include "bno08x_uart_rvc.h"
#include "key.h"
int track_mode=1;

static uint8_t read_gray(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

void read_redlight_pin(){
#if defined(redlight_PIN_7_PIN)
    p[0] = read_gray(redlight_PIN_7_PORT, redlight_PIN_7_PIN);
    p[1] = read_gray(redlight_PIN_6_PORT, redlight_PIN_6_PIN);
    p[2] = read_gray(redlight_PIN_5_PORT, redlight_PIN_5_PIN);
    p[3] = read_gray(redlight_PIN_4_PORT, redlight_PIN_4_PIN);
    p[4] = read_gray(redlight_PIN_3_PORT, redlight_PIN_3_PIN);
    p[5] = read_gray(redlight_PIN_2_PORT, redlight_PIN_2_PIN);
    p[6] = read_gray(redlight_PIN_1_PORT, redlight_PIN_1_PIN);
    p[7] = read_gray(redlight_PIN_0_PORT, redlight_PIN_0_PIN);
#else
    p[0] = read_gray(GPIOA, DL_GPIO_PIN_27);
    p[1] = read_gray(GPIOA, DL_GPIO_PIN_25);
    p[2] = read_gray(GPIOB, DL_GPIO_PIN_25);
    p[3] = read_gray(GPIOB, DL_GPIO_PIN_20);
    p[4] = read_gray(GPIOB, DL_GPIO_PIN_17);
    p[5] = read_gray(GPIOA, DL_GPIO_PIN_16);
    p[6] = read_gray(GPIOB, DL_GPIO_PIN_19);
    p[7] = 0U;
#endif
}
/**
 * 检查是否有3个及以上灰度传感器识别到信号
 * 返回值：1 - 检测到3个及以上信号；0 - 检测到少于3个信号
 */
int check_three_or_more() {
    // 先读取所有传感器的状态
    read_redlight_pin();
    
    // 统计高电平（1）的数量
    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (p[i] == 1) {
            count++;
        }
    }

    // 判断数量是否达到或超过3个
    return (count >= 3) ? 1 : 0;
}


