#ifndef __motor_H
#define __motor_H

#include <stdint.h>

void motor_init();
void set_right_PWM(int duty);
void  set_left_PWM(int duty);
void motor_set_raw_compare(uint32_t left_compare, uint32_t right_compare);
void motor_force_pwm_gpio(uint8_t enable, uint8_t reverse);
#endif
