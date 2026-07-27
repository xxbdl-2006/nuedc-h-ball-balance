#include "motor.h"
#include "ti_msp_dl_config.h"

#define TB6612_PWM_PERIOD 1000

static uint32_t motor_compare_from_duty(int duty)
{
    if (duty < 0) {
        duty = -duty;
    }

    if (duty > TB6612_PWM_PERIOD) {
        duty = TB6612_PWM_PERIOD;
    }

    /*
     * SysConfig uses edge-aligned down counting with CCx=period as 0% duty.
     * Keep the public duty range as 0..1000, and write the compare value
     * expected by that non-inverted PWM setup.
     */
   return (uint32_t)duty;
}

static void motor_set_forward_dir(void)
{
    DL_GPIO_clearPins(tb6612_PORT, tb6612_BIN2_PIN);
    DL_GPIO_setPins(tb6612_PORT, tb6612_BIN1_PIN);
    DL_GPIO_clearPins(tb6612_PORT, tb6612_AIN2_PIN);
    DL_GPIO_setPins(tb6612_PORT, tb6612_AIN1_PIN);
}

static void motor_set_reverse_dir(void)
{
    DL_GPIO_setPins(tb6612_PORT, tb6612_BIN2_PIN);
    DL_GPIO_clearPins(tb6612_PORT, tb6612_BIN1_PIN);
    DL_GPIO_setPins(tb6612_PORT, tb6612_AIN2_PIN);
    DL_GPIO_clearPins(tb6612_PORT, tb6612_AIN1_PIN);
}

static void motor_restore_pwm_pins(void)
{
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_tb6612_C0_IOMUX, GPIO_PWM_tb6612_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_tb6612_C0_PORT, GPIO_PWM_tb6612_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_PWM_tb6612_C1_IOMUX, GPIO_PWM_tb6612_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_tb6612_C1_PORT, GPIO_PWM_tb6612_C1_PIN);
    DL_TimerA_startCounter(PWM_tb6612_INST);
}

void motor_force_pwm_gpio(uint8_t enable, uint8_t reverse)
{
    DL_GPIO_initDigitalOutput(GPIO_PWM_tb6612_C0_IOMUX);
    DL_GPIO_initDigitalOutput(GPIO_PWM_tb6612_C1_IOMUX);
    DL_GPIO_enableOutput(GPIO_PWM_tb6612_C0_PORT, GPIO_PWM_tb6612_C0_PIN);
    DL_GPIO_enableOutput(GPIO_PWM_tb6612_C1_PORT, GPIO_PWM_tb6612_C1_PIN);

    if (reverse != 0U) {
        motor_set_reverse_dir();
    } else {
        motor_set_forward_dir();
    }

    if (enable != 0U) {
        DL_GPIO_setPins(GPIO_PWM_tb6612_C0_PORT, GPIO_PWM_tb6612_C0_PIN);
        DL_GPIO_setPins(GPIO_PWM_tb6612_C1_PORT, GPIO_PWM_tb6612_C1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_PWM_tb6612_C0_PORT, GPIO_PWM_tb6612_C0_PIN);
        DL_GPIO_clearPins(GPIO_PWM_tb6612_C1_PORT, GPIO_PWM_tb6612_C1_PIN);
    }
}

void motor_set_raw_compare(uint32_t left_compare, uint32_t right_compare)
{
    if (left_compare > TB6612_PWM_PERIOD) {
        left_compare = TB6612_PWM_PERIOD;
    }

    if (right_compare > TB6612_PWM_PERIOD) {
        right_compare = TB6612_PWM_PERIOD;
    }

    motor_restore_pwm_pins();
    motor_set_forward_dir();
    DL_TimerA_setCaptureCompareValue(PWM_tb6612_INST,
        left_compare, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_tb6612_INST,
        right_compare, DL_TIMER_CC_1_INDEX);
}

// 初始化电机控制引脚
void motor_init(){
    motor_restore_pwm_pins();
    motor_set_reverse_dir();
    set_right_PWM(0);
    set_left_PWM(0);
}

// 设置右侧电机的 PWM 占空比
void set_right_PWM(int duty){
    if (duty < 0) {
        DL_GPIO_setPins(tb6612_PORT, tb6612_BIN2_PIN);
        DL_GPIO_clearPins(tb6612_PORT, tb6612_BIN1_PIN);
        duty = -duty; // 取绝对值
    } else {
        DL_GPIO_clearPins(tb6612_PORT, tb6612_BIN2_PIN);
        DL_GPIO_setPins(tb6612_PORT, tb6612_BIN1_PIN);
    }
    motor_restore_pwm_pins();
    DL_TimerA_setCaptureCompareValue(PWM_tb6612_INST,
        motor_compare_from_duty(duty), DL_TIMER_CC_1_INDEX);
}

// 设置左侧电机的 PWM 占空比
void set_left_PWM(int duty) {
    if (duty < 0) {
        DL_GPIO_setPins(tb6612_PORT, tb6612_AIN2_PIN);
        DL_GPIO_clearPins(tb6612_PORT, tb6612_AIN1_PIN);
        duty = -duty; // 取绝对值
    } else {
        DL_GPIO_clearPins(tb6612_PORT, tb6612_AIN2_PIN);
        DL_GPIO_setPins(tb6612_PORT, tb6612_AIN1_PIN);
    }
    motor_restore_pwm_pins();
    DL_TimerA_setCaptureCompareValue(PWM_tb6612_INST,
        motor_compare_from_duty(duty), DL_TIMER_CC_0_INDEX);
}
