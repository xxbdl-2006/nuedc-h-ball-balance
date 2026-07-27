#include "pid_control.h"

#include <stdbool.h>

#include "motor.h"
#include "pid.h"
#include "ti_msp_dl_config.h"

#define ANGLE_CORRECTION_LIMIT PID_CONTROL_SPEED_TARGET_LIMIT
#define ANGLE_DEADBAND_DEGREES  2.0f
#define MOTOR_PWM_MAX          900.0f

/* PID 实例只由本模块管理，其他模块通过公开接口访问。 */
static pid_t angle_pid;
static pid_t motor_left_pid;
static pid_t motor_right_pid;

/* 方向：1=正转，-1=反转，0=目标为零。 */
static int8_t left_direction;
static int8_t right_direction;
static int32_t left_target_signed;
static int32_t right_target_signed;
static float common_speed_target;

/* 中断写入，主循环通过 pid_control_get_status() 原子读取。 */
static volatile pid_control_status_t control_status;

static uint32_t pid_control_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void pid_control_exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static int32_t round_symmetric(float value)
{
    return (value >= 0.0f)
        ? (int32_t)(value + 0.5f)
        : (int32_t)(value - 0.5f);
}

static int32_t round_nonnegative(float value)
{
    return (int32_t)(value + 0.5f);
}

static float limit_speed_target(float target)
{
    if (target > PID_CONTROL_SPEED_TARGET_LIMIT) {
        return PID_CONTROL_SPEED_TARGET_LIMIT;
    }
    if (target < -PID_CONTROL_SPEED_TARGET_LIMIT) {
        return -PID_CONTROL_SPEED_TARGET_LIMIT;
    }
    return target;
}

static float normalize_angle_degrees(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static bool angle_pid_gains_are_zero(void)
{
    return angle_pid.p == 0.0f &&
        angle_pid.i == 0.0f &&
        angle_pid.d == 0.0f;
}

static void set_one_motor_target(pid_t *pid, int8_t *direction,
    int32_t signed_target)
{
    if (signed_target > 0) {
        *direction = 1;
        pid->target = (float)signed_target;
    } else if (signed_target < 0) {
        *direction = -1;
        pid->target = (float)(-signed_target);
    } else {
        *direction = 0;
        pid->target = 0.0f;
    }
}

/* 保存两个车轮合成后的有符号目标，并提取各自方向。 */
static void motor_target_set(int32_t left_target, int32_t right_target)
{
    left_target_signed = left_target;
    right_target_signed = right_target;
    set_one_motor_target(&motor_left_pid, &left_direction, left_target);
    set_one_motor_target(&motor_right_pid, &right_direction, right_target);
}

static int32_t motor_pid_update(pid_t *pid, int8_t direction,
    int32_t encoder_count)
{
    if (direction == 0) {
        pid->target = 0.0f;
        pid_reset(pid);
        return 0;
    }

    /* 投影到目标方向：同向为正，反向为负。 */
    pid->now = (float)direction * (float)encoder_count;
    pid_cal(pid);
    pidout_limit(pid, 0.0f, MOTOR_PWM_MAX);

    return (int32_t)direction * round_nonnegative(pid->out);
}

static void pid_control_publish_status(float current_angle,
    int32_t enc_left, int32_t enc_right,
    int32_t left_output, int32_t right_output)
{
    control_status.angle_p = angle_pid.p;
    control_status.angle_i = angle_pid.i;
    control_status.angle_d = angle_pid.d;
    control_status.motor_p = motor_left_pid.p;
    control_status.motor_i = motor_left_pid.i;
    control_status.motor_d = motor_left_pid.d;
    control_status.angle_target = angle_pid.target;
    control_status.angle_now = current_angle;
    control_status.angle_out = angle_pid.out;
    control_status.speed_target = common_speed_target;
    control_status.left_target = left_target_signed;
    control_status.left_now = enc_left;
    control_status.left_output = left_output;
    control_status.right_target = right_target_signed;
    control_status.right_now = enc_right;
    control_status.right_output = right_output;
    control_status.update_count++;
}

void pid_control_init(void)
{
    /* 默认参数为零，架空车轮后再通过蓝牙逐步调参。 */
    pid_init(&angle_pid, POSITION_PID, 1.7f, 0.0f, 0.9f);
    pid_init(&motor_left_pid, DELTA_PID, 5.0f, 0.87f, 0.0f);
    pid_init(&motor_right_pid, DELTA_PID, 5.0f, 0.87f, 0.0f);

    left_direction = 0;
    right_direction = 0;
    left_target_signed = 0;
    right_target_signed = 0;
    common_speed_target = 0.0f;
    control_status = (pid_control_status_t){0};

    set_left_PWM(0);
    set_right_PWM(0);
}

void pid_control_update(float current_angle, int32_t enc_left, int32_t enc_right)
{
    float angle_error;
    float base_speed_target;
    float left_target;
    float right_target;
    int32_t left_output;
    int32_t right_output;

    angle_error = normalize_angle_degrees(
        angle_pid.target - current_angle);

    /* 角度误差在正负 2 度死区内时，清空历史并停止角度修正。 */
    if (angle_pid_gains_are_zero() ||
        (angle_error >= -ANGLE_DEADBAND_DEGREES &&
         angle_error <= ANGLE_DEADBAND_DEGREES)) {
        pid_reset(&angle_pid);
    } else {
        /* pid_cal() 内部使用 target-now，以等效当前角保证采用最短角误差。 */
        angle_pid.now = angle_pid.target - angle_error;
        pid_cal(&angle_pid);
        pidout_limit(&angle_pid,
            -ANGLE_CORRECTION_LIMIT, ANGLE_CORRECTION_LIMIT);
    }

    /* 公共速度负责前后运动，角度输出按左减右加产生差速。 */
    base_speed_target = common_speed_target;
    left_target = limit_speed_target(base_speed_target - angle_pid.out);
    right_target = limit_speed_target(base_speed_target + angle_pid.out);
    motor_target_set(round_symmetric(left_target),
        round_symmetric(right_target));

    left_output = motor_pid_update(
        &motor_left_pid, left_direction, enc_left);
    right_output = motor_pid_update(
        &motor_right_pid, right_direction, enc_right);

    set_left_PWM((int)left_output);
    set_right_PWM((int)right_output);

    pid_control_publish_status(current_angle, enc_left, enc_right,
        left_output, right_output);
}

void pid_control_set_angle_target(float target)
{
    uint32_t primask;

    target = normalize_angle_degrees(target);
    primask = pid_control_enter_critical();
    angle_pid.target = target;
    control_status.angle_target = target;
    pid_control_exit_critical(primask);
}

void pid_control_set_speed_target(float target)
{
    uint32_t primask = pid_control_enter_critical();

    common_speed_target = limit_speed_target(target);
    control_status.speed_target = common_speed_target;

    pid_control_exit_critical(primask);
}

void pid_control_stop(void)
{
    uint32_t primask = pid_control_enter_critical();
    uint32_t update_count = control_status.update_count;

    angle_pid.target = 0.0f;
    common_speed_target = 0.0f;
    motor_left_pid.target = 0.0f;
    motor_right_pid.target = 0.0f;
    pid_reset(&angle_pid);
    pid_reset(&motor_left_pid);
    pid_reset(&motor_right_pid);

    left_direction = 0;
    right_direction = 0;
    left_target_signed = 0;
    right_target_signed = 0;

    control_status = (pid_control_status_t){0};
    control_status.angle_p = angle_pid.p;
    control_status.angle_i = angle_pid.i;
    control_status.angle_d = angle_pid.d;
    control_status.motor_p = motor_left_pid.p;
    control_status.motor_i = motor_left_pid.i;
    control_status.motor_d = motor_left_pid.d;
    control_status.update_count = update_count;

    set_left_PWM(0);
    set_right_PWM(0);
    pid_control_exit_critical(primask);
}

void pid_control_get_status(pid_control_status_t *status)
{
    uint32_t primask;

    if (status == 0) {
        return;
    }

    primask = pid_control_enter_critical();
    *status = control_status;
    pid_control_exit_critical(primask);
}

void pid_control_set_gain(const char *cmd, float value)
{
    uint32_t primask;

    if (cmd == 0 || cmd[0] == '\0' || cmd[1] == '\0' || cmd[2] != '\0') {
        return;
    }

    primask = pid_control_enter_critical();

    if (cmd[1] == '1') {
        if (cmd[0] == 'P') angle_pid.p = value;
        if (cmd[0] == 'I') angle_pid.i = value;
        if (cmd[0] == 'D') angle_pid.d = value;

        /* 用户把角度三项都设为零时，立即清除旧积分和修正量。 */
        if (angle_pid_gains_are_zero()) {
            pid_reset(&angle_pid);
            control_status.angle_out = 0.0f;
        }
    } else if (cmd[1] == '2') {
        if (cmd[0] == 'P') {
            motor_left_pid.p = value;
            motor_right_pid.p = value;
        }
        if (cmd[0] == 'I') {
            motor_left_pid.i = value;
            motor_right_pid.i = value;
        }
        if (cmd[0] == 'D') {
            motor_left_pid.d = value;
            motor_right_pid.d = value;
        }
    }

    control_status.angle_p = angle_pid.p;
    control_status.angle_i = angle_pid.i;
    control_status.angle_d = angle_pid.d;
    control_status.motor_p = motor_left_pid.p;
    control_status.motor_i = motor_left_pid.i;
    control_status.motor_d = motor_left_pid.d;

    pid_control_exit_critical(primask);
}
