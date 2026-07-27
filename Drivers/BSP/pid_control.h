#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>

/* 公共速度和每个车轮最终目标的范围，单位：编码器脉冲/50 ms。 */
#define PID_CONTROL_SPEED_TARGET_LIMIT 200.0f

/* 三路 PID 的只读状态快照，供 OLED 和 UART0 波形发送使用。 */
typedef struct {
    float angle_p;
    float angle_i;
    float angle_d;
    float motor_p;
    float motor_i;
    float motor_d;
    float angle_target;
    float angle_now;
    float angle_out;
    float speed_target;
    int32_t left_target;
    int32_t left_now;
    int32_t left_output;
    int32_t right_target;
    int32_t right_now;
    int32_t right_output;
    uint32_t update_count;
} pid_control_status_t;

/* 初始化一个角度位置式 PID 和两个独立的轮速增量式 PID。 */
void pid_control_init(void);

/*
 * 每 50 ms 调用一次。
 * current_angle 单位为度，编码器值单位为脉冲/50 ms。
 */
void pid_control_update(float current_angle, int32_t enc_left, int32_t enc_right);

/* 设置需要保持的航向角，单位为度。 */
void pid_control_set_angle_target(float target);

/* 设置公共有符号速度目标，内部限制到 ±200 脉冲/50 ms。 */
void pid_control_set_speed_target(float target);

/* 清零角度/速度目标和三路 PID 运行历史，并立即关闭两路电机。 */
void pid_control_stop(void);

/* 原子复制最近一次完整控制状态。 */
void pid_control_get_status(pid_control_status_t *status);

/* P1/I1/D1 调角度环，P2/I2/D2 同时调左右轮速度环。 */
void pid_control_set_gain(const char *cmd, float value);

#endif
