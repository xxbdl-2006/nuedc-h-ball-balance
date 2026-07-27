#ifndef __PID_H_
#define __PID_H_

#include <stdint.h>

/* PID 计算模式：角度环使用位置式，轮速环使用增量式。 */
enum {
    POSITION_PID = 0,
    DELTA_PID
};

typedef struct {
    float target;
    float now;
    float error[3];
    float p;
    float i;
    float d;
    float pout;
    float iout;
    float dout;
    float out;
    uint32_t pid_mode;
} pid_t;

/* 初始化模式和参数，同时清零全部运行状态。 */
void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);

/* 清除误差和输出历史，保留目标值、模式及 PID 参数。 */
void pid_reset(pid_t *pid);

/* 按 pid_mode 执行一次位置式或增量式 PID 计算。 */
void pid_cal(pid_t *pid);

/* 只限制总输出，不改变积分项和误差历史。 */
void pidout_limit(pid_t *pid, float min_out, float max_out);

#endif
