#include "pid.h"

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
    if (pid == 0) {
        return;
    }

    *pid = (pid_t){0};
    pid->pid_mode = mode;
    pid->p = p;
    pid->i = i;
    pid->d = d;
}

void pid_reset(pid_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->now = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
    pid->out = 0.0f;
}

void pid_cal(pid_t *pid)
{
    if (pid == 0) {
        return;
    }

    pid->error[0] = pid->target - pid->now;

    if (pid->pid_mode == DELTA_PID) {
        /* 增量式：本次只计算输出增量，再累加到原输出。 */
        pid->pout = pid->p * (pid->error[0] - pid->error[1]);
        pid->iout = pid->i * pid->error[0];
        pid->dout = pid->d *
            (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->out += pid->pout + pid->iout + pid->dout;
    } else if (pid->pid_mode == POSITION_PID) {
        /* 位置式：P、累计 I 和本次 D 直接组成当前输出。 */
        pid->pout = pid->p * pid->error[0];
        pid->iout += pid->i * pid->error[0];
        pid->dout = pid->d * (pid->error[0] - pid->error[1]);
        pid->out = pid->pout + pid->iout + pid->dout;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

void pidout_limit(pid_t *pid, float min_out, float max_out)
{
    if (pid == 0) {
        return;
    }

    if (pid->out > max_out) {
        pid->out = max_out;
    }
    if (pid->out < min_out) {
        pid->out = min_out;
    }
}
