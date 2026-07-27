#ifndef	__timer_H__
#define __timer_H__

#include "ti_msp_dl_config.h"


void timer_init(void);
extern int target_circles;        // 目标圈数(可在1-5之间设置)

extern int task;
extern int start_flag;
extern uint8_t initial_search_direction;
#endif