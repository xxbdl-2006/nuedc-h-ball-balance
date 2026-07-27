#ifndef _encoder_H_
#define _encoder_H_

#include "ti_msp_dl_config.h"

typedef enum {
    FORWARD,  // 正向
    REVERSAL  // 反向
} ENCODER_DIR;

typedef struct {
    volatile int32_t temp_count; //保存实时计数值
    int32_t count;         						 //根据定时器时间更新的计数值
    ENCODER_DIR dir;            	 //旋转方向
} ENCODER_RES;


void encoder_init(void);
int get_encoder_count_L(void);
ENCODER_DIR get_encoder_dir_L(void);
int get_encoder_count_R(void);
ENCODER_DIR get_encoder_dir_R(void);
void encoder_update(void);

#endif
