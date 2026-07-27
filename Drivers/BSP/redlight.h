#ifndef __redlight_h_
#define __redlight_h_
#include "ti_msp_dl_config.h"



extern uint8_t p[8];

void read_redlight_pin();
int check_three_or_more();
void track();
void track4();

#endif
