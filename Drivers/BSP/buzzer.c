#include "buzzer.h"
#include "clock.h"

#ifndef BUZZER_ACTIVE_LOW
#define BUZZER_ACTIVE_LOW 0
#endif

void Buzzer_On(void)
{
#if defined(buzzer_control_PIN)
#if BUZZER_ACTIVE_LOW
    DL_GPIO_clearPins(buzzer_PORT, buzzer_control_PIN);
#else
    DL_GPIO_setPins(buzzer_PORT, buzzer_control_PIN);
#endif
#endif
}

void Buzzer_Off(void)
{
#if defined(buzzer_control_PIN)
#if BUZZER_ACTIVE_LOW
    DL_GPIO_setPins(buzzer_PORT, buzzer_control_PIN);
#else
    DL_GPIO_clearPins(buzzer_PORT, buzzer_control_PIN);
#endif
#endif
}

void Buzzer_Beep(uint32_t ms)
{
    Buzzer_On();
    mspm0_delay_ms(ms);
    Buzzer_Off();
}
