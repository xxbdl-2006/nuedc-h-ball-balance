#include "bsp_sg90.h"

static unsigned int servo_angle;

void Set_Servo_Angle_X(unsigned int angle)
{
    servo_angle = (angle > 180U) ? 180U : angle;
}

void Set_Servo_Angle_Y(unsigned int angle)
{
    servo_angle = (angle > 180U) ? 180U : angle;
}

unsigned int Get_Servo_Angle(void)
{
    return servo_angle;
}
