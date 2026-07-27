#include "encoder.h"
#include "pid.h"

static ENCODER_RES motor_encoder_L;
static ENCODER_RES motor_encoder_R;

void encoder_init(void)
{
    NVIC_ClearPendingIRQ(encoder_L_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(encoder_L_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
}

int get_encoder_count_L(void)
{
    return motor_encoder_L.count;
}

ENCODER_DIR get_encoder_dir_L(void)
{
    return motor_encoder_L.dir;
}

int get_encoder_count_R(void)
{
    return -motor_encoder_R.count;
}

ENCODER_DIR get_encoder_dir_R(void)
{
    return motor_encoder_R.dir;
}

void encoder_update(void)
{
    uint32_t primask;
    int32_t left_count;
    int32_t right_count;

    primask = __get_PRIMASK();
    __disable_irq();
    left_count = motor_encoder_L.temp_count;
    right_count = motor_encoder_R.temp_count;
    motor_encoder_L.temp_count = 0;
    motor_encoder_R.temp_count = 0;
    if (primask == 0U) {
        __enable_irq();
    }

    motor_encoder_L.count = left_count;
    motor_encoder_L.dir = (motor_encoder_L.count >= 0) ? FORWARD : REVERSAL;
    motor_encoder_R.count = right_count;
    motor_encoder_R.dir = (motor_encoder_R.count <= 0) ? FORWARD : REVERSAL;
}

static void encoder_update_pair(volatile int32_t *count,
    GPIO_Regs *a_port, uint32_t a_pin, GPIO_Regs *b_port, uint32_t b_pin,
    uint8_t a_triggered, uint8_t b_triggered)
{
    if (a_triggered) {
        if (DL_GPIO_readPins(b_port, b_pin) == 0U) {
            (*count)--;
        } else {
            (*count)++;
        }
    }

    if (b_triggered) {
        if (DL_GPIO_readPins(a_port, a_pin) == 0U) {
            (*count)++;
        } else {
            (*count)--;
        }
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t l_a_status = DL_GPIO_getEnabledInterruptStatus(
        encoder_L_PB26_PORT, encoder_L_PB26_PIN);
    uint32_t l_b_status = DL_GPIO_getEnabledInterruptStatus(
        encoder_L_PA13_PORT, encoder_L_PA13_PIN);

    encoder_update_pair(&motor_encoder_L.temp_count,
        encoder_L_PB26_PORT, encoder_L_PB26_PIN,
        encoder_L_PA13_PORT, encoder_L_PA13_PIN,
        (l_a_status & encoder_L_PB26_PIN) == encoder_L_PB26_PIN,
        (l_b_status & encoder_L_PA13_PIN) == encoder_L_PA13_PIN);

    DL_GPIO_clearInterruptStatus(encoder_L_PB26_PORT, encoder_L_PB26_PIN);
    DL_GPIO_clearInterruptStatus(encoder_L_PA13_PORT, encoder_L_PA13_PIN);

    uint32_t r_status = DL_GPIO_getEnabledInterruptStatus(
        encoder_R_PORT, encoder_R_PB5_PIN | encoder_R_PB4_PIN);

    encoder_update_pair(&motor_encoder_R.temp_count,
        encoder_R_PORT, encoder_R_PB5_PIN,
        encoder_R_PORT, encoder_R_PB4_PIN,
        (r_status & encoder_R_PB5_PIN) == encoder_R_PB5_PIN,
        (r_status & encoder_R_PB4_PIN) == encoder_R_PB4_PIN);

    DL_GPIO_clearInterruptStatus(
        encoder_R_PORT, encoder_R_PB5_PIN | encoder_R_PB4_PIN);
}
