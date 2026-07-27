#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "mpu6050.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"

uint8_t enable_group1_irq = 0;

void Interrupt_Init(void)
{
    if(enable_group1_irq)
    {
        NVIC_EnableIRQ(1);
    }
}

void SysTick_Handler(void)
{
    tick_ms++;
}

#if defined UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;

    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    if((DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false) &&
        (rxSize < sizeof(bno08x_dmaBuffer))) {
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);
    }

    for(uint8_t i = 2U; i <= 14U; i++) {
        checkSum += bno08x_dmaBuffer[i];
    }

    if((rxSize == 19U) &&
        (bno08x_dmaBuffer[0] == 0xAAU) &&
        (bno08x_dmaBuffer[1] == 0xAAU) &&
        (checkSum == bno08x_dmaBuffer[18])) {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (float) ((int16_t) ((bno08x_dmaBuffer[4] << 8) |
            bno08x_dmaBuffer[3])) / 100.0f;
        bno08x_data.pitch = (float) ((int16_t) ((bno08x_dmaBuffer[6] << 8) |
            bno08x_dmaBuffer[5])) / 100.0f;
        bno08x_data.roll = (float) ((int16_t) ((bno08x_dmaBuffer[8] << 8) |
            bno08x_dmaBuffer[7])) / 100.0f;
        bno08x_data.ax = (int16_t) ((bno08x_dmaBuffer[10] << 8) |
            bno08x_dmaBuffer[9]);
        bno08x_data.ay = (int16_t) ((bno08x_dmaBuffer[12] << 8) |
            bno08x_dmaBuffer[11]);
        bno08x_data.az = (int16_t) ((bno08x_dmaBuffer[14] << 8) |
            bno08x_dmaBuffer[13]);
        bno08x_data.data_ready = 1U;
        bno08x_data.product_id_ok = 1U;
        bno08x_data.packet_count++;
        bno08x_data.last_update_ms = tick_ms;
    } else if(rxSize != 0U) {
        bno08x_data.error_count++;
    }

    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID,
        (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}
#endif

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    WIT_ProcessBytes(wit_dmaBuffer, rxSize);

    while(DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        WIT_ProcessByte(DL_UART_receiveData(UART_WIT_INST));

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif

// void GROUP1_IRQHandler(void)
// {
//     switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
//         #if defined GPIO_MULTIPLE_GPIOA_INT_IIDX
//         case GPIO_MULTIPLE_GPIOA_INT_IIDX:
//             switch (DL_GPIO_getPendingInterrupt(GPIOA))
//             {
//                 #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOA)
//                 case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
//                     Read_Quad();
//                     break;
//                 #endif

//                 #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOA)
//                 case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
//                     Read_LSM6DSV16X();
//                     break;
//                 #endif

//                 #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOA)
//                 case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
//                     Read_VL53L0X();
//                     break;
//                 #endif

//                 default:
//                     break;
//             }
//         #endif

//         #if defined GPIO_MULTIPLE_GPIOB_INT_IIDX
//         case GPIO_MULTIPLE_GPIOB_INT_IIDX:
//             switch (DL_GPIO_getPendingInterrupt(GPIOB))
//             {
//                 #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOB)
//                 case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
//                     Read_Quad();
//                     break;
//                 #endif

//                 #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOB)
//                 case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
//                     Read_LSM6DSV16X();
//                     break;
//                 #endif

//                 #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOB)
//                 case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
//                     Read_VL53L0X();
//                     break;
//                 #endif

//                 default:
//                     break;
//             }
//         #endif

//         #if defined GPIO_MPU6050_INT_IIDX
//             case GPIO_MPU6050_INT_IIDX:
//                 Read_Quad();
//                 break;
//         #endif

//         #if defined GPIO_LSM6DSV16X_INT_IIDX
//             case GPIO_LSM6DSV16X_INT_IIDX:
//                 Read_LSM6DSV16X();
//                 break;
//         #endif

//         #if defined GPIO_VL53L0X_INT_IIDX
//             case GPIO_VL53L0X_INT_IIDX:
//                 Read_VL53L0X();
//                 break;
//         #endif
//     }
// }
