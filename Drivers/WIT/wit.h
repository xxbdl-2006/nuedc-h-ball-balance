/*
 * SysConfig Configuration Steps:
 *   UART:
 *     1. Add an UART module.
 *     2. Name it as "UART_WIT".
 *     3. Set "Target Baud Rate" according to module configuration.
 *     4. Set "Communication Direction" to "TX/RX" if commands are needed,
 *        or "RX only" for passive angle feedback.
 *     5. Check the box "Enable FIFOs".
 *     6. Set "RX Timeout Interrupt Counts" to "1".
 *     7. Check the "RX timeout" box at "Enable Interrupts".
 *     8. Set "Configure DMA RX Trigger" to "UART RX interrupt".
 *     9. Set "DMA Channel RX Name" to "DMA_WIT".
 *     10. Set "Address Mode" to "Fixed addr. to Block addr.".
 *     11. Set "Source Length" and "Destination Length" to "Byte".
 *     12. Set the pin according to your needs.
 */

#ifndef __WIT_H
#define __WIT_H

#include "ti_msp_dl_config.h"

#define WIT_FRAME_ACCEL  (0x01u)
#define WIT_FRAME_GYRO   (0x02u)
#define WIT_FRAME_ANGLE  (0x04u)

typedef struct {
    float pitch;
    float roll;
    float yaw;
    float temperature;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    int16_t version;
    uint8_t updated_flags;
    uint32_t packet_count;
    uint32_t checksum_error_count;
    uint32_t last_update_ms;
} WIT_Data_t;

extern volatile WIT_Data_t wit_data;
extern uint8_t wit_dmaBuffer[33];

void WIT_Init(void);
void WIT_RequestZero(void);
void WIT_ProcessByte(uint8_t byte);
void WIT_ProcessBytes(const uint8_t *data, uint8_t len);
void WIT_ClearUpdatedFlags(uint8_t flags);
void WIT_SendBytes(const uint8_t *data, uint8_t len);

#endif /* #ifndef __WIT_H */
