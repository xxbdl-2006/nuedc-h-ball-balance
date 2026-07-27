#ifndef _BNO08X_UART_RVC_H_
#define _BNO08X_UART_RVC_H_

#include "ti_msp_dl_config.h"
#include <stdbool.h>

typedef struct {
    uint8_t index;
    float pitch;
    float roll;
    float yaw;
    float quat_i;
    float quat_j;
    float quat_k;
    float quat_real;
    int16_t ax;
    int16_t ay;
    int16_t az;
    uint8_t accuracy;
    uint8_t data_ready;
    uint8_t product_id_ok;
    uint32_t packet_count;
    uint32_t error_count;
    uint32_t last_update_ms;
} BNO08X_Data_t;

extern BNO08X_Data_t bno08x_data;
extern uint8_t bno08x_dmaBuffer[19];
extern uint8_t bno08x_i2c_addr;
extern uint8_t bno08x_i2c_scan_count;
extern uint8_t bno08x_i2c_scan_first;
extern uint8_t bno08x_i2c_scan_second;
extern uint8_t bno08x_i2c_scl_low;
extern uint8_t bno08x_i2c_sda_low;
extern uint8_t bno08x_init_stage;
extern uint8_t bno08x_i2c_last_error;
extern bool bno08x_i2c_online;

void BNO08X_Init(void);
bool BNO08X_Probe(void);
void BNO08X_ScanBus(void);
void BNO08X_RequestZero(void);
bool BNO08X_Update(void);
void BNO08X_ProcessByte(uint8_t byte);
void BNO08X_ProcessBytes(const uint8_t *data, uint8_t len);

#endif  /* #ifndef _BNO08X_UART_RVC_H_ */
