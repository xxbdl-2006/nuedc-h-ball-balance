/*
 * HC05/JDY31 Bluetooth UART driver for MSPM0G3507.
 */

#ifndef __HC05_H
#define __HC05_H

#include <stdint.h>

#define BLERX_LEN_MAX 200U
#define BLUETOOTH_LAST_TEXT_LEN 20U

extern uint8_t Bluetooth_ConnectFlag;
extern uint8_t BLERX_BUFF[BLERX_LEN_MAX];
extern uint8_t BLERX_FLAG;
extern uint8_t BLERX_LEN;
extern volatile uint32_t Bluetooth_RxCount;
extern volatile uint32_t Bluetooth_TxCount;
extern volatile uint32_t Bluetooth_IrqCount;
extern volatile uint32_t Bluetooth_PollCount;
extern volatile uint32_t Bluetooth_OverflowCount;
extern volatile uint8_t Bluetooth_LastByte;
extern uint8_t Bluetooth_LastText[BLUETOOTH_LAST_TEXT_LEN + 1U];
extern volatile uint8_t Bluetooth_LastTextLen;

void BLE_Send_Bit(uint8_t ch);
void BLE_send_String(const uint8_t *str);
void Clear_BLERX_BUFF(void);
void Bluetooth_Init(void);
void Bluetooth_Poll(void);
void Bluetooth_ProcessReceivedByte(uint8_t byte);
uint8_t Get_Bluetooth_ConnectFlag(void);
int Receive_Bluetooth_Data(void);
void Send_Bluetooth_Data(const char *dat);

#endif
