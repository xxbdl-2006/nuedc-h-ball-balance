/*
 * serial_debug.h — UART debug report, Bluetooth test helpers, and BNO08X
 * health monitoring for the NUEDC IMU board test.
 *
 * Sends one diagnostic JSON-like line per second over UART0, handles KEY2
 * actions for the BNO08X and Bluetooth pages, and periodically retries
 * BNO08X probing when the sensor is offline.
 */

#ifndef _SERIAL_DEBUG_H_
#define _SERIAL_DEBUG_H_

#include <stdint.h>

/* ---- Public API ---- */

void     serial_debug_init(void);
void     serial_debug_tick(void);
void     serial_debug_bno08x_action(void);
void     serial_debug_bluetooth_ping(void);
uint32_t serial_debug_get_last_bt_rx_ms(void);

#endif /* _SERIAL_DEBUG_H_ */
