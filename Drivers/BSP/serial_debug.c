/*
 * serial_debug.c — UART debug report, Bluetooth test, and BNO08X health
 * monitoring implementation.
 *
 * Sends a key=value diagnostic line every 1 s over UART0.
 * Manages periodic BNO08X update (50 ms), automatic re-probe (1 s),
 * Bluetooth RX age tracking, and KEY2 actions for BNO08X / Bluetooth pages.
 */

#include "serial_debug.h"

#include "bno08x_uart_rvc.h"
#include "bsp_hc05.h"
#include "clock.h"
#include "oled_display.h"
#include "wit.h"

#include <stdbool.h>
#include <stdio.h>

/* ---- Static state ---- */
static uint32_t last_uart_ms;
static uint32_t last_bno_update_ms;
static uint32_t last_bno_reprobe_ms;
static uint32_t last_bt_rx_ms;
static uint32_t last_bt_rx_count;
static uint32_t bt_ping_count;
static uint32_t bno_probe_count;

/* ================================================================
 *  Internal helpers
 * ================================================================ */

static void uart0_send_char(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true) {
    }
    DL_UART_Main_transmitData(UART_0_INST, ch);
    Bluetooth_TxCount++;
}

static void uart0_send_string(const char *str)
{
    if (str == NULL) {
        return;
    }

    while (*str != '\0') {
        uart0_send_char(*str++);
    }
}

static void reprobe_bno08x(void)
{
    bno_probe_count++;
    BNO08X_Init();
}

static void send_uart_report(void)
{
    char msg[384];

    snprintf(msg, sizeof(msg),
        "page=%u key=%u "
        "bno_scan=%u bno_scan_first=0x%02X bno_scan_second=0x%02X "
        "bno_scl_low=%u bno_sda_low=%u bno_stage=%u bno_i2c_err=%u "
        "bno_online=%u bno_addr=0x%02X bno_id=%u "
        "bno_pk=%lu bno_err=%lu bno_age=%lu "
        "bno_yaw=%.2f bno_pitch=%.2f bno_roll=%.2f "
        "jy_pk=%lu jy_err=%lu jy_yaw=%d jy_pitch=%d jy_roll=%d jy_age=%lu "
        "bt_rx=%lu bt_tx=%lu bt_irq=%lu bt_poll=%lu bt_last=0x%02X\r\n",
        oled_display_get_page(),
        oled_display_get_last_key(),
        bno08x_i2c_scan_count,
        bno08x_i2c_scan_first,
        bno08x_i2c_scan_second,
        bno08x_i2c_scl_low,
        bno08x_i2c_sda_low,
        bno08x_init_stage,
        bno08x_i2c_last_error,
        (unsigned int)bno08x_i2c_online,
        bno08x_i2c_addr,
        bno08x_data.product_id_ok,
        (unsigned long)bno08x_data.packet_count,
        (unsigned long)bno08x_data.error_count,
        (unsigned long)((uint32_t)tick_ms - bno08x_data.last_update_ms),
        bno08x_data.yaw,
        bno08x_data.pitch,
        bno08x_data.roll,
        (unsigned long)wit_data.packet_count,
        (unsigned long)wit_data.checksum_error_count,
        (int)wit_data.yaw,
        (int)wit_data.pitch,
        (int)wit_data.roll,
        (unsigned long)((uint32_t)tick_ms - wit_data.last_update_ms),
        (unsigned long)Bluetooth_RxCount,
        (unsigned long)Bluetooth_TxCount,
        (unsigned long)Bluetooth_IrqCount,
        (unsigned long)Bluetooth_PollCount,
        Bluetooth_LastByte);
    uart0_send_string(msg);
}

/* ================================================================
 *  Public API
 * ================================================================ */

void serial_debug_init(void)
{
    last_uart_ms        = 0U;
    last_bno_update_ms  = 0U;
    last_bno_reprobe_ms = 0U;
    last_bt_rx_ms       = 0U;
    last_bt_rx_count    = 0U;
    bt_ping_count       = 0U;
    bno_probe_count     = 0U;

    uart0_send_string("\r\nNUEDC_2026 board test: BNO08X JY61P JDY31\r\n");
}

void serial_debug_tick(void)
{
    uint32_t now = (uint32_t)tick_ms;

    /* 50 ms — BNO08X SHTP polling */
    if ((now - last_bno_update_ms) >= 50U) {
        (void)BNO08X_Update();
        last_bno_update_ms = now;
    }

    /* 1 s — automatic BNO08X re-probe when offline or erroring */
    if ((bno08x_i2c_online == false || bno08x_data.error_count != 0U) &&
        (now - last_bno_reprobe_ms) >= 1000U) {
        reprobe_bno08x();
        last_bno_reprobe_ms = now;
    }

    /* Track Bluetooth RX age for the OLED display */
    if (Bluetooth_RxCount != last_bt_rx_count) {
        last_bt_rx_count = Bluetooth_RxCount;
        last_bt_rx_ms    = now;
    }

    /* 1 s — diagnostic report */
    if ((now - last_uart_ms) >= 1000U) {
        send_uart_report();
        last_uart_ms = now;
    }
}

void serial_debug_bno08x_action(void)
{
    if (bno08x_i2c_online == false || bno08x_data.packet_count == 0U) {
        reprobe_bno08x();
    } else {
        BNO08X_RequestZero();
    }
}

void serial_debug_bluetooth_ping(void)
{
    char msg[64];

    bt_ping_count++;
    snprintf(msg, sizeof(msg), "JDY31 TEST %lu\r\n",
        (unsigned long)bt_ping_count);
    Send_Bluetooth_Data(msg);
}

uint32_t serial_debug_get_last_bt_rx_ms(void)
{
    return last_bt_rx_ms;
}
