/*
 * oled_display.c — OLED page display manager implementation.
 *
 * Renders 5 board-test pages on a 128×64 OLED via cached line writes.
 * Each page displays live sensor / IO state relevant to one subsystem.
 */

#include "oled_display.h"

#include "bno08x_uart_rvc.h"
#include "bsp_hc05.h"
#include "clock.h"
#include "pid_control.h"
#include "encoder.h"
#include "oled_hardware_i2c.h"
#include "redlight.h"
#include "serial_debug.h"
#include "wit.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ---- Global shared with redlight module ---- */
uint8_t p[8];

/* ---- Static state ---- */
static uint8_t  oled_line[32];
static char     oled_cache[8][OLED_TEXT_COLS + 1U];
static uint8_t  current_page;
static uint8_t  last_key;
static uint32_t last_oled_ms;

/* ================================================================
 *  Internal helpers
 * ================================================================ */

static void oled_invalidate_cache(void)
{
    memset(oled_cache, 0, sizeof(oled_cache));
}

/*
 * Print a formatted string to OLED line <line> (0–7).
 * Only sends the I2C command when the content actually changed
 * (cache-based flicker reduction).
 */
static void oled_print8(uint8_t line, const char *fmt, ...)
{
    va_list args;
    char    padded[OLED_TEXT_COLS + 1U];
    uint8_t len;

    if (line >= 8U) {
        return;
    }

    va_start(args, fmt);
    vsnprintf((char *)oled_line, sizeof(oled_line), fmt, args);
    va_end(args);

    memset(padded, ' ', OLED_TEXT_COLS);
    padded[OLED_TEXT_COLS] = '\0';

    len = (uint8_t)strlen((char *)oled_line);
    if (len > OLED_TEXT_COLS) {
        len = OLED_TEXT_COLS;
    }
    memcpy(padded, oled_line, len);

    if (memcmp(oled_cache[line], padded, OLED_TEXT_COLS + 1U) != 0) {
        memcpy(oled_cache[line], padded, OLED_TEXT_COLS + 1U);
        OLED_ShowString(0, line, (uint8_t *)padded, 8);
    }
}

static void show_header(const char *title)
{
    oled_print8(0, "NUEDC IMU TEST");
    oled_print8(1, "%s", title);
}

/* ================================================================
 *  Per-page display functions
 * ================================================================ */

static void show_page_bno08x(void)
{
    uint32_t age = (uint32_t)tick_ms - bno08x_data.last_update_ms;

    show_header("BNO08X I2C");
    oled_print8(2, "A:%02X On:%u ID:%u P:%lu",
        bno08x_i2c_addr,
        (unsigned int)bno08x_i2c_online,
        bno08x_data.product_id_ok,
        (unsigned long)bno08x_data.packet_count);
    oled_print8(3, "Yaw:%.2f", bno08x_data.yaw);

    oled_print8(5, "Acc:%u Age:%lums",
        bno08x_data.accuracy,
        (unsigned long)age);
    oled_print8(6, "Err:%lu St:%u E:%u",
        (unsigned long)bno08x_data.error_count,
        bno08x_init_stage,
        bno08x_i2c_last_error);
    oled_print8(7, "K1:Next K2:Zero");
}

static void show_page_jy61p(void)
{
    uint32_t age = (uint32_t)tick_ms - wit_data.last_update_ms;

    show_header("JY61P UART2");
    oled_print8(2, "PB15 TX PB16 RX");
    oled_print8(3, "Yaw:%d", (int)wit_data.yaw);
    oled_print8(4, "Pit:%d Rol:%d",
        (int)wit_data.pitch,
        (int)wit_data.roll);
    oled_print8(5, "Pk:%lu Err:%lu",
        (unsigned long)wit_data.packet_count,
        (unsigned long)wit_data.checksum_error_count);
    oled_print8(6, "Age:%lums", (unsigned long)age);
    oled_print8(7, "K1:Next K2:Zero");
}

static void show_page_gray(void)
{
    uint8_t mask   = 0U;
    uint8_t active = 0U;

    read_redlight_pin();
    for (uint8_t i = 0U; i < 8U; i++) {
        if (p[i] != 0U) {
            mask |= (uint8_t)(1U << i);
            active++;
        }
    }

    show_header("GRAY 8CH");
    oled_print8(2, "01234567:%u%u%u%u%u%u%u%u",
        p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
    oled_print8(3, "Mask:0x%02X Active:%u", mask, active);
    oled_print8(4, "0 PA27 1 PA25");
    oled_print8(5, "2 PB25 3 PB20");
    oled_print8(6, "4 PA14 5 PA16");
    oled_print8(7, "6 PB17 7 PB19");
}

static void show_page_motor(void)
{

    show_header("ENCODER COUNT");

    oled_print8(2, "L:%d", get_encoder_count_L());
    oled_print8(3, "R:%d", get_encoder_count_R());
    oled_print8(4, "L Dir:%s", get_encoder_dir_L() == FORWARD ? "FWD" : "REV");
    oled_print8(5, "R Dir:%s", get_encoder_dir_R() == FORWARD ? "FWD" : "REV");

    oled_print8(7, "P3:Page BT tune");
}

static void show_page_bluetooth(void)
{
    pid_control_status_t status;

    pid_control_get_status(&status);

    show_header("BT PID SETTINGS");
    oled_print8(2, "A P:%.2f I:%.3f", status.angle_p, status.angle_i);
    oled_print8(3, "A D:%.3f T:%d",  status.angle_d,
        (int)status.angle_target);
    oled_print8(4, "A N:%d O:%d", (int)status.angle_now,
        (int)status.angle_out);
    oled_print8(5, "M P:%.2f I:%.3f", status.motor_p, status.motor_i);
    /* M T 为公共有符号速度目标，单位：脉冲/50 ms。 */
    oled_print8(6, "M D:%.3f T:%d", status.motor_d,
        (int)status.speed_target);
    oled_print8(7, "RX:%lu AT/CSV", (unsigned long)Bluetooth_RxCount);
}

/* ================================================================
 *  Public API
 * ================================================================ */

void oled_display_init(void)
{
    OLED_Init();
    OLED_Clear();
    oled_invalidate_cache();
    oled_print8(0, "NUEDC IMU TEST");
    oled_print8(2, "Init BNO/JY61P");
    current_page  = PAGE_BNO08X;
    last_key      = 0U;
    last_oled_ms  = 0U;
}

void oled_display_refresh(void)
{
    switch (current_page) {
    case PAGE_BNO08X:
        show_page_bno08x();
        break;
    case PAGE_JY61P:
        show_page_jy61p();
        break;
    case PAGE_GRAY:
        show_page_gray();
        break;
    case PAGE_MOTOR:
        show_page_motor();
        break;
    default:
        show_page_bluetooth();
        break;
    }

    last_oled_ms = (uint32_t)tick_ms;
}

void oled_display_switch_page(void)
{
    if (current_page == PAGE_MOTOR) {
        pid_control_stop();
    }

    current_page = (current_page + 1U) % IMU_TEST_PAGE_COUNT;
    OLED_Clear();
    oled_invalidate_cache();
    oled_display_refresh();
}

void oled_display_prev_page(void)
{
    if (current_page == PAGE_MOTOR) {
        pid_control_stop();
    }

    current_page = (current_page == 0U)
        ? (IMU_TEST_PAGE_COUNT - 1U)
        : (current_page - 1U);
    OLED_Clear();
    oled_invalidate_cache();
    oled_display_refresh();
}

void oled_display_zero_action(void)
{
    switch (current_page) {
    case PAGE_BNO08X:
        serial_debug_bno08x_action();
        break;
    case PAGE_JY61P:
        WIT_RequestZero();
        break;
    case PAGE_MOTOR:
        pid_control_stop();
        break;
    default:
        break;
    }
}

void oled_display_tick(void)
{
    if ((uint32_t)(tick_ms - last_oled_ms) >= OLED_REFRESH_MS) {
        oled_display_refresh();
    }
}

uint8_t oled_display_get_page(void)
{
    return current_page;
}

uint8_t oled_display_get_last_key(void)
{
    return last_key;
}

void oled_display_set_last_key(uint8_t key)
{
    last_key = key;
}
