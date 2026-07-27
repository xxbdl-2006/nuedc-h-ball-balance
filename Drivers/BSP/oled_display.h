/*
 * oled_display.h — OLED page display manager for NUEDC IMU board test.
 *
 * Manages 5 test pages (BNO08X, JY61P, Gray, Motor, Bluetooth) with
 * low-flicker cached OLED rendering and page switching.
 */

#ifndef _OLED_DISPLAY_H_
#define _OLED_DISPLAY_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* Page identifiers */
#define IMU_TEST_PAGE_COUNT 5U
#define PAGE_BNO08X          0U
#define PAGE_JY61P           1U
#define PAGE_GRAY            2U
#define PAGE_MOTOR           3U
#define PAGE_BLUETOOTH       4U

/* Display geometry */
#define OLED_TEXT_COLS        21U
#define OLED_REFRESH_MS       100U

/* ---- Public API ---- */

void     oled_display_init(void);
void     oled_display_refresh(void);
void     oled_display_switch_page(void);   /* KEY1 single-click → next page    */
void     oled_display_prev_page(void);     /* KEY1 double-click → previous page */
void     oled_display_zero_action(void);   /* KEY1 long-press  → page-zero     */
void     oled_display_tick(void);
uint8_t  oled_display_get_page(void);
uint8_t  oled_display_get_last_key(void);
void     oled_display_set_last_key(uint8_t key);

#endif /* _OLED_DISPLAY_H_ */
