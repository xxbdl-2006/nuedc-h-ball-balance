#ifndef _MAIN_H_
#define _MAIN_H_

/*
 * main.h — Shared convenience header for the NUEDC board-test project.
 *
 * Pulls in driver headers that many application modules use transitively.
 * The new application-level modules (oled_display, motor_test, serial_debug)
 * are also included here so that future code can include just this header.
 */

#include "clock.h"
#include "interrupt.h"

#include "mpu6050.h"
#include "oled_software_i2c.h"
#include "oled_hardware_i2c.h"
#include "oled_software_spi.h"
#include "oled_hardware_spi.h"
#include "ultrasonic_capture.h"
#include "ultrasonic_gpio.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"

/* Application-level modules (post-refactor) */
#include "oled_display.h"
#include "serial_debug.h"

#endif  /* #ifndef _MAIN_H_ */
