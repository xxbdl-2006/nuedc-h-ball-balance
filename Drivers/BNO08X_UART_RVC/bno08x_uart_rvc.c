#include "bno08x_uart_rvc.h"
#include "clock.h"
#include <math.h>
#include <string.h>

/*
 * 2026 temporary bring-up note:
 * BNO08X is now back on the final 2026 PCB wiring: I2C1 SCL=PB2, SDA=PB3.
 * The UART-RVC parser is kept only as a temporary fallback for isolated tests.
 */

BNO08X_Data_t bno08x_data;
uint8_t bno08x_dmaBuffer[19];
uint8_t bno08x_i2c_addr;
uint8_t bno08x_i2c_scan_count;
uint8_t bno08x_i2c_scan_first;
uint8_t bno08x_i2c_scan_second;
uint8_t bno08x_i2c_scl_low;
uint8_t bno08x_i2c_sda_low;
uint8_t bno08x_init_stage;
uint8_t bno08x_i2c_last_error;
bool bno08x_i2c_online;

#define BNO08X_CHANNEL_EXECUTABLE       (1U)
#define BNO08X_CHANNEL_CONTROL          (2U)
#define BNO08X_CHANNEL_REPORTS          (3U)
#define BNO08X_REPORT_PRODUCT_ID_RESP   (0xF8U)
#define BNO08X_REPORT_PRODUCT_ID_REQ    (0xF9U)
#define BNO08X_REPORT_BASE_TIMESTAMP    (0xFBU)
#define BNO08X_REPORT_SET_FEATURE       (0xFDU)
#define BNO08X_SENSOR_ROTATION_VECTOR   (0x05U)
#define BNO08X_MAX_PACKET_SIZE          (128U)
#define BNO08X_I2C_CHUNK_SIZE           (32U)
#define BNO08X_WAIT_COUNT               (2000000U)

static uint8_t bno08x_shtp_header[4];
static uint8_t bno08x_shtp_data[BNO08X_MAX_PACKET_SIZE];
static uint8_t bno08x_sequence[6];
static uint8_t bno08x_rvc_packet[19];
static uint8_t bno08x_rvc_index;
static float bno08x_yaw_offset;
static float bno08x_pitch_offset;
static float bno08x_roll_offset;
static uint8_t bno08x_zero_pending = 1U;

static float bno08x_normalize_angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }

    while (angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static void bno08x_apply_zero(void)
{
    if (bno08x_zero_pending != 0U) {
        bno08x_yaw_offset = bno08x_data.yaw;
        bno08x_pitch_offset = bno08x_data.pitch;
        bno08x_roll_offset = bno08x_data.roll;
        bno08x_zero_pending = 0U;
    }

    bno08x_data.yaw = bno08x_normalize_angle(
        bno08x_data.yaw - bno08x_yaw_offset);
    bno08x_data.pitch = bno08x_normalize_angle(
        bno08x_data.pitch - bno08x_pitch_offset);
    bno08x_data.roll = bno08x_normalize_angle(
        bno08x_data.roll - bno08x_roll_offset);
}

void BNO08X_RequestZero(void)
{
    bno08x_zero_pending = 1U;
}

static int16_t bno08x_get_rvc_i16(uint8_t low_index)
{
    return (int16_t) ((uint16_t) bno08x_rvc_packet[low_index] |
        ((uint16_t) bno08x_rvc_packet[low_index + 1U] << 8));
}

static void bno08x_parse_rvc_packet(void)
{
    uint8_t sum = 0;

    for (uint8_t i = 2U; i <= 14U; i++) {
        sum += bno08x_rvc_packet[i];
    }

    if ((bno08x_rvc_packet[0] != 0xAAU) ||
        (bno08x_rvc_packet[1] != 0xAAU) ||
        (sum != bno08x_rvc_packet[18])) {
        bno08x_data.error_count++;
        return;
    }

    bno08x_data.index = bno08x_rvc_packet[2];
    bno08x_data.yaw = (float) bno08x_get_rvc_i16(3) / 100.0f;
    bno08x_data.pitch = (float) bno08x_get_rvc_i16(5) / 100.0f;
    bno08x_data.roll = (float) bno08x_get_rvc_i16(7) / 100.0f;
    bno08x_apply_zero();
    bno08x_data.ax = bno08x_get_rvc_i16(9);
    bno08x_data.ay = bno08x_get_rvc_i16(11);
    bno08x_data.az = bno08x_get_rvc_i16(13);
    bno08x_data.data_ready = 1U;
    bno08x_data.product_id_ok = 1U;
    bno08x_data.packet_count++;
    bno08x_data.last_update_ms = tick_ms;
}

void BNO08X_ProcessByte(uint8_t byte)
{
    if (bno08x_rvc_index == 0U && byte != 0xAAU) {
        return;
    }

    if (bno08x_rvc_index == 1U && byte != 0xAAU) {
        bno08x_rvc_index = (byte == 0xAAU) ? 1U : 0U;
        return;
    }

    bno08x_rvc_packet[bno08x_rvc_index++] = byte;

    if (bno08x_rvc_index >= sizeof(bno08x_rvc_packet)) {
        bno08x_parse_rvc_packet();
        bno08x_rvc_index = 0U;
    }
}

void BNO08X_ProcessBytes(const uint8_t *data, uint8_t len)
{
    if (data == 0) {
        return;
    }

    for (uint8_t i = 0U; i < len; i++) {
        BNO08X_ProcessByte(data[i]);
    }
}

#if defined(I2C_BNO08X_INST)
static bool bno08x_wait_idle(void)
{
    uint32_t wait = BNO08X_WAIT_COUNT;

    while ((DL_I2C_getControllerStatus(I2C_BNO08X_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--wait == 0U) {
            DL_I2C_resetControllerTransfer(I2C_BNO08X_INST);
            return false;
        }
    }
    return true;
}

static bool bno08x_i2c_write(const uint8_t *data, uint16_t len)
{
    uint16_t sent = 0U;
    uint32_t wait = BNO08X_WAIT_COUNT;

    if ((data == 0) || (len == 0U) || (bno08x_i2c_addr == 0U)) {
        return false;
    }

    if (bno08x_wait_idle() == false) {
        bno08x_i2c_last_error = 1U;
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_BNO08X_INST);
    DL_I2C_clearInterruptStatus(I2C_BNO08X_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_NACK);
    DL_I2C_startControllerTransfer(I2C_BNO08X_INST, bno08x_i2c_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);

    while (wait-- != 0U) {
        if (sent < len) {
            uint16_t fill_count = DL_I2C_fillControllerTXFIFO(I2C_BNO08X_INST,
                (uint8_t *) &data[sent], len - sent);
            if (fill_count != 0U) {
                sent += fill_count;
                wait = BNO08X_WAIT_COUNT;
            }
        }

        if (DL_I2C_getRawInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_NACK) != 0U) {
            DL_I2C_clearInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_NACK);
            if (bno08x_wait_idle() == false) {
                DL_I2C_resetControllerTransfer(I2C_BNO08X_INST);
            }
            bno08x_i2c_last_error = 2U;
            return false;
        }

        if (DL_I2C_getRawInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U) {
            DL_I2C_clearInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
            if (bno08x_wait_idle()) {
                bno08x_i2c_last_error = 0U;
                return true;
            }
            bno08x_i2c_last_error = 1U;
            return false;
        }
    }

    DL_I2C_resetControllerTransfer(I2C_BNO08X_INST);
    bno08x_i2c_last_error = 3U;
    return false;
}

static bool bno08x_scan_addr(uint8_t addr)
{
    uint8_t dummy = 0U;
    uint16_t sent = 0U;
    uint32_t wait = BNO08X_WAIT_COUNT;

    if (bno08x_wait_idle() == false) {
        bno08x_i2c_last_error = 11U;
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_BNO08X_INST);
    DL_I2C_clearInterruptStatus(I2C_BNO08X_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_NACK);
    DL_I2C_startControllerTransfer(I2C_BNO08X_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    while (wait-- != 0U) {
        if (sent == 0U) {
            sent = DL_I2C_fillControllerTXFIFO(I2C_BNO08X_INST, &dummy, 1U);
        }

        if (DL_I2C_getRawInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_NACK) != 0U) {
            DL_I2C_clearInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_NACK);
            if (bno08x_wait_idle() == false) {
                DL_I2C_resetControllerTransfer(I2C_BNO08X_INST);
            }
            bno08x_i2c_last_error = 12U;
            return false;
        }

        if (DL_I2C_getRawInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) != 0U) {
            DL_I2C_clearInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
            if (bno08x_wait_idle()) {
                bno08x_i2c_last_error = 0U;
                return true;
            }
            bno08x_i2c_last_error = 11U;
            return false;
        }
    }

    DL_I2C_resetControllerTransfer(I2C_BNO08X_INST);
    bno08x_i2c_last_error = 13U;
    return false;
}

void BNO08X_ScanBus(void)
{
    uint8_t saved_addr = bno08x_i2c_addr;

    bno08x_i2c_scan_count = 0U;
    bno08x_i2c_scan_first = 0U;
    bno08x_i2c_scan_second = 0U;
    bno08x_i2c_scl_low =
        (DL_I2C_getSCLStatus(I2C_BNO08X_INST) ==
            DL_I2C_CONTROLLER_SCL_LOW) ? 1U : 0U;
    bno08x_i2c_sda_low =
        (DL_I2C_getSDAStatus(I2C_BNO08X_INST) ==
            DL_I2C_CONTROLLER_SDA_LOW) ? 1U : 0U;

    if (bno08x_scan_addr(0x4BU)) {
        bno08x_i2c_scan_count = 1U;
        bno08x_i2c_scan_first = 0x4BU;
    } else if (bno08x_scan_addr(0x4AU)) {
        bno08x_i2c_scan_count = 1U;
        bno08x_i2c_scan_first = 0x4AU;
    }

    bno08x_i2c_scl_low |=
        (DL_I2C_getSCLStatus(I2C_BNO08X_INST) ==
            DL_I2C_CONTROLLER_SCL_LOW) ? 1U : 0U;
    bno08x_i2c_sda_low |=
        (DL_I2C_getSDAStatus(I2C_BNO08X_INST) ==
            DL_I2C_CONTROLLER_SDA_LOW) ? 1U : 0U;
    bno08x_i2c_addr = saved_addr;
}

static bool bno08x_i2c_read(uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    uint32_t wait = BNO08X_WAIT_COUNT;

    if ((data == 0) || (len == 0U) || (bno08x_i2c_addr == 0U)) {
        return false;
    }

    if (bno08x_wait_idle() == false) {
        bno08x_i2c_last_error = 21U;
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_BNO08X_INST);
    DL_I2C_clearInterruptStatus(I2C_BNO08X_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
        DL_I2C_INTERRUPT_CONTROLLER_NACK);
    DL_I2C_startControllerTransfer(I2C_BNO08X_INST, bno08x_i2c_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);

    while (count < len) {
        if (DL_I2C_getRawInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_NACK) != 0U) {
            DL_I2C_clearInterruptStatus(I2C_BNO08X_INST,
                DL_I2C_INTERRUPT_CONTROLLER_NACK);
            if (bno08x_wait_idle() == false) {
                DL_I2C_resetControllerTransfer(I2C_BNO08X_INST);
            }
            bno08x_i2c_last_error = 22U;
            return false;
        }

        if (DL_I2C_isControllerRXFIFOEmpty(I2C_BNO08X_INST) == false) {
            data[count++] = DL_I2C_receiveControllerData(I2C_BNO08X_INST);
            wait = BNO08X_WAIT_COUNT;
        } else if (--wait == 0U) {
            DL_I2C_resetControllerTransfer(I2C_BNO08X_INST);
            bno08x_i2c_last_error = 23U;
            return false;
        }
    }

    wait = BNO08X_WAIT_COUNT;
    while (DL_I2C_getRawInterruptStatus(I2C_BNO08X_INST,
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) == 0U) {
        if (--wait == 0U) {
            DL_I2C_resetControllerTransfer(I2C_BNO08X_INST);
            bno08x_i2c_last_error = 24U;
            return false;
        }
    }
    DL_I2C_clearInterruptStatus(I2C_BNO08X_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    return true;
}

static bool bno08x_send_packet(uint8_t channel, uint8_t data_len)
{
    uint8_t packet[BNO08X_MAX_PACKET_SIZE + 4U];
    uint16_t packet_len = (uint16_t) data_len + 4U;

    if (packet_len > sizeof(packet)) {
        return false;
    }

    packet[0] = (uint8_t) (packet_len & 0xFFU);
    packet[1] = (uint8_t) (packet_len >> 8);
    packet[2] = channel;
    packet[3] = bno08x_sequence[channel]++;
    memcpy(&packet[4], bno08x_shtp_data, data_len);

    return bno08x_i2c_write(packet, packet_len);
}

static bool bno08x_probe_addr(uint8_t addr)
{
    bno08x_i2c_addr = addr;
    bno08x_i2c_online = true;
    return true;
}

static bool bno08x_receive_packet(bool count_error);

static bool bno08x_soft_reset(void)
{
    bno08x_shtp_data[0] = 1U;
    if (bno08x_send_packet(BNO08X_CHANNEL_EXECUTABLE, 1U) == false) {
        bno08x_i2c_last_error = 31U;
        return false;
    }
    mspm0_delay_ms(100);

    bno08x_shtp_data[0] = 1U;
    (void) bno08x_send_packet(BNO08X_CHANNEL_EXECUTABLE, 1U);
    mspm0_delay_ms(100);

    return true;
}

static bool bno08x_receive_packet(bool count_error)
{
    uint16_t packet_len;
    uint16_t data_len;
    uint16_t data_spot = 0U;

    if (bno08x_i2c_read(bno08x_shtp_header, 4U) == false) {
        if (count_error) {
            bno08x_data.error_count++;
        }
        return false;
    }

    packet_len = ((uint16_t) bno08x_shtp_header[1] << 8) |
        bno08x_shtp_header[0];
    packet_len &= ~(1U << 15);

    if (packet_len <= 4U) {
        return false;
    }

    data_len = packet_len - 4U;
    if (data_len > BNO08X_MAX_PACKET_SIZE) {
        data_len = BNO08X_MAX_PACKET_SIZE;
    }

    while (data_spot < data_len) {
        uint8_t rx[BNO08X_I2C_CHUNK_SIZE];
        uint16_t chunk = data_len - data_spot;

        if (chunk > (BNO08X_I2C_CHUNK_SIZE - 4U)) {
            chunk = BNO08X_I2C_CHUNK_SIZE - 4U;
        }

        if (bno08x_i2c_read(rx, chunk + 4U) == false) {
            if (count_error) {
                bno08x_data.error_count++;
            }
            return false;
        }

        memcpy(&bno08x_shtp_data[data_spot], &rx[4], chunk);
        data_spot += chunk;
    }

    return true;
}
#endif

#if !defined(I2C_BNO08X_INST)
void BNO08X_ScanBus(void)
{
    bno08x_i2c_scan_count = 0U;
    bno08x_i2c_scan_first = 0U;
    bno08x_i2c_scan_second = 0U;
    bno08x_i2c_scl_low = 0U;
    bno08x_i2c_sda_low = 0U;
}
#endif

static float bno08x_q_to_float(int16_t value, uint8_t q)
{
    return (float) value / (float) (1UL << q);
}

static int16_t bno08x_get_i16(uint8_t low_index)
{
    return (int16_t) ((uint16_t) bno08x_shtp_data[low_index] |
        ((uint16_t) bno08x_shtp_data[low_index + 1U] << 8));
}

static void bno08x_quat_to_euler(void)
{
    float qi = bno08x_data.quat_i;
    float qj = bno08x_data.quat_j;
    float qk = bno08x_data.quat_k;
    float qr = bno08x_data.quat_real;
    float sqr = qr * qr;
    float sqi = qi * qi;
    float sqj = qj * qj;
    float sqk = qk * qk;
    float denom = sqi + sqj + sqk + sqr;
    const float rad_to_deg = 57.2957795f;

    if (denom <= 0.000001f) {
        return;
    }

    bno08x_data.yaw = atan2f(2.0f * (qi * qj + qk * qr),
        (sqi - sqj - sqk + sqr)) * rad_to_deg;
    bno08x_data.pitch = asinf(-2.0f * (qi * qk - qj * qr) / denom) *
        rad_to_deg;
    bno08x_data.roll = atan2f(2.0f * (qj * qk + qi * qr),
        (-sqi - sqj + sqk + sqr)) * rad_to_deg;
}

static bool bno08x_parse_input_report(void)
{
    uint8_t report_id;
    int16_t raw_i;
    int16_t raw_j;
    int16_t raw_k;
    int16_t raw_real;

    if ((bno08x_shtp_header[2] != BNO08X_CHANNEL_REPORTS) ||
        (bno08x_shtp_data[0] != BNO08X_REPORT_BASE_TIMESTAMP)) {
        return false;
    }

    report_id = bno08x_shtp_data[5];
    if (report_id != BNO08X_SENSOR_ROTATION_VECTOR) {
        return false;
    }

    bno08x_data.accuracy = bno08x_shtp_data[7] & 0x03U;
    raw_i = bno08x_get_i16(9);
    raw_j = bno08x_get_i16(11);
    raw_k = bno08x_get_i16(13);
    raw_real = bno08x_get_i16(15);

    bno08x_data.quat_i = bno08x_q_to_float(raw_i, 14);
    bno08x_data.quat_j = bno08x_q_to_float(raw_j, 14);
    bno08x_data.quat_k = bno08x_q_to_float(raw_k, 14);
    bno08x_data.quat_real = bno08x_q_to_float(raw_real, 14);
    bno08x_quat_to_euler();
    bno08x_apply_zero();

    bno08x_data.data_ready = 1U;
    bno08x_data.packet_count++;
    bno08x_data.last_update_ms = tick_ms;
    bno08x_i2c_last_error = 0U;
    return true;
}

static bool bno08x_request_product_id(void)
{
#if defined(I2C_BNO08X_INST)
    bno08x_shtp_data[0] = BNO08X_REPORT_PRODUCT_ID_REQ;
    bno08x_shtp_data[1] = 0U;
    if (bno08x_send_packet(BNO08X_CHANNEL_CONTROL, 2U) == false) {
        return false;
    }

    for (uint8_t i = 0; i < 10U; i++) {
        mspm0_delay_ms(5);
        if (bno08x_receive_packet(true) &&
            bno08x_shtp_data[0] == BNO08X_REPORT_PRODUCT_ID_RESP) {
            bno08x_data.product_id_ok = 1U;
            return true;
        }
    }
#endif
    return false;
}

static bool bno08x_enable_rotation_vector(uint16_t interval_ms)
{
#if defined(I2C_BNO08X_INST)
    uint32_t interval_us = (uint32_t) interval_ms * 1000UL;

    memset(bno08x_shtp_data, 0, 17U);
    bno08x_shtp_data[0] = BNO08X_REPORT_SET_FEATURE;
    bno08x_shtp_data[1] = BNO08X_SENSOR_ROTATION_VECTOR;
    bno08x_shtp_data[5] = (uint8_t) (interval_us & 0xFFU);
    bno08x_shtp_data[6] = (uint8_t) ((interval_us >> 8) & 0xFFU);
    bno08x_shtp_data[7] = (uint8_t) ((interval_us >> 16) & 0xFFU);
    bno08x_shtp_data[8] = (uint8_t) ((interval_us >> 24) & 0xFFU);

    return bno08x_send_packet(BNO08X_CHANNEL_CONTROL, 17U);
#else
    (void) interval_ms;
    return false;
#endif
}

bool BNO08X_Probe(void)
{
    bno08x_i2c_online = false;
    bno08x_i2c_addr = 0U;

#if defined(I2C_BNO08X_INST)
    if (bno08x_i2c_scan_first == 0x4BU) {
        (void) bno08x_probe_addr(0x4BU);
    } else if (bno08x_i2c_scan_first == 0x4AU) {
        (void) bno08x_probe_addr(0x4AU);
    } else if (bno08x_scan_addr(0x4BU)) {
        (void) bno08x_probe_addr(0x4BU);
    } else if (bno08x_scan_addr(0x4AU)) {
        (void) bno08x_probe_addr(0x4AU);
    }

    if (bno08x_i2c_addr != 0U) {
        bno08x_i2c_online = true;
    }
#endif

    return bno08x_i2c_online;
}

void BNO08X_Init(void)
{
    memset(&bno08x_data, 0, sizeof(bno08x_data));
    memset(bno08x_dmaBuffer, 0, sizeof(bno08x_dmaBuffer));
    memset(bno08x_sequence, 0, sizeof(bno08x_sequence));
    bno08x_rvc_index = 0U;
    bno08x_init_stage = 1U;
    bno08x_i2c_last_error = 0U;
    bno08x_yaw_offset = 0.0f;
    bno08x_pitch_offset = 0.0f;
    bno08x_roll_offset = 0.0f;
    bno08x_zero_pending = 1U;

#if defined(UART_BNO08X_INST) && defined(DMA_BNO08X_CHAN_ID)
    bno08x_i2c_online = true;
    bno08x_i2c_addr = 0U;
    bno08x_init_stage = 7U;

    DL_DMA_setSrcAddr(DMA, DMA_BNO08X_CHAN_ID,
        (uint32_t) (&UART_BNO08X_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID,
        (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);

    NVIC_ClearPendingIRQ(UART_BNO08X_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BNO08X_INST_INT_IRQN);
    return;
#elif defined(I2C_BNO08X_INST)
    /* The 2026 PCB BNO08X wiring has been verified at 7-bit address 0x4B.
     * Do not auto-fallback to 0x4A here; a transient reset NACK would make the
     * display show the wrong address and stop an otherwise working sensor.
     */
    bno08x_i2c_addr = 0x4BU;
    bno08x_i2c_online = true;
    bno08x_i2c_scan_count = 1U;
    bno08x_i2c_scan_first = 0x4BU;
    bno08x_i2c_scan_second = 0U;
    bno08x_i2c_scl_low =
        (DL_I2C_getSCLStatus(I2C_BNO08X_INST) ==
            DL_I2C_CONTROLLER_SCL_LOW) ? 1U : 0U;
    bno08x_i2c_sda_low =
        (DL_I2C_getSDAStatus(I2C_BNO08X_INST) ==
            DL_I2C_CONTROLLER_SDA_LOW) ? 1U : 0U;
    bno08x_init_stage = 2U;
    mspm0_delay_ms(300);

    if (bno08x_soft_reset() == false) {
        bno08x_init_stage = 4U;
        mspm0_delay_ms(100);
    }
    if (bno08x_request_product_id() == false) {
        mspm0_delay_ms(20);
    }
    if ((bno08x_data.product_id_ok == 0U) &&
        (bno08x_request_product_id() == false)) {
        bno08x_init_stage = 5U;
    }

    mspm0_delay_ms(200);
    bool feature_enabled = bno08x_enable_rotation_vector(50U);
    if (feature_enabled == false) {
        DL_I2C_resetControllerTransfer(I2C_BNO08X_INST);
        mspm0_delay_ms(20);
        feature_enabled = bno08x_enable_rotation_vector(50U);
    }
    if (feature_enabled == false) {
        mspm0_delay_ms(200);
        (void) bno08x_request_product_id();
        feature_enabled = bno08x_enable_rotation_vector(50U);
    }
    if (feature_enabled == false) {
        bno08x_i2c_online = false;
        bno08x_init_stage = 6U;
        return;
    }
    bno08x_init_stage = 7U;
    bno08x_i2c_last_error = 0U;
#else
    bno08x_init_stage = 0U;
#endif
}

bool BNO08X_Update(void)
{
#if defined(UART_BNO08X_INST)
    return false;
#endif

#if defined(I2C_BNO08X_INST)
    if (bno08x_i2c_online == false) {
        return false;
    }

    if (bno08x_receive_packet(true) == false) {
        return false;
    }

    return bno08x_parse_input_report();
#else
    return false;
#endif
}
