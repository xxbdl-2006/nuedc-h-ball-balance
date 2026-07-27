#include "wit.h"
#include "clock.h"

uint8_t wit_dmaBuffer[33];

volatile WIT_Data_t wit_data;

static uint8_t wit_packet[11];
static uint8_t wit_packet_index;
static float wit_yaw_offset;
static float wit_pitch_offset;
static float wit_roll_offset;
static uint8_t wit_zero_pending = 1U;

static float wit_normalize_angle(float angle)
{
    while(angle > 180.0f) {
        angle -= 360.0f;
    }

    while(angle < -180.0f) {
        angle += 360.0f;
    }

    return angle;
}

static void wit_apply_zero(void)
{
    if(wit_zero_pending != 0U) {
        wit_yaw_offset = wit_data.yaw;
        wit_pitch_offset = wit_data.pitch;
        wit_roll_offset = wit_data.roll;
        wit_zero_pending = 0U;
    }

    wit_data.yaw = wit_normalize_angle(wit_data.yaw - wit_yaw_offset);
    wit_data.pitch = wit_normalize_angle(wit_data.pitch - wit_pitch_offset);
    wit_data.roll = wit_normalize_angle(wit_data.roll - wit_roll_offset);
}

void WIT_RequestZero(void)
{
    wit_zero_pending = 1U;
}

static int16_t wit_get_i16(uint8_t low_index)
{
    return (int16_t) ((uint16_t) wit_packet[low_index] |
        ((uint16_t) wit_packet[low_index + 1] << 8));
}

static void wit_parse_packet(void)
{
    uint8_t sum = 0;

    for(uint8_t i = 0; i < 10; i++)
        sum += wit_packet[i];

    if(sum != wit_packet[10])
    {
        wit_data.checksum_error_count++;
        return;
    }

    switch(wit_packet[1])
    {
        case 0x51:
            wit_data.ax = (float) wit_get_i16(2) / 2.048f;
            wit_data.ay = (float) wit_get_i16(4) / 2.048f;
            wit_data.az = (float) wit_get_i16(6) / 2.048f;
            wit_data.temperature = (float) wit_get_i16(8) / 100.0f;
            wit_data.updated_flags |= WIT_FRAME_ACCEL;
            break;

        case 0x52:
            wit_data.gx = (float) wit_get_i16(2) / 16.384f;
            wit_data.gy = (float) wit_get_i16(4) / 16.384f;
            wit_data.gz = (float) wit_get_i16(6) / 16.384f;
            wit_data.updated_flags |= WIT_FRAME_GYRO;
            break;

        case 0x53:
            wit_data.roll  = (float) wit_get_i16(2) / 32768.0f * 180.0f;
            wit_data.pitch = (float) wit_get_i16(4) / 32768.0f * 180.0f;
            wit_data.yaw   = (float) wit_get_i16(6) / 32768.0f * 180.0f;
            wit_apply_zero();
            wit_data.version = wit_get_i16(8);
            wit_data.updated_flags |= WIT_FRAME_ANGLE;
            break;

        default:
            return;
    }

    wit_data.packet_count++;
    wit_data.last_update_ms = tick_ms;
}

void WIT_Init(void)
{
    wit_packet_index = 0;
    wit_data.updated_flags = 0;
    wit_data.packet_count = 0;
    wit_data.checksum_error_count = 0;
    wit_data.last_update_ms = 0;
    wit_yaw_offset = 0.0f;
    wit_pitch_offset = 0.0f;
    wit_roll_offset = 0.0f;
    wit_zero_pending = 1U;

    DL_DMA_setSrcAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t)(&UART_WIT_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);

    NVIC_EnableIRQ(UART_WIT_INST_INT_IRQN);
}

void WIT_ProcessByte(uint8_t byte)
{
    if(wit_packet_index == 0 && byte != 0x55)
        return;

    wit_packet[wit_packet_index++] = byte;

    if(wit_packet_index >= sizeof(wit_packet))
    {
        wit_parse_packet();
        wit_packet_index = 0;
    }
}

void WIT_ProcessBytes(const uint8_t *data, uint8_t len)
{
    if(data == 0)
        return;

    for(uint8_t i = 0; i < len; i++)
        WIT_ProcessByte(data[i]);
}

void WIT_ClearUpdatedFlags(uint8_t flags)
{
    wit_data.updated_flags &= (uint8_t) ~flags;
}

void WIT_SendBytes(const uint8_t *data, uint8_t len)
{
    if(data == 0)
        return;

    for(uint8_t i = 0; i < len; i++)
    {
        while(DL_UART_isBusy(UART_WIT_INST) == true);
        DL_UART_Main_transmitData(UART_WIT_INST, data[i]);
    }
}
