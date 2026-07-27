/*
 * HC05/JDY31 Bluetooth UART driver and three-PID tuning protocol.
 *
 * UART ISR ownership is intentionally small: received bytes are copied into
 * a software ring. Frame parsing, PID updates, replies, and CSV telemetry run
 * from Bluetooth_Poll() in the main loop.
 */

#include "bsp_hc05.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "clock.h"
#include "pid_control.h"
#include "ti_msp_dl_config.h"

#define BLUETOOTH_RX_QUEUE_LEN 64U
#define BLUETOOTH_RX_QUEUE_MASK (BLUETOOTH_RX_QUEUE_LEN - 1U)
#define BLUETOOTH_FRAME_MAX 31U
#define BLUETOOTH_CSV_PERIOD_MS 50U

typedef enum {
    BLUETOOTH_FRAME_ERROR_NONE = 0,
    BLUETOOTH_FRAME_ERROR_FORMAT,
    BLUETOOTH_FRAME_ERROR_LENGTH
} bluetooth_frame_error_t;

_Static_assert((BLUETOOTH_RX_QUEUE_LEN & BLUETOOTH_RX_QUEUE_MASK) == 0U,
    "Bluetooth RX queue length must be a power of two");

uint8_t Bluetooth_ConnectFlag;
uint8_t BLERX_BUFF[BLERX_LEN_MAX];
uint8_t BLERX_FLAG;
uint8_t BLERX_LEN;
volatile uint32_t Bluetooth_RxCount;
volatile uint32_t Bluetooth_TxCount;
volatile uint32_t Bluetooth_IrqCount;
volatile uint32_t Bluetooth_PollCount;
volatile uint32_t Bluetooth_OverflowCount;
volatile uint8_t Bluetooth_LastByte;
uint8_t Bluetooth_LastText[BLUETOOTH_LAST_TEXT_LEN + 1U];
volatile uint8_t Bluetooth_LastTextLen;

static uint8_t bluetooth_rx_queue[BLUETOOTH_RX_QUEUE_LEN];
static volatile uint8_t bluetooth_rx_head;
static volatile uint8_t bluetooth_rx_tail;

static char bluetooth_frame[BLUETOOTH_FRAME_MAX + 1U];
static uint8_t bluetooth_frame_length;
static bool bluetooth_discard_frame;
static bluetooth_frame_error_t bluetooth_discard_reason;
static bool bluetooth_csv_enabled;
static uint32_t bluetooth_last_csv_ms;

static void Bluetooth_SendError(const char *reason)
{
    Send_Bluetooth_Data("ERR,");
    Send_Bluetooth_Data(reason);
    Send_Bluetooth_Data("!\r\n");
}

static void Bluetooth_SendOk(void)
{
    Send_Bluetooth_Data("OK!\r\n");
}

/* FireWater：通道0=左轮目标，通道1=左轮实际，单位均为脉冲/50ms。 */
static void Bluetooth_SendFireWater(void)
{
    char message[48];
    pid_control_status_t status;

    pid_control_get_status(&status);
    (void)snprintf(message, sizeof(message), "pid:%ld,%ld\n",
        (long)status.left_target, (long)status.left_now);
    Send_Bluetooth_Data(message);
}

static bool Bluetooth_ParseNumber(const char *text, float *value_out)
{
    bool negative = false;
    bool decimal_seen = false;
    bool digit_seen = false;
    float value = 0.0f;
    float decimal_scale = 0.1f;

    if (text == NULL || value_out == NULL || *text == '\0') {
        return false;
    }

    if (*text == '-' || *text == '+') {
        negative = (*text == '-');
        text++;
    }

    while (*text != '\0') {
        if (*text >= '0' && *text <= '9') {
            uint8_t digit = (uint8_t)(*text - '0');
            digit_seen = true;
            if (decimal_seen) {
                value += (float)digit * decimal_scale;
                decimal_scale *= 0.1f;
            } else {
                value = value * 10.0f + (float)digit;
                if (value > 1000000.0f) {
                    return false;
                }
            }
        } else if (*text == '.' && !decimal_seen) {
            decimal_seen = true;
        } else {
            return false;
        }
        text++;
    }

    if (!digit_seen) {
        return false;
    }

    *value_out = negative ? -value : value;
    return true;
}

static bool Bluetooth_ApplyGain(const char *command, float value)
{
    pid_control_set_gain(command, value);
    return true;
}

static void Bluetooth_ProcessFrame(char *frame)
{
    char *equals;
    float value;
    bool applied;

    if (strcmp(frame, "Q") == 0) {
        Bluetooth_ConnectFlag = 1U;
        Bluetooth_SendFireWater();
        return;
    }

    if (strcmp(frame, "STOP") == 0) {
        pid_control_stop();
        Bluetooth_ConnectFlag = 1U;
        Bluetooth_SendOk();
        return;
    }

    equals = strchr(frame, '=');
    if (equals == NULL || equals == frame || equals[1] == '\0' ||
        strchr(equals + 1, '=') != NULL) {
        Bluetooth_SendError("FMT");
        return;
    }

    *equals = '\0';
    if (!Bluetooth_ParseNumber(equals + 1, &value)) {
        Bluetooth_SendError("FMT");
        return;
    }

    /* RUN commands are intentionally unsupported; use STOP! for a safe stop. */
    if ((strcmp(frame, "P1") == 0 || strcmp(frame, "I1") == 0 ||
         strcmp(frame, "D1") == 0 || strcmp(frame, "P2") == 0 ||
         strcmp(frame, "I2") == 0 || strcmp(frame, "D2") == 0)) {
        applied = Bluetooth_ApplyGain(frame, value);
    } else if (strcmp(frame, "It") == 0 || strcmp(frame, "T1") == 0 ||
               strcmp(frame, "AT") == 0) {
        pid_control_set_angle_target(value);
        applied = true;
    } else if (strcmp(frame, "Ot") == 0 || strcmp(frame, "T2") == 0 ||
               strcmp(frame, "ST") == 0 || strcmp(frame, "T") == 0) {
        /* 公共速度带符号，单位为编码器脉冲/50 ms。 */
        if (value >= -PID_CONTROL_SPEED_TARGET_LIMIT &&
            value <= PID_CONTROL_SPEED_TARGET_LIMIT) {
            pid_control_set_speed_target(value);
            applied = true;
        } else {
            applied = false;
        }
    } else if (strcmp(frame, "CSV") == 0) {
        if (value == 1.0f) {
            bluetooth_csv_enabled = true;
            bluetooth_last_csv_ms = (uint32_t)tick_ms;
            applied = true;
        } else if (value == 0.0f) {
            bluetooth_csv_enabled = false;
            applied = true;
        } else {
            applied = false;
        }
    } else {
        Bluetooth_SendError("CMD");
        return;
    }

    if (!applied) {
        Bluetooth_SendError("RANGE");
        return;
    }

    Bluetooth_ConnectFlag = 1U;
    Bluetooth_SendOk();
}

static void Bluetooth_RecordByte(uint8_t byte)
{
    uint8_t len;

    Bluetooth_RxCount++;
    Bluetooth_LastByte = byte;

    if (byte < 32U || byte > 126U) {
        byte = ' ';
    }

    len = Bluetooth_LastTextLen;
    if (len < BLUETOOTH_LAST_TEXT_LEN) {
        Bluetooth_LastText[len++] = byte;
    } else {
        memmove(&Bluetooth_LastText[0], &Bluetooth_LastText[1],
            BLUETOOTH_LAST_TEXT_LEN - 1U);
        Bluetooth_LastText[BLUETOOTH_LAST_TEXT_LEN - 1U] = byte;
    }

    Bluetooth_LastText[BLUETOOTH_LAST_TEXT_LEN] = '\0';
    Bluetooth_LastTextLen = len;
}

static void Bluetooth_RecordFrameByte(uint8_t byte)
{
    if (BLERX_FLAG != 0U && byte != '\r' && byte != '\n') {
        Clear_BLERX_BUFF();
    }

    if (byte == '\r' || byte == '\n') {
        return;
    }

    if (BLERX_LEN < BLERX_LEN_MAX - 1U) {
        BLERX_BUFF[BLERX_LEN++] = byte;
        BLERX_BUFF[BLERX_LEN] = '\0';
    } else {
        Bluetooth_OverflowCount++;
        Clear_BLERX_BUFF();
    }

    if (byte == '!') {
        BLERX_FLAG = 1U;
    }
}

static void Bluetooth_QueueByte(uint8_t byte)
{
    uint8_t next_head = (uint8_t)((bluetooth_rx_head + 1U) & BLUETOOTH_RX_QUEUE_MASK);

    if (next_head == bluetooth_rx_tail) {
        Bluetooth_OverflowCount++;
        return;
    }

    bluetooth_rx_queue[bluetooth_rx_head] = byte;
    bluetooth_rx_head = next_head;
    Bluetooth_IrqCount++;
}

void BLE_Send_Bit(uint8_t ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true) {
    }
    DL_UART_Main_transmitData(UART_0_INST, ch);
    Bluetooth_TxCount++;
}

void BLE_send_String(const uint8_t *str)
{
    while (str != NULL && *str != '\0') {
        BLE_Send_Bit(*str++);
    }
}

void Clear_BLERX_BUFF(void)
{
    BLERX_LEN = 0U;
    BLERX_FLAG = 0U;
    BLERX_BUFF[0] = '\0';
}

void Bluetooth_Init(void)
{
    Bluetooth_ConnectFlag = 0U;
    Bluetooth_RxCount = 0U;
    Bluetooth_TxCount = 0U;
    Bluetooth_IrqCount = 0U;
    Bluetooth_PollCount = 0U;
    Bluetooth_OverflowCount = 0U;
    Bluetooth_LastByte = 0U;
    Bluetooth_LastTextLen = 0U;
    memset(Bluetooth_LastText, 0, sizeof(Bluetooth_LastText));
    Clear_BLERX_BUFF();

    bluetooth_rx_head = 0U;
    bluetooth_rx_tail = 0U;
    bluetooth_frame_length = 0U;
    bluetooth_discard_frame = false;
    bluetooth_discard_reason = BLUETOOTH_FRAME_ERROR_NONE;
    bluetooth_csv_enabled = false;
    bluetooth_last_csv_ms = (uint32_t)tick_ms;

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void Bluetooth_ProcessReceivedByte(uint8_t byte)
{
    Bluetooth_RecordByte(byte);
    Bluetooth_RecordFrameByte(byte);

    if (byte == '\r' || byte == '\n') {
        return;
    }

    if (bluetooth_discard_frame) {
        if (byte == '!') {
            if (bluetooth_discard_reason == BLUETOOTH_FRAME_ERROR_LENGTH) {
                Bluetooth_SendError("LEN");
            } else {
                Bluetooth_SendError("FMT");
            }
            bluetooth_discard_frame = false;
            bluetooth_discard_reason = BLUETOOTH_FRAME_ERROR_NONE;
            bluetooth_frame_length = 0U;
        }
        return;
    }

    if (byte == '!') {
        bluetooth_frame[bluetooth_frame_length] = '\0';
        if (bluetooth_frame_length == 0U) {
            Bluetooth_SendError("FMT");
        } else {
            Bluetooth_ProcessFrame(bluetooth_frame);
        }
        bluetooth_frame_length = 0U;
        return;
    }

    if (byte < 32U || byte > 126U) {
        bluetooth_discard_frame = true;
        bluetooth_discard_reason = BLUETOOTH_FRAME_ERROR_FORMAT;
        bluetooth_frame_length = 0U;
        return;
    }

    if (bluetooth_frame_length >= BLUETOOTH_FRAME_MAX) {
        Bluetooth_OverflowCount++;
        bluetooth_discard_frame = true;
        bluetooth_discard_reason = BLUETOOTH_FRAME_ERROR_LENGTH;
        bluetooth_frame_length = 0U;
        return;
    }

    bluetooth_frame[bluetooth_frame_length++] = (char)byte;
}

void Bluetooth_Poll(void)
{
    uint32_t now_ms;

    while (bluetooth_rx_tail != bluetooth_rx_head) {
        uint8_t byte = bluetooth_rx_queue[bluetooth_rx_tail];
        bluetooth_rx_tail = (uint8_t)((bluetooth_rx_tail + 1U) & BLUETOOTH_RX_QUEUE_MASK);
        Bluetooth_PollCount++;
        Bluetooth_ProcessReceivedByte(byte);
    }

    now_ms = (uint32_t)tick_ms;
    if (bluetooth_csv_enabled &&
        (uint32_t)(now_ms - bluetooth_last_csv_ms) >= BLUETOOTH_CSV_PERIOD_MS) {
        bluetooth_last_csv_ms = now_ms;
        Bluetooth_SendFireWater();
    }
}

uint8_t Get_Bluetooth_ConnectFlag(void)
{
    return Bluetooth_ConnectFlag;
}

int Receive_Bluetooth_Data(void)
{
    return BLERX_FLAG != 0U ? 1 : 0;
}

void Send_Bluetooth_Data(const char *dat)
{
    BLE_send_String((const uint8_t *)dat);
}

void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_0_INST) == DL_UART_IIDX_RX) {
        while (DL_UART_isRXFIFOEmpty(UART_0_INST) == false) {
            Bluetooth_QueueByte(DL_UART_Main_receiveData(UART_0_INST));
        }
    }
}
