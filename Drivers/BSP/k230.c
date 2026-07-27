#include "k230.h"
#include "ti_msp_dl_config.h"
// 接收缓冲区和状态变量
 uint8_t  rxBuf[7];               // 7字节数据包缓冲区（符合协议格式）
 uint8_t  idx   = 0;              // 接收字节计数
 uint8_t  frame_ok = 0;  // 帧接收完成标志（volatile确保多线程可见）
uint16_t target_x = 0;  // 目标X坐标（使用无符号类型匹配发送端）
 uint16_t target_y = 0;  // 目标Y坐标



/**
 * 串口配置函数
 * 初始化中断相关设置
 */
void k230_usart_config(void)
{
    // 清除串口中断标志
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    // 使能串口中断
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}

/**
 * 串口接收中断处理函数
 * 实现数据包解析和帧同步
 */
void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_1_INST))
    {
        case DL_UART_IIDX_RX:
        {
            uint8_t byte = DL_UART_Main_receiveData(UART_1_INST);

            // 帧同步机制：检测到帧头0xAA时重置接收计数
            if (byte == 0xAA) {
                idx = 0;  // 重新开始接收新帧
            }

            // 填充缓冲区（防止溢出）
            if (idx < 7) {
                rxBuf[idx++] = byte;
            }

            // 完整接收一帧数据（7字节）
            if (idx == 7) {
                // 校验帧头、帧尾和中间标识位
                if (rxBuf[0] == 0xAA &&      // 帧头校验
                    rxBuf[1] == 0xFF &&      // 数据标识校验
                    rxBuf[6] == 0xEE) {      // 帧尾校验

                    // 小端模式解析X坐标（低字节在前，高字节在后）
                    target_x = (uint16_t)(rxBuf[3] << 8 | rxBuf[2]);
                    // 小端模式解析Y坐标
                    target_y = (uint16_t)(rxBuf[5] << 8 | rxBuf[4]);
                    frame_ok = 1;  // 设置帧有效标志
                }
                idx = 0;  // 重置计数器，准备接收下一帧
            }
            break;
        }

        default:
            break;
    }
}