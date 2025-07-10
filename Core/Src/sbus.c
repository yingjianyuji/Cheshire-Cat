#include "sbus.h"
#include <string.h>

void SBUS_Init(UART_HandleTypeDef *huart) {
    // UART配置应在CubeMX中完成，参数为：
    // BaudRate: 100000
    // WordLength: UART_WORDLENGTH_9B
    // StopBits: UART_STOPBITS_2
    // Parity: UART_PARITY_EVEN
    // Mode: UART_MODE_TX_RX
    // HardwareFlowControl: UART_HWCONTROL_NONE
}

void SBUS_PreparePacket(SBUS_Data *sbusData, uint8_t packet[SBUS_FRAME_LENGTH]) {
    /* 清空数据包 */
    memset(packet, 0, SBUS_FRAME_LENGTH);
    
    /* 设置起始字节 */
    packet[0] = SBUS_START_BYTE;
    
    /* 编码16个通道数据(每个通道11位) */
    packet[1] = (uint8_t)(sbusData->channels[0] & 0xFF);
    packet[2] = (uint8_t)((sbusData->channels[0] >> 8 | sbusData->channels[1] << 3) & 0xFF);
    packet[3] = (uint8_t)((sbusData->channels[1] >> 5 | sbusData->channels[2] << 6) & 0xFF);
    packet[4] = (uint8_t)((sbusData->channels[2] >> 2) & 0xFF);
    packet[5] = (uint8_t)((sbusData->channels[2] >> 10 | sbusData->channels[3] << 1) & 0xFF);
    packet[6] = (uint8_t)((sbusData->channels[3] >> 7 | sbusData->channels[4] << 4) & 0xFF);
    packet[7] = (uint8_t)((sbusData->channels[4] >> 4 | sbusData->channels[5] << 7) & 0xFF);
    packet[8] = (uint8_t)((sbusData->channels[5] >> 1) & 0xFF);
    packet[9] = (uint8_t)((sbusData->channels[5] >> 9 | sbusData->channels[6] << 2) & 0xFF);
    packet[10] = (uint8_t)((sbusData->channels[6] >> 6 | sbusData->channels[7] << 5) & 0xFF);
    packet[11] = (uint8_t)((sbusData->channels[7] >> 3) & 0xFF);
    packet[12] = (uint8_t)(sbusData->channels[8] & 0xFF);
    packet[13] = (uint8_t)((sbusData->channels[8] >> 8 | sbusData->channels[9] << 3) & 0xFF);
    packet[14] = (uint8_t)((sbusData->channels[9] >> 5 | sbusData->channels[10] << 6) & 0xFF);
    packet[15] = (uint8_t)((sbusData->channels[10] >> 2) & 0xFF);
    packet[16] = (uint8_t)((sbusData->channels[10] >> 10 | sbusData->channels[11] << 1) & 0xFF);
    packet[17] = (uint8_t)((sbusData->channels[11] >> 7 | sbusData->channels[12] << 4) & 0xFF);
    packet[18] = (uint8_t)((sbusData->channels[12] >> 4 | sbusData->channels[13] << 7) & 0xFF);
    packet[19] = (uint8_t)((sbusData->channels[13] >> 1) & 0xFF);
    packet[20] = (uint8_t)((sbusData->channels[13] >> 9 | sbusData->channels[14] << 2) & 0xFF);
    packet[21] = (uint8_t)((sbusData->channels[14] >> 6 | sbusData->channels[15] << 5) & 0xFF);
    packet[22] = (uint8_t)((sbusData->channels[15] >> 3) & 0xFF);
    
    /* 设置标志位(第23字节) */
    packet[23] = 0x00;
    if(sbusData->digital_channel_17) packet[23] |= (1 << 0);
    if(sbusData->digital_channel_18) packet[23] |= (1 << 1);
    if(sbusData->frame_lost) packet[23] |= (1 << 2);
    if(sbusData->failsafe) packet[23] |= (1 << 3);
    
    /* 设置结束字节 */
    packet[24] = SBUS_END_BYTE;
}

void SBUS_SendPacket(UART_HandleTypeDef *huart, uint8_t packet[SBUS_FRAME_LENGTH]) {
    HAL_UART_Transmit_DMA(huart, packet, SBUS_FRAME_LENGTH);
}