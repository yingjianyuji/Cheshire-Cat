#ifndef SBUS_H
#define SBUS_H

#include "stm32h7xx_hal.h"

#define SBUS_FRAME_LENGTH 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00
#define SBUS_CHANNEL_MIN 172
#define SBUS_CHANNEL_MAX 1811
#define SBUS_CHANNEL_NEUTRAL 992

typedef struct {
    uint16_t channels[16];
    uint8_t digital_channel_17;
    uint8_t digital_channel_18;
    uint8_t frame_lost;
    uint8_t failsafe;
} SBUS_Data;

void SBUS_Init(UART_HandleTypeDef *huart);
void SBUS_PreparePacket(SBUS_Data *sbusData, uint8_t packet[SBUS_FRAME_LENGTH]);
void SBUS_SendPacket(UART_HandleTypeDef *huart, uint8_t packet[SBUS_FRAME_LENGTH]);

#endif