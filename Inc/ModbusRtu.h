#include "stm32f1xx_hal.h"
#include "string.h"
#include "stdio.h"

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))


void StartMB();
char* MBGetQuery(UART_HandleTypeDef *huart,  uint16_t regAdress, uint16_t regNum);
char* getRxBuffer(UART_HandleTypeDef *huart, uint8_t regNum);
void get_FC3(uint8_t regNum);
void sendTxBuffer(UART_HandleTypeDef *huart, uint8_t regNum);

uint16_t calcCRC(uint8_t u8length);

