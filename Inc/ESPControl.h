#include "stm32f1xx_hal.h"


void TxESP(UART_HandleTypeDef *huart, char *command, char *pData, uint16_t Size);
void RxESP(UART_HandleTypeDef *huart, uint8_t resetOnExept, char *pData, uint16_t Size);
void WakeupESP(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime, UART_HandleTypeDef *huart);
void SleepESP(UART_HandleTypeDef *huart);
void ResetESP(UART_HandleTypeDef *huart);
void InitRealTime(UART_HandleTypeDef *huart, RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime);
//void GetLastEnergy(UART_HandleTypeDef *huarto, double *lastEnergy);