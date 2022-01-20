#include "main.h"

void setAlarm(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime);
void sleepEnd(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime, UART_HandleTypeDef *ESPhuart, uint8_t *selectedElemIndex, uint8_t *scrollCount,
    uint8_t *buttonIsPressed, uint32_t *lastInteractionTime, UART_HandleTypeDef *TestHuart);
void sleepBegin(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime, UART_HandleTypeDef *ESPhuart, uint8_t *selectedElemIndex, uint8_t *scrollCount,
    uint8_t *buttonIsPressed, uint32_t *lastInteractionTime, UART_HandleTypeDef *TestHuart);