#include "controllerSleep.h"

void setAlarm(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime) {
    RTC_AlarmTypeDef myAlarm = {0};
    if(((realTime->Hours == 22 && realTime->Minutes > 30) || (realTime->Hours > 22 && realTime->Hours <= 23)) ||
    (realTime->Hours >= 0 && realTime->Hours < 3) || (realTime->Hours == 3 && realTime->Minutes <= 59))
    {
        myAlarm.AlarmTime.Hours = 4;
        myAlarm.AlarmTime.Minutes = 0;
        myAlarm.AlarmTime.Seconds = 0;
    }else
    {
        if(realTime->Minutes >= 0 && realTime->Minutes <=19)
        {
            myAlarm.AlarmTime.Hours = realTime->Hours;
            myAlarm.AlarmTime.Minutes = 20;
            myAlarm.AlarmTime.Seconds = 0;
        }else if(realTime->Minutes >= 20 && realTime->Minutes <=39)
        {
            myAlarm.AlarmTime.Hours = realTime->Hours;
            myAlarm.AlarmTime.Minutes = 40;
            myAlarm.AlarmTime.Seconds = 0;
        }else
        {
            myAlarm.AlarmTime.Hours = realTime->Hours+1;
            myAlarm.AlarmTime.Minutes = 0;
            myAlarm.AlarmTime.Seconds = 0;
        }
    }
    if (HAL_RTC_SetAlarm_IT(hrtc, &myAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        Error_Handler();
    }
}

void sleepEnd(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime, UART_HandleTypeDef *ESPhuart, uint8_t *selectedElemIndex, uint8_t *scrollCount,
    uint8_t *buttonIsPressed, uint32_t *lastInteractionTime, UART_HandleTypeDef *TestHuart) {
    SystemClock_Config();
    *selectedElemIndex = 0;
    *scrollCount = 0;
    *buttonIsPressed = 0;
    HAL_ResumeTick();
    char mess[10] = "Wakeup";
    HAL_UART_Transmit(TestHuart, (uint8_t *) mess,15,0xFFFF);
    //HAL_GPIO_WritePin(GPIOA, WakesUp_Pin, GPIO_PIN_RESET);
    //HAL_Delay(100);
    //HAL_GPIO_WritePin(GPIOA, WakesUp_Pin, GPIO_PIN_SET);
    DelAllExceptions();
    LoadingScreen();
    HAL_RTC_GetTime(hrtc, realTime, RTC_FORMAT_BIN);
    WakeupESP(hrtc, realTime, ESPhuart);

    sprintf(mess, "%u:%u", realTime->Hours, realTime->Minutes);
    HAL_UART_Transmit(TestHuart, (uint8_t *) mess, 10, 0xFFFF);

    RefreshLCD();
    *lastInteractionTime = HAL_GetTick();
    DataRequest();
}

void sleepBegin(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime, UART_HandleTypeDef *ESPhuart, uint8_t *selectedElemIndex, uint8_t *scrollCount,
    uint8_t *buttonIsPressed, uint32_t *lastInteractionTime, UART_HandleTypeDef *TestHuart) {
    setAlarm(hrtc, realTime);
    SleepESP(ESPhuart);
    char mess[15] = "Goes to sleep";
    HAL_UART_Transmit(TestHuart, (uint8_t *) mess,15,0xFFFF);
    //HAL_GPIO_WritePin(GPIOA, GoesToSleep_Pin, GPIO_PIN_RESET);
    //HAL_Delay(100);
    //HAL_GPIO_WritePin(GPIOA, GoesToSleep_Pin, GPIO_PIN_SET);
    lcd_clear();
    setBacklightOff();	
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    sleepEnd(hrtc, realTime, ESPhuart, selectedElemIndex, scrollCount, buttonIsPressed, lastInteractionTime, TestHuart);
}