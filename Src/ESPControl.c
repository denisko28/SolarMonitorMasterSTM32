#include "ESPControl.h"
#include "main.h"

void TxESP(UART_HandleTypeDef *huart, char *command, char *pData, uint16_t Size)
{
	uint8_t cmdSize = strlen(command); 

	for(uint8_t i = 0; i<2; i++) {
		HAL_UART_Transmit(huart, (uint8_t*)command, (uint16_t)cmdSize, 0xFFFF);
		RxESP(huart, 1, pData, Size);
		if(strcmp(pData, "No response") == 0)
			strcpy(pData, "0");
		else
			break;
	}
}

void RxESP(UART_HandleTypeDef *huart, uint8_t resetOnExept, char *pData, uint16_t Size)
{
	char *tempBuff = calloc(500, sizeof(char));
	HAL_UART_Receive_IT(huart,(uint8_t*) tempBuff, 500);
	HAL_Delay(1);
	HAL_UART_AbortReceive_IT(huart);
	free(tempBuff);

	//while(huart->RxState != HAL_UART_STATE_READY){}
	HAL_UART_Receive_IT(huart, (uint8_t *) pData, Size);

	const uint32_t beginTime = HAL_GetTick();
	uint8_t success = 1; 
	while(strstr(pData, "^^") == NULL){
		HAL_Delay(100);
		if(HAL_GetTick() - beginTime > 16000){
			AddException('6');
			strcpy(pData, "No response");
			HAL_UART_AbortReceive_IT(huart);
			if(resetOnExept == 1)
				ResetESP(huart);
			success = 0;
			return;
		}
	}
	if(success == 1) {
		HAL_UART_AbortReceive_IT(huart);
		uint8_t len = strlen(pData);
		*(pData + len-1) = '\0';
		*(pData + len-2) = '\0';
		DelException('6');
	}
}

void WakeupESP(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime, UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(ESP_Wakeup_GPIO_Port, ESP_Wakeup_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(ESP_Wakeup_GPIO_Port, ESP_Wakeup_Pin, GPIO_PIN_SET);
	
	char responseRx[15] = {0};
	RxESP(huart, 1, responseRx, 15);
	if(strcmp(responseRx, "No response") != 0) {
		RxESP(huart, 1, responseRx, 15);
		if(strcmp(responseRx, "Error 1") != 0) {
			DelException('1');
			InitRealTime(huart, hrtc, realTime);
		}else
			AddException('1');
	}
}

void SleepESP(UART_HandleTypeDef *huart)
{
	char commandToSend[2] = "4";
	char responseRx[10] = {0};
	TxESP(huart, commandToSend, responseRx, 10);
	if(strcmp(responseRx, "OK") == 0)
		DelException('5');
	else
		AddException('5');
}

void ResetESP(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(ESP_Reset_GPIO_Port, ESP_Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(ESP_Reset_GPIO_Port, ESP_Reset_Pin, GPIO_PIN_SET);
	// Clearing uart from trash
	// char *tempBuff = calloc(500, sizeof(char));
	// HAL_UART_Receive_IT(huart,(uint8_t*) tempBuff, 500);
	// HAL_Delay(500);
	// HAL_UART_AbortReceive_IT(huart);
	// free(tempBuff);
	// Internet connection check
	char responseRx[15] = {0};
	RxESP(huart, 0, responseRx, 15);
	if(strcmp(responseRx, "Error 1") != 0) {
		DelException('1');
	}else
		AddException('1');
}

void InitRealTime(UART_HandleTypeDef *huart, RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *realTime)
{
	char commandToSend[2] = "1";
	char responseRx[15] = {0};
	TxESP(huart, commandToSend, responseRx, 15);
	if(strcmp(responseRx, "Error 3") != 0 && strcmp(responseRx, "0") != 0) {
		DelException('2');
		realTime->Hours = atoi(strtok(responseRx,":"));
		realTime->Minutes = atoi(strtok(NULL,":"));
		realTime->Seconds = 0;
		if (HAL_RTC_SetTime(hrtc, realTime, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}
	}else
		AddException('2');
}

// void GetLastEnergy(UART_HandleTypeDef *huart, double *lastEnergy)
// {
// 	char commandToSend[2] = "3";
// 	char* responseRx = TxESP(huart, commandToSend);	
// 	if(strcmp(responseRx, "Error 3") != 0 && strcmp(responseRx, "0") != 0) {
// 		*lastEnergy = atof(responseRx);
// 	}else
// 		AddException('3');

// 	free(responseRx);
// }
	