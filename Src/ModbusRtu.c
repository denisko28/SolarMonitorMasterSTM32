#include "ModbusRtu.h"
#include "main.h"

uint8_t u8id = 0;
uint8_t u8BufferSize = 6;
uint16_t u16InCnt, u16OutCnt, u16errCnt = 0;
uint16_t au16regs[16];
uint8_t au8Buffer[64];
char stri[12] = {0};


int16_t makeWord(uint8_t h, uint8_t l);


enum MESSAGE
{
    ID = 0, //!< ID field
    FUNC, //!< Function code position
    ADD_HI, //!< Address high byte
    ADD_LO, //!< Address low byte
    NB_HI, //!< Number of coils or registers high byte
    NB_LO, //!< Number of coils or registers low byte
    BYTE_CNT  //!< byte counter
};

void StartMB()
{
	// set RS485 transceiver to receive mode
	HAL_GPIO_WritePin(RS485_RTS_GPIO_Port, RS485_RTS_Pin, GPIO_PIN_RESET);
}

char* MBGetQuery(UART_HandleTypeDef *huart,  uint16_t regAdress, uint16_t regNum)
{
	au8Buffer[ID] = 1;
	au8Buffer[ FUNC ] = 3;
	au8Buffer[ ADD_HI ] = highByte(regAdress);
	au8Buffer[ ADD_LO ] = lowByte(regAdress);
	au8Buffer[ NB_HI ] = highByte(regNum);
	au8Buffer[ NB_LO ] = lowByte(regNum);
	sendTxBuffer(huart, regNum);

	return getRxBuffer(huart, regNum);
}

char* getRxBuffer(UART_HandleTypeDef *huart, uint8_t regNum)
{
	memset(stri, 0x00, 12);
	
	const uint32_t beginTime = HAL_GetTick();
	while(huart->RxXferCount != 0){
		HAL_Delay(100);
		if(HAL_GetTick() - beginTime > 1000){
			AddException('7');
			strcpy(stri, "0.1");
			HAL_UART_AbortReceive(huart);
			return stri;
		}
	}

	DelException('7');
	u8BufferSize = 0;
	get_FC3(regNum);
	
	
	if(regNum == 2)
		sprintf(stri, "%d", (au16regs[0]*65536+au16regs[1]));
	else
		sprintf(stri, "%d", au16regs[0]);

	if(stri[0] == '0' && strlen(stri)>2)
	{
		memmove(stri, stri+1, strlen(stri));
	}
	uint8_t striLen = strlen(stri);		
	if(striLen > 1)
	{
		stri[striLen] = stri[striLen-1];
		stri[striLen-1] = '.';
	}
	
	return stri;
}

void get_FC3(uint8_t regNum)
{
	uint8_t u8byte;
	u8byte = 3;

	au16regs[0] = ((uint16_t)au8Buffer[ u8byte ] << 8) | au8Buffer[ u8byte+1 ];

	if(regNum == 2)
	{
		au16regs[1] = ((uint16_t)au8Buffer[ u8byte+2 ] << 8) | au8Buffer[ u8byte+3 ];
	}
}

void sendTxBuffer(UART_HandleTypeDef *huart, uint8_t regNum)
{
	u8BufferSize = 6;
	uint16_t u16crc = calcCRC( u8BufferSize );
	au8Buffer[ u8BufferSize ] = u16crc >> 8;
	u8BufferSize++;
	au8Buffer[ u8BufferSize ] = u16crc & 0x00ff;
	u8BufferSize++;

	// set RS485 transceiver to transmit mode
	HAL_GPIO_WritePin(RS485_RTS_GPIO_Port, RS485_RTS_Pin, GPIO_PIN_SET);
	
	// transfer buffer to UART
	HAL_UART_Transmit(huart, au8Buffer, u8BufferSize, 0xFFFF);

	// must wait transmission end before changing pin state
	// soft serial does not need it since it is blocking
	// ...but the implementation in SoftwareSerial does nothing
	// anyway, so no harm in calling it.
	
	// return RS485 transceiver to receive mode
	while(huart->gState != HAL_UART_STATE_READY){}
	
	HAL_GPIO_WritePin(RS485_RTS_GPIO_Port, RS485_RTS_Pin, GPIO_PIN_RESET);
	
	//Turn on UART iteraption
	if(regNum == 2)
	{
		HAL_UART_Receive_IT(huart,(uint8_t*)  au8Buffer, 9);
	}else {
		HAL_UART_Receive_IT(huart,(uint8_t*)  au8Buffer, 7);
	}
}

uint16_t calcCRC(uint8_t u8length)
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++)
    {
        temp = temp ^ au8Buffer[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}

