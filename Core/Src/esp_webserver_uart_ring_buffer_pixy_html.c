/*
 * esp_webserver_uart_ring_buffer_pixy_html.c
 *
 */
#include "esp_webserver_uart_ring_buffer_pixy.h"

void Auto_Init(uint8_t IPbuffer[])
{
	uint8_t buffer[2501] = "";
    switch (stateFlag.rxFlag) {
	case 0:
		while (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)AT, strlen((const char*)AT)) != HAL_OK) {}
		stateFlag.rxFlag = 1;
		break;

	case 1:
		memcpy(buffer, uartState[UART1].rxbufferAdd + uartState[UART1].rxbuffDex, autoServerSize);
		if (strstr((char*)buffer, AT_OK) != NULL){
			stateFlag.rxFlag = 2;
		}
		break;

	case 2:
		while (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CWMODE, strlen((const char*)CWMODE)) != HAL_OK) {}
		stateFlag.rxFlag = 3;
		break;

	case 3:
		memcpy(buffer, uartState[UART1].rxbufferAdd + uartState[UART1].rxbuffDex, autoServerSize);
		if (strstr((char*)buffer, AT_OK) != NULL){
			stateFlag.rxFlag = 4;
		}
		break;

	case 4:
		while (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CWJAP, strlen((const char*)CWJAP)) != HAL_OK) {}
		for(int x = 0; x < 1000; x++){};
		stateFlag.rxFlag = 5;
		break;

	case 5:
		memcpy(buffer, uartState[UART1].rxbufferAdd + uartState[UART1].rxbuffDex, autoServerSize);
		if (strstr((char*)buffer, AT_OK) != NULL) {
		 stateFlag.rxFlag = 6;
		}
		break;

	case 6:
		while (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CIFSR, strlen((const char*)CIFSR)) != HAL_OK) {}
		stateFlag.rxFlag = 7;
		break;

	case 7:
		memcpy(buffer, uartState[UART1].rxbufferAdd + uartState[UART1].rxbuffDex, autoServerSize);
		if (strstr((char*)buffer, "+CIFSR:") != NULL) {
			memcpy(IPbuffer, uartState[UART1].rxbufferAdd + uartState[UART1].rxbuffDex, autoServerSize);
			printLCD(IPbuffer);
			stateFlag.rxFlag = 8;
		}
		if (strstr((char*)buffer,"busy")!= NULL)
		{
			stateFlag.rxFlag = 4;
		}
		break;

	case 8:
		while (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CIPMUX, strlen((const char*)CIPMUX)) != HAL_OK) {}
		stateFlag.rxFlag = 9;
		break;

	case 9:
		memcpy(buffer, uartState[UART1].rxbufferAdd + uartState[UART1].rxbuffDex, autoServerSize);
		if (strstr((char*)buffer, AT_OK) != NULL) {
			stateFlag.rxFlag = 10;
		}
		break;

	case 10:
		while (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CIPSERVER, strlen((const char*)CIPSERVER)) != HAL_OK) {}
		stateFlag.rxFlag = 11;
		break;

	case 11:
		memcpy(buffer, uartState[UART1].rxbufferAdd + uartState[UART1].rxbuffDex, autoServerSize);
		if (strstr((char*)buffer, AT_OK) != NULL) {
			stateFlag.rxFlag = 12;
		}
		break;

	case 12:
		while (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)CIPSTATUS, strlen((const char*)CIPSTATUS)) != HAL_OK) {}
		stateFlag.rxFlag = 13;
		break;

	case 13:
		memcpy(buffer, uartState[UART1].rxbufferAdd + uartState[UART1].rxbuffDex, autoServerSize);
		if (strstr((char*)buffer, AT_OK) != NULL) {
			stateFlag.rxFlag = 14;
		}
//		else {
//			stateFlag.rxFlag = 0;
//		}
		break;

	case 14:
		stateFlag.rxFlag =15;
		break;
    }
}

void RequestHandle(void)
{
	uint8_t circPtr = uartState[UART1].txCircArrayPtr;
	uint16_t buffIdx = uartState[UART1].circArray[circPtr][Index];
	char* buf = (char*)&(uartState[UART1].rxbufferAdd[buffIdx]);
	char* tempIPD;
	char* req;

	if(((req = strstr(buf,"GET /"))!= NULL) && ((tempIPD = strstr(buf, "IPD,"))!=NULL))
	{

		webRequest.get = *(req+5);
		webRequest.ipd = *(tempIPD+4);
		switch(webRequest.get)
		{
		case ' ':
			webRequest.process = WebServer;
			buffer[11] = (uint8_t)webRequest.ipd;
			TransmitESP(&buffer[0], strlen((const char*)&buffer[0]),TRUE);
			break;

		case '1':
			webRequest.process = ToggleBlueLED;
			buffer[51] = (uint8_t)webRequest.ipd;
			TransmitESP(&buffer[40],strlen((const char*)&buffer[40]),TRUE);
			break;

		case '2':
			webRequest.process = ToggleRedLED;
			buffer[51] = (uint8_t)webRequest.ipd;
			TransmitESP(&buffer[40],strlen((const char*)&buffer[40]),TRUE);
			break;

		case '3':
		case '4':
			webRequest.process = GetGreenLED;
			buffer[51] = (uint8_t)webRequest.ipd;
			TransmitESP(&buffer[40],strlen((const char*)&buffer[40]),TRUE);
			break;

		case 'f':
			webRequest.process = Spam;
			buffer[33] = (uint8_t)webRequest.ipd;
			TransmitESP(&buffer[21], strlen((const char*)&buffer[21]), TRUE);
			break;

		}
	}
	else if ((req = strstr(buf, "OK\r\n>"))!=NULL){ //SEND BITS
		switch(webRequest.process)
		{
		case WebServer:
			TransmitESP(&buffer[100], strlen((const char*)&buffer[100]), TRUE); //AT+CIPSEND=IPD,1850
			break;

		case ToggleBlueLED:
			ToggleLED(Blue);
			break;

		case ToggleRedLED:
			ToggleLED(Red);
			break;

		case GetGreenLED:
			if(pixyState.spiTurnState==MOVE_FORWARD){TransmitESP(&buffer[80],strlen((const char*)&buffer[80]), TRUE);}
			else if(pixyState.spiTurnState==MOVE_LEFT){TransmitESP(&buffer[85],strlen((const char*)&buffer[85]), TRUE);}
			else if(pixyState.spiTurnState==MOVE_RIGHT){TransmitESP(&buffer[90],strlen((const char*)&buffer[90]), TRUE);}
			break;
		}
	}
	else if ((req = strstr(buf, "SEND OK"))!=NULL){ //CIPCLOSE
		buffer[33] = (uint8_t)webRequest.ipd;
		TransmitESP(&buffer[21], strlen((const char*)&buffer[21]), TRUE);
	}
	else
	{
		uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Sz] = 0;
		if ((uartState[UART1].txCircArrayPtr) == (CircSize-1)){
			uartState[UART1].txCircArrayPtr = 0;
		}
		else {
			uartState[UART1].txCircArrayPtr+=1;
		}
		if(uartState[UART1].rxState == Wait){
			uartState[UART1].rxState = GotData;
		}
	}
}

void ToggleLED(uint8_t led)
{
	switch(led) //SET is off RESET is on
	{
	case Blue:
		if(HAL_GPIO_ReadPin(Blue_LD_GPIO_Port, Blue_LD_Pin) == GPIO_PIN_SET){
			TransmitESP(&buffer[65],strlen((const char*)&buffer[65]),TRUE);
		}
		else {
			TransmitESP(&buffer[60],strlen((const char*)&buffer[60]),TRUE);
		}
		HAL_GPIO_TogglePin(Blue_LD_GPIO_Port, Blue_LD_Pin);
		break;

	case Red:
		if(HAL_GPIO_ReadPin(Red_LD_GPIO_Port, Red_LD_Pin) == GPIO_PIN_SET){
			TransmitESP(&buffer[75],strlen((const char*)&buffer[75]),TRUE);
		}
		else {
			TransmitESP(&buffer[70],strlen((const char*)&buffer[70]),TRUE);
		}
		HAL_GPIO_TogglePin(Red_LD_GPIO_Port, Red_LD_Pin);
		break;
	}
}
