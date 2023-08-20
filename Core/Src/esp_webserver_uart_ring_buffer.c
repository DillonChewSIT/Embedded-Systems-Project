/*
 * esp_webserver_uart_ring_buffer.c
 *
 */
#include "esp_webserver_uart_ring_buffer_pixy.h"

void ClearAllData(uint8_t nUART){
	uartState[nUART].dataSize = 0;
	uartState[nUART].rxCircArrayPtr = 0;
	uartState[nUART].txCircArrayPtr = 0;
	uartState[nUART].rxbuffDex = 0;
	uartState[nUART].remainSize = MaxBufferSize;
	for (uint8_t i = 0; i < CircSize; ++i)
	{
		uartState[nUART].circArray[i][Sz] = 0;
	}
}

void StartReceiver(uint8_t nUART)
{
	HAL_UARTEx_ReceiveToIdle_DMA(uartState[nUART].huartAdd, uartState[nUART].rxbufferAdd+uartState[nUART].rxbuffDex, uartState[nUART].remainSize);
	uartState[nUART].rxdmaAdd->Instance->CCR &= ~(DMA_IT_TC | DMA_IT_HT);
	uartState[nUART].rxState = Receiving;
}

void Transmit(uint8_t nUART, uint8_t* buffer, uint16_t index, uint16_t size, uint8_t updateCirc){
	uint8_t txHandle = uartState[nUART].txDex;
	HAL_UART_Transmit_DMA(uartState[nUART].huartAdd, buffer + index, size);
	uartState[nUART].txdmaAdd->Instance->CCR &= ~(DMA_IT_HT);
	uartState[nUART].txState = Transmitting;
	if (updateCirc == TRUE){
		uartState[txHandle].circArray[uartState[txHandle].txCircArrayPtr][Sz] = 0;
		if ((uartState[txHandle].txCircArrayPtr) == (CircSize-1)){
			uartState[txHandle].txCircArrayPtr = 0;
		}
		else{
			uartState[txHandle].txCircArrayPtr+=1;
		}
	}
	if (uartState[nUART].rxState == Wait){
		uartState[nUART].rxState = GotData;
	}
}

void TransmitESP(uint8_t* buffer, uint16_t size, uint8_t updateCirc)
{
	//printf("%s",buffer);
	HAL_UART_Transmit_DMA(uartState[UART1].huartAdd, buffer, size);
	uartState[UART1].txdmaAdd->Instance->CCR &= ~(DMA_IT_HT);
	uartState[UART1].txState = Transmitting;
	if (updateCirc == TRUE){
		uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Sz] = 0;
		if ((uartState[UART1].txCircArrayPtr) == (CircSize-1)){
			uartState[UART1].txCircArrayPtr = 0;
		}
		else {
			uartState[UART1].txCircArrayPtr+=1;
		}
	}
	if (uartState[UART1].rxState == Wait){
		uartState[UART1].rxState = GotData;
	}
}

void TransmitMOT(uint8_t* buffer, uint16_t size, uint8_t updateCirc)
{
	//printf("%s",buffer);
	HAL_UART_Transmit_DMA(uartState[UART2].huartAdd, buffer, size);
	uartState[UART2].txdmaAdd->Instance->CCR &= ~(DMA_IT_HT);
	uartState[UART2].txState = Transmitting;
	if (updateCirc == TRUE){
		uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Sz] = 0;
		if ((uartState[UART2].txCircArrayPtr) == (CircSize-1)){
			uartState[UART2].txCircArrayPtr = 0;
		}
		else {
			uartState[UART2].txCircArrayPtr+=1;
		}
	}
	if (uartState[UART2].rxState == Wait){
		uartState[UART2].rxState = GotData;
	}
}
void RecordIntoCirc(uint8_t nUART, uint8_t delimiter)
{
	uint8_t iDex = uartState[nUART].rxCircArrayPtr;
	if ((uartState[nUART].circArray[iDex][Sz]) == 0){
		uartState[nUART].circArray[iDex][Index] = uartState[nUART].rxbuffDex;
		uartState[nUART].circArray[iDex][Sz] = uartState[nUART].dataSize;

		if (iDex == CircSize-1){uartState[nUART].rxCircArrayPtr = 0;}
		else {uartState[nUART].rxCircArrayPtr += 1;}
		uartState[nUART].rxbuffDex += uartState[nUART].dataSize;
		if (delimiter)
		{
//			if(uartState[nUART].rxbufferAdd[uartState[nUART].rxbuffDex] == 0)
//			{
//				uartState[nUART].circArray[iDex][Index]+=1;
//				uartState[nUART].circArray[iDex][Sz]+=1;
//				uartState[nUART].dataSize+=1;
//			}
//			else {
				uartState[nUART].rxbufferAdd[uartState[nUART].rxbuffDex] = 0;
				uartState[nUART].rxbuffDex+=1;
				uartState[nUART].dataSize +=1;
				uartState[nUART].circArray[iDex][Index]+=1;
				uartState[nUART].circArray[iDex][Sz]+=1;
			//}
		}
		if (uartState[nUART].rxbuffDex > BuffRewindLimit){
			uartState[nUART].rxbuffDex = 0;
			uartState[nUART].remainSize = MaxBufferSize;
		}
		else {
			uartState[nUART].remainSize -= uartState[nUART].dataSize;
		}
		uartState[nUART].dataSize = 0;
		uartState[nUART].rxState = Idle;
	}
	else {uartState[nUART].rxState = Wait;}
}

void PrintError(uint8_t errorCode, uint8_t nUART)
{
	switch(errorCode){
	case 0b00000001: printf("Parity Error\r\n");break;
	case 0b00000010: printf("Noise Error\r\n");break;
	case 0b00000100: printf("Frame Error\r\n");break;
	case 0b00001000: printf("Overrun Error\r\n");break;
	case 0b00010000: printf("DMA Error\r\n");break;
	case 0b00100000: printf("Receiver Timeout Error\r\n");break;
	case 0b00000110: printf("Noise and Frame Error\r\n");break;
	case 0b00001010: printf("Overrun and Noise Error\r\n");break;
	case 0b00001100: printf("Overrun and Frame Error\r\n");break;
	case 0b00001110: printf("Overrun, Frame and Noise Error\r\n");break;
	default:		 printf("Unknown Error\r\n");break;
 	}
}
