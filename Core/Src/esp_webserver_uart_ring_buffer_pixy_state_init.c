/*
 * esp_webserver_uart_ring_buffer_pixy_state_init.c
 *
 */
#include "esp_webserver_uart_ring_buffer_pixy.h"

void Uart_Init(void)
{
	uartState[UART1].huartAdd 		= &huart1;
	uartState[UART1].txbufferAdd 	= buffArray[UART2];
	uartState[UART1].txDex 			= UART2;
	uartState[UART1].rxdmaAdd 		= &hdma_usart1_rx;
	uartState[UART1].txdmaAdd 		= &hdma_usart1_tx;

	uartState[UART2].huartAdd 		= &huart2;
	uartState[UART2].txbufferAdd 	= buffArray[UART1];
	uartState[UART2].txDex 			= UART1;
	uartState[UART2].rxdmaAdd 		= &hdma_usart2_rx;
	uartState[UART2].txdmaAdd 		= &hdma_usart2_tx;

	for (int i = 0; i <2; ++i)
	{
		uartState[i].txState 		= Idle;
		uartState[i].rxState 		= Idle;
		uartState[i].InError 		= FALSE;
		uartState[i].rxbufferAdd 	= buffArray[i];
		uartState[i].dataSize 		= 0;
		uartState[i].remainSize 	= MaxBufferSize;
		uartState[i].rxDex 			= i;
		uartState[i].rxCircArrayPtr = 0;
		uartState[i].txCircArrayPtr = 0;
	}
}

void Flag_Init(void)
{
	stateFlag.autoinitFlag 			= 0;
	stateFlag.rxFlag 				= 0;
	webRequest.request 				= 0;
	webRequest.requestAction 		= 0;
	webRequest.process 				= 0;
	webRequest.action 				= 0;
}

void Pixy_Init(void)
{
	pixyState.hspi					= &hspi2;
	pixyState.rxbufferAdd			= spiBuffArray;
	pixyState.spiState 				= Idle;
	pixyState.spiChecksum 			= 3;
	pixyState.dataSize				= MaxSpiSize;

	pixyState.getVersion	 		= pixyGetVersion;
	pixyState.setLamp	 			= pixySetLamp;
	pixyState.getAllFeatures	 	= pixyGetAllFeatures;
	pixyState.getMainFeatures	 	= pixyGetMainFeatures;

	pixyState.IntersectionDecision  = NONE;
	pixyState.vectorX				= 0;
	pixyState.vectorY               = 0;
	pixyState.branchCount			= 0;
	pixyState.barcodeId				= BARCODE_NULL;
	pixyState.barcodeY              = 0;
	pixyState.spiTurnState			= MOVE_FORWARD;
}

void Motor_Init(void)
{
	motorState.leftSpd              = 0;
	motorState.rightSpd             = 0;
}

void String_Init(uint8_t* buffer)
{
	strcpy((char*)&buffer[0],(char*)CIPSEND); 		//AT+CIPSEND= ,SizeofHTML
	strcpy((char*)&buffer[21],(char*)CIPCLOSE); 	//AT+CIPCLOSE=
	strcpy((char*)&buffer[40],(char*)CIPSENDLED);	//AT+CIPSEND= ,2
	strcpy((char*)&buffer[60],(char*)BlueLEDOFF);	//Blue LED OFF
	strcpy((char*)&buffer[65],(char*)BlueLEDON);	//Blue LED ON
	strcpy((char*)&buffer[70],(char*)RedLEDOFF);	//Red LED OFF
	strcpy((char*)&buffer[75],(char*)RedLEDON);		//Red LED ON
	strcpy((char*)&buffer[80],(char*)PixyTurnFEED);	//Pixy move forward
	strcpy((char*)&buffer[85],(char*)PixyTurnLEFT);	//Pixy turn left
	strcpy((char*)&buffer[90],(char*)PixyTurnRIGHT);//Pixy turn right
	strcpy((char*)&buffer[100],(char*)html);		//Webserver HTML
	strcpy((char*)&tpixBuffer[0],(char*)SpiPASSED);	//SPI Checksum Passed
	strcpy((char*)&tpixBuffer[13],(char*)SpiFAILED);//SPI Checksum Failed
}
