/*
 * esp_webserver_uart_ring_buffer_pixy_callback.c
 *
 */
#include "esp_webserver_uart_ring_buffer_pixy.h"

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(stateMachine == TPIX)
	{
		pixyState.spiState = GotData;
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(stateMachine == AUTO_LISTEN)
	{
		HAL_SPI_Receive_IT(pixyState.hspi, pixyState.rxbufferAdd, pixyState.dataSize);
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(stateMachine == AUTO_LISTEN)
	{
		pixyState.spiState = GotData;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t nUART;
	if (huart->Instance == huart1.Instance)
	{
		nUART = UART1;
	}
	else if (huart->Instance == huart2.Instance)
	{
		nUART = UART2;
	}
	uartState[nUART].txState = Idle;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint8_t nUART;
	if (huart->Instance == huart1.Instance)
	{
		nUART = UART1;
		if (stateMachine == AUTO_SERVER)
		{
			autoServerSize = Size;
		}
		if (stateMachine == AUTO_LISTEN)
		{
			webRequest.request = 1;
			autoServerSize = Size;
		}
		uartState[nUART].rxState = GotData;
	}
	else if (huart->Instance == huart2.Instance)
	{
		nUART = UART2;
		uartState[nUART].rxState = TestAutoCheck;
	}
	uartState[nUART].dataSize = Size;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
	{
		uartState[UART1].InError = TRUE;
		uartState[UART1].rxState = Error;
	}
	else if (huart->Instance == huart2.Instance)
	{
		uartState[UART2].InError = TRUE;
		uartState[UART2].rxState = Error;
	}
}
