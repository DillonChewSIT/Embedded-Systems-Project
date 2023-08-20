/*
 * esp_webserver_uart_ring_buffer_pixy.c
 *
 */
#include "esp_webserver_uart_ring_buffer_pixy.h"

void prepend(int huart)
{
	if(uartState[huart].dataSize > MaxBufferSize)
	{
		return;
	}
	if (huart == UART2)
	{
		uint8_t buffer[MaxBufferSize] = "UART2_Rxed: ";
		memcpy(buffer+11, uartState[UART2].rxbufferAdd, strlen((const char*)uartState[UART2].rxbufferAdd));
		memcpy(uartState[UART2].rxbufferAdd,buffer, strlen((const char*)buffer));
		printf("Prepended to UART2: %s\r\n", uartState[UART2].rxbufferAdd);
	}
	else if (huart == UART1)
	{
		uint8_t buffer[MaxBufferSize] = "UART1_Rxed: ";
		memcpy(buffer+11, uartState[UART1].rxbufferAdd, strlen((const char*)uartState[UART1].rxbufferAdd));
		memcpy(uartState[UART1].rxbufferAdd,buffer, strlen((const char*)buffer));
		printf("Prepended to UART1: %s\r\n", uartState[UART1].rxbufferAdd);
	}
}

void WordSearch(uint8_t huart)
{
	if (uartState[huart].dataSize!=6)
	{
		if (stateMachine == AUTO_SERVER || stateMachine == AUTO_LISTEN){
			uartState[huart].rxState = Idle;
			return;
		}
		if(stateMachine == TEST || stateMachine == TPIX || stateMachine == TMOT){
			uartState[huart].rxState = GotData;
			return;
		}
	}

	const char* autoFind = "AUTO\r\n";
	const char* testFind = "TEST\r\n";
	const char* tpixFind = "TPIX\r\n";
	const char* tmotFind = "TMOT\r\n";
	uint8_t buffer[6];
	memcpy(buffer, uartState[huart].rxbufferAdd + uartState[huart].rxbuffDex, uartState[huart].dataSize);

	if (strcmp((char*)buffer, autoFind) == 0)   //if uart2 rxbuffer contains AUTO, do something
	{
		HAL_UART_AbortTransmit(&huart2);
		if(stateMachine == AUTO_SERVER)
		{
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Already on AUTO Mode!\r\n", 23);
		}
		else
		{
			stateFlag.rxFlag = 0;
			stateMachine = AUTO_SERVER;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Switching to AUTO Mode!\r\n", 25);
//			ClearAllData(UART1);
//			ClearAllData(UART2);
//			memset(uartState[UART1].rxbufferAdd, 0, MaxBufferSize);
//			memset(uartState[UART2].rxbufferAdd, 0, MaxBufferSize);
		}
		uartState[huart].rxState = Idle;
	}
	else if (strcmp((char*)buffer, tpixFind) == 0)
	{
		if(stateMachine == TPIX)
		{
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Already on TPIX Mode!\r\n", 23);
		}
		else {
			stateMachine = TPIX;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Switching to TPIX Mode!\r\n", 25);
//			ClearAllData(UART1);
//			ClearAllData(UART2);
//			memset(uartState[UART1].rxbufferAdd, 0, MaxBufferSize);
//			memset(uartState[UART2].rxbufferAdd, 0, MaxBufferSize);
			pixyState.spiState = Idle;
		}
		uartState[huart].rxState = Idle;
	}
	else if (strcmp((char*)buffer, testFind) == 0)   //if uart2 rxbuffer contains TEST, do something
	{
		if (stateMachine == TEST)
		{
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Already on TEST Mode!\r\n", 23);
		}
		else
		{
			stateFlag.autoinitFlag = 0;
			stateFlag.rxFlag = 0;
			stateMachine = TEST;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Switching to TEST Mode!\r\n", 25);
//			ClearAllData(UART1);
//			ClearAllData(UART2);
//			memset(uartState[UART1].rxbufferAdd, 0, MaxBufferSize);
//			memset(uartState[UART2].rxbufferAdd, 0, MaxBufferSize);
		}
		uartState[huart].rxState = Idle;
	}
	else if (strcmp((char*)buffer, tmotFind) == 0)
	{
		if (stateMachine == TMOT)
		{
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Already on TMOT Mode!\r\n", 23);
		}
		else
		{
			stateFlag.autoinitFlag = 0;
			stateFlag.rxFlag = 0;
			stateMachine = TMOT;
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)"Switching to TMOT Mode!\r\n", 25);
//			ClearAllData(UART1);
//			ClearAllData(UART2);
//			memset(uartState[UART1].rxbufferAdd, 0, MaxBufferSize);
//			memset(uartState[UART2].rxbufferAdd, 0, MaxBufferSize);
		}
		uartState[huart].rxState = Idle;
	}
	else {
		if(stateMachine == AUTO_SERVER || stateMachine == AUTO_LISTEN)
		{
			uartState[huart].rxState = Idle;
			return;
		}
		if (stateMachine == TEST || stateMachine == TPIX || stateMachine == TMOT)
		{
			uartState[huart].rxState = GotData;
			return;
		}
	}
	return;
}

uint8_t AFC1Search(uint8_t* buffer, uint16_t* pose, uint16_t* size)
{
	uint16_t Start = 0;
	uint16_t End = MaxSpiSize;
	uint16_t bBegin;
	uint16_t bEnd;
	uint16_t checksum;
	uint16_t calc_checksum = 0;

	/* Look for Header AF C1 */
	while (Start < (End-1) && (!((buffer[Start]==0xAF)&&(buffer[Start+1]==0xC1))))
	{
		Start++;
	}

	/* Read Data */
	if (Start < (End-1))
	{
//		printf("AFC1 Found\r\n");

		/* set the buffer starting index to point at AFC1 */
		*pose+=Start;

//		/* Print Data to Console */
//		printf("SPI2 Rx:");
//		for(uint16_t i = Start; i < End; i++)
//		{
//			printf(" %x", pixyState.rxbufferAdd[i]);
//		}
//		printf("\r\n");

		/* 2 bytes header, 1 byte packet type, 1 byte data length, 2 bytes checksum */
		*size =  buffer[Start+3] + 6; // 4th byte is data length, only counts length after checksum

		/* 5th & 6th byte is Checksum in Little Endian, convert to Big Endian */
		checksum = (uint16_t)(buffer[Start+4]) | (uint16_t)((buffer[Start+5]) << 8);

		/* Offset start by Size to find End */
		End = Start + (*size);

		/* Offset start by 6 bytes for first feature type */
		Start += 6;

		/* Mark where Feature Data Starts and Ends */
		bBegin = Start;
		bEnd = End;

		while (Start < End)
		{
			/* summing up payload 16 bytes to check if it corresponds to payload bytes */
			calc_checksum+=(uint16_t)buffer[Start];
			Start++;
		}

		/* Validate Checksum */
		if(checksum == calc_checksum)
		{
//			printf("Checksum Passed\r\n");

			if(stateMachine == AUTO_LISTEN)
			{
				/* Search for Feature */
				findFeature(buffer, bBegin, bEnd);
			}

			return 1;
		}
		else {
//			printf("Checksum Failed\r\n");
			return 2;
		}
	}
	return 0;
}

void findFeature(uint8_t* buffer, uint16_t start, uint16_t end)
{
	uint8_t featureSize;
	while(start < end)
	{
		featureSize = buffer[start+1];
		switch(buffer[start])
		{
		case 0x01:	// Vector
			pixyState.vectorX = buffer[start+4]; // Vector Destination x-coord
			pixyState.vectorY = buffer[start+5];
			pixyState.branchCount = 0; // Reset Branch Count
			pixyState.barcodeId = BARCODE_NULL; // Set Barcode ID to NULL
			break;
		case 0x02:	// Intersection
			pixyState.branchCount = buffer[start+4]; // Set Branch Count
			break;
		case 0x04:	// Barcode
			pixyState.barcodeY = buffer[start+3];
			pixyState.barcodeId = buffer[start+5]; // Set Barcode ID
			break;
		}
		/* Increment Buffer Pointer to next Feature */
		start += (2 + featureSize);
	}
//	printf("Vector Destination X-Coord: %d\r\n", pixyState.vectorX);
//	printf("Intersection Branch Count: %d\r\n", pixyState.branchCount);
//	printf("Barcode Number: %d\r\n", pixyState.barcodeId);
	if(pixyState.barcodeId == BARCODE_STOP)
	{
		pixyState.spiTurnState = MOVE_STOP;
	}
	else if(pixyState.barcodeId == BARCODE_NULL)
	{
		if(pixyState.vectorX < 25)
		{
			pixyState.spiTurnState = MOVE_LEFT;
		}
		else if(pixyState.vectorX > 53)
		{
			pixyState.spiTurnState = MOVE_RIGHT;
		}
		else
		{
			pixyState.spiTurnState = MOVE_FORWARD;
		}
	}
	else if(pixyState.branchCount > 1)
	{
		{
			if(pixyState.barcodeId == BARCODE_LEFT)
			{
				pixyState.spiTurnState = MOVE_LEFT;
				pixyState.IntersectionDecision = MOVE_LEFT;

			}
			else if(pixyState.barcodeId == BARCODE_RIGHT)
			{
				pixyState.spiTurnState = MOVE_RIGHT;
				pixyState.IntersectionDecision = MOVE_RIGHT;
			}
			else
			{
				pixyState.spiTurnState = MOVE_FORWARD;
				pixyState.IntersectionDecision = MOVE_FORWARD;
			}
		}
	}
//	if(pixyState.spiTurnState==MOVE_FORWARD){printf("LINE SAYS: FEED\r\n");}
//	else if(pixyState.spiTurnState==MOVE_LEFT){printf("LINE SAYS: LEFT\r\n");}
//	else if(pixyState.spiTurnState==MOVE_RIGHT){printf("LINE SAYS: RIGHT\r\n");}
}

void printLCD(uint8_t IPbuffer[]){
    uint8_t IPCopy[16];
    HD44780_Clear();
    HD44780_SetCursor(0,0);
    for (int i = 24, j =0; i < 39; i++, j++){
        IPCopy[j] = IPbuffer[i];
    }
    //printf("\"%s\"\r\n", IPAddress);
    HD44780_PrintStr((const char*)IPCopy);
    HAL_Delay(100);
}
