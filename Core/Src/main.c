/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "liquidcrystal_i2c.h"
//#include "esp_webserver_html.h"
#include "esp_webserver_uart_ring_buffer_pixy.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LONG_DELAY 1000 // 1 second delay
#define MEDIUM_DELAY 500 // 0.5 seconds delay
#define SHORT_DELAY 250 // 0.25 seconds delay

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile uint16_t delay_value = LONG_DELAY; // 1 second blink

uint8_t buffArray[2][MaxBufferSize]; 	// buffer[0,*] is for UART1
										// buffer[1,*] is for UART2
uint8_t spiBuffArray[MaxSpiSize];		// buffer for SPI2 Rx
uint8_t stateMachine = TEST;	// Initialize State Machine at TEST mode
uint16_t autoServerSize = 0;
uint8_t buffer[4500]="";
uint8_t tpixBuffer[26]="";	// SPI2 C_Passed/C_Failed
uint8_t leftmotbuff[20] = "";
uint8_t rightmotbuff[20] = "";

uint32_t BufferTick;
PixyState pixyState;
UartState uartState[2];
StateFlags stateFlag;
WebRequest webRequest;
MotorState motorState;
MotorCmd motorCmd;
VCWState leftVCW;
VCWState rightVCW;
PIDState pidState;

/* Hexadecimal Arrays for Pixy Commands */
uint8_t pixyGetVersion[4] 		      = {0xAE, 0xC1, 0x0E, 0x00};
uint8_t pixySetLamp[6] 			      = {0xAE, 0xC1, 0x16, 0x02, 0x01, 0x01};
uint8_t pixyGetMainFeatures[6] 	      = {0xAE, 0xC1, 0x30, 0x02, 0x00, 0x07};
uint8_t pixyGetAllFeatures[6] 	      = {0xAE, 0xC1, 0x30, 0x02, 0x01, 0x07};
uint8_t pixyIntersectionTurnLeft[6]   = {0xAE, 0xC1, 0x3A, 0x02, 0x3C, 0x00};
uint8_t pixyIntersectionTurnRight[6]  = {0xAE, 0xC1, 0x3A, 0x02, 0xC4, 0xFF};
uint8_t pixyIntersectionGoStraight[6] = {0xAE, 0xC1, 0x3A, 0x02, 0x00, 0x00};

uint8_t pixySetLampFlag = FALSE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
void GetExpiryValTick(uint32_t * maxTickVal, uint32_t delayMs);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		//__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin) {
		case Blue_B1_Pin:
			//UNUSED(GPIO_Pin);
			 switch (delay_value) {
				  case LONG_DELAY: delay_value = MEDIUM_DELAY;break;
				  case MEDIUM_DELAY: delay_value = SHORT_DELAY;break;
				  default: delay_value = LONG_DELAY;break;
			  }
			 printf("The blink duration is %u milliseconds.\n", delay_value);
			 break;

		 case left_vcw_Pin:
			if(HAL_GPIO_ReadPin(left_vcw_GPIO_Port, left_vcw_Pin)){
				leftVCW.RiseCount++;
				leftVCW.RiseTime = uwTick;
//				printf("RiseTime: %d\r\n", leftVCW.RiseTime);
			}else{
				leftVCW.FallCount++;
				leftVCW.TimeDiff = uwTick - leftVCW.RiseTime;
//				printf("TimeDiff: %d\r\n", leftVCW.TimeDiff);
			}
			break;

		case right_vcw_Pin:
			if(HAL_GPIO_ReadPin(right_vcw_GPIO_Port, right_vcw_Pin)){
				rightVCW.RiseCount++;
				rightVCW.RiseTime = uwTick;
			}else{
				rightVCW.FallCount++;
				rightVCW.TimeDiff = uwTick - rightVCW.RiseTime;
			}
			break;
	}
}
void GetExpiryValTick(uint32_t * maxTickVal, uint32_t delayMs){
	uint32_t currentTick = uwTick;
	*maxTickVal = ((0xFFFFFFFF - currentTick) >= delayMs) ? (currentTick + delayMs) : 0xFFFFFFFF ;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  uint32_t ExpiryTick;
  uint32_t AutoTick;

  Uart_Init();
  String_Init(buffer);
  Flag_Init();
  Pixy_Init();
  Motor_Init();

  HD44780_Init(2);
  HD44780_Clear();

  pidState.kp = 0.2;
  motorState.leftSpd = 900;
  motorState.rightSpd = 900;

  uint8_t IPbuffer[200];

  HAL_GPIO_WritePin(Blue_LD_GPIO_Port, Blue_LD_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Red_LD_GPIO_Port, Red_LD_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GetExpiryValTick(&AutoTick, 200);
  GetExpiryValTick(&ExpiryTick, (uint32_t) delay_value);
  while (1)
  {
	if (uwTick >= ExpiryTick){
	  HAL_GPIO_TogglePin(Green_LD2_GPIO_Port, Green_LD2_Pin);
	  GetExpiryValTick(&ExpiryTick, (uint32_t) delay_value);

//	  if(stateMachine == AUTO_LISTEN)
//	  {
//		printf("Main Vector XY: %u %u\r\n", pixyState.vectorX, pixyState.vectorY);
//		printf("Barcode ID: %u\r\n", pixyState.barcodeId);
//	  }
	}

	/* If UART Error when Rx, Abort */
	for (uint8_t nUART = 0; nUART <2; nUART++)
	{
		if (uartState[nUART].InError == TRUE)
		{
			printf("UART%d Error Code: \"%lu\" ", (nUART+1), uartState[nUART].huartAdd->ErrorCode);
			PrintError(uartState[nUART].huartAdd->ErrorCode, nUART);
			ClearAllData(nUART);
			memset(uartState[nUART].rxbufferAdd, 0, MaxBufferSize);
	        if(uartState[nUART].huartAdd->ErrorCode >= 0b00010000)
	        {
	        	HAL_DMA_Abort(uartState[nUART].rxdmaAdd);
	        }
	        HAL_UART_AbortReceive(uartState[nUART].huartAdd);
			uartState[nUART].rxState = Idle;
			uartState[nUART].InError = FALSE;
		}
	}

	/* Check for State Machine change prompt */
	if (uartState[UART2].rxState == TestAutoCheck){WordSearch(UART2);}

	/* TEST mode */
	if (stateMachine == TEST)
	{
		for (uint8_t nUART = 0; nUART <2; nUART++)
		{
			/* If RxIdle, Start Rx */
			if (uartState[nUART].rxState == Idle){StartReceiver(nUART);}

			/* If RxGotData & Size != 0 */
			if (uartState[nUART].rxState == GotData && uartState[nUART].dataSize!=0){
				RecordIntoCirc(nUART, FALSE);
				uartState[nUART].rxState = Idle;
			}
		}
		/* If TxIdle & Size > 0 */
		if (uartState[UART1].txState == Idle && uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Sz] > 0){
			Transmit(UART1, (uartState[UART1].txbufferAdd),uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Index],uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Sz], TRUE);
		}
		if (uartState[UART2].txState == Idle && uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Sz] > 0){
			Transmit(UART2, (uartState[UART2].txbufferAdd),uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Index],uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Sz], TRUE);
		}
	}

	/* AUTO mode Flag, call ESP Init */
	if (stateMachine == AUTO_SERVER)
	{
		Auto_Init(IPbuffer);
		if (stateFlag.rxFlag == 15)
		{
			/* if circular buffer completes, change to AUTO mode */
			if (uartState[UART1].rxCircArrayPtr == (uartState[UART1].txCircArrayPtr))
			{
//				ClearAllData(UART1);
				stateMachine = AUTO_LISTEN;
			}
		}

		for (uint8_t nUART = 0; nUART <2; nUART++)
		{
			/* If RxIdle, Start Rx */
			if (uartState[nUART].rxState == Idle){StartReceiver(nUART);}

			/* If RxGotData & Size != 0 */
			if (uartState[nUART].rxState == GotData && uartState[nUART].dataSize!=0){
				RecordIntoCirc(nUART, FALSE);
				uartState[nUART].rxState = Idle;
			}
		}
		/* If TxIdle & Size > 0 */
		if (uartState[UART1].txState == Idle && uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Sz] > 0){
			Transmit(UART1, (uartState[UART1].txbufferAdd),uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Index],uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Sz], TRUE);
		}
		if (uartState[UART2].txState == Idle && uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Sz] > 0){
			Transmit(UART2, (uartState[UART2].txbufferAdd),uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Index],uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Sz], TRUE);
		}
	}

	/* AUTO mode */
	if (stateMachine == AUTO_LISTEN)
	{
		/* If UART1 RxIdle, start Rx */
		if (uartState[UART1].rxState == Idle){StartReceiver(UART1);}

		/* If UART1 RxGotData & Size != 0 */
		if (uartState[UART1].rxState == GotData && uartState[UART1].dataSize!=0){
			RecordIntoCirc(UART1, TRUE);
			uartState[UART1].rxState = Idle;
		}

		/* If UART1 TxIdle */
		if (uartState[UART1].txState == Idle)
		{
			/* If UART1 TxCirc Size = 0 */
			if(uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Sz] > 0)
			{
				RequestHandle();
			}
		}
		/* If SPI2 Idle */
		if (pixyState.spiState == Idle)
		{
		/* Flush SPI2 Rx Buffer */
			memset(pixyState.rxbufferAdd, 0, MaxSpiSize);

			/* Set dataSize to Max SPI Size */
			pixyState.dataSize = MaxSpiSize;
			if (pixySetLampFlag == TRUE && pixyState.IntersectionDecision!= NONE)
			{
				switch(pixyState.IntersectionDecision)
				{
				case MOVE_FORWARD:
					HAL_SPI_Transmit_IT(pixyState.hspi, pixyState.IntersectionGoStraight, 6);
				case MOVE_LEFT:
					HAL_SPI_Transmit_IT(pixyState.hspi, pixyState.IntersectionTurnLeft, 6);
				case MOVE_RIGHT:
					HAL_SPI_Transmit_IT(pixyState.hspi, pixyState.IntersectionTurnRight, 6);
				}
				pixyState.IntersectionDecision = NONE;
			}
			else if(pixySetLampFlag == TRUE)
			{
//				if (uwTick >= AutoTick){
				/* Transmit GetAllFeatures */
				HAL_SPI_Transmit_IT(pixyState.hspi, pixyState.getAllFeatures, 6);
//				GetExpiryValTick(&AutoTick, (uint32_t) delay_value);
//				}
			}
			else
			{
				/* Transmit SetLamp */
				HAL_SPI_Transmit_IT(pixyState.hspi, pixyState.setLamp, 6);
				pixySetLampFlag = TRUE;
			}

			/* Set SPI2 to Busy */
			pixyState.spiState = Busy;
		}

		if(pixyState.spiState == Busy)
		{
			if(uwTick>=AutoTick)
			{
				HAL_SPI_Abort_IT(&hspi2);
				pixyState.spiState = Idle;
				GetExpiryValTick(&AutoTick, 500);
			}
		}

		/* If SPI2 GotData */
		if(pixyState.spiState == GotData)
		{
			/* Process Data */
			pixyState.spiChecksum = AFC1Search(pixyState.rxbufferAdd, 0, &pixyState.dataSize);

			/* Set SPI2 to Idle */
			pixyState.spiState = Idle;
		}
	}

	/* TPIX mode */
	if (stateMachine == TPIX)
	{
		/* If UART2 RxIdle, start Rx */
		if(uartState[UART2].rxState == Idle){StartReceiver(UART2);}

		/* If UART2 RxGotData */
		if (uartState[UART2].rxState == GotData)
		{
			/* Clear out buffer of Max SPI Size before receiving data */
			for(uint16_t i = uartState[UART2].rxbuffDex + uartState[UART2].dataSize; i < (MaxSpiSize + uartState[UART2].rxbuffDex); i++)
			{
				uartState[UART2].rxbufferAdd[i] = 0;
			}

			/* Set Size to Max SPI Size */
			uartState[UART2].dataSize = MaxSpiSize;
			RecordIntoCirc(UART2, FALSE);
		}

		/* If PixyIdle */
		if (pixyState.spiState == Idle)
		{
			/* If UART2 Circular Buffer is not empty */
			if (uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Sz]!=0)
			{
				/* Set Size to Max SPI Size */
				uartState[UART1].dataSize = MaxSpiSize;

				/* Transmit and Receive the Max SPI Size */
				HAL_SPI_TransmitReceive_IT(&hspi2, &(uartState[UART2].rxbufferAdd[uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Index]]), (uartState[UART1].rxbufferAdd + uartState[UART1].rxbuffDex), uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Sz]);

				/* Set SPI State to Busy */
				pixyState.spiState = Busy;

				/* Set UART2 Circular Buffer to empty */
				uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Sz] = 0;

				if (uartState[UART2].txCircArrayPtr == (CircSize - 1))
				{
					uartState[UART2].txCircArrayPtr = 0;
				}
				else
				{
					uartState[UART2].txCircArrayPtr+=1;
				}

				if(uartState[UART2].rxState == Wait)
				{
					uartState[UART2].rxState = GotData;
				}
			}
		}

		/* If PixyGotData */
		if (pixyState.spiState == GotData)
		{
			/* Search for header bytes, validate checksum */
			pixyState.spiChecksum = AFC1Search(&(uartState[UART1].rxbufferAdd[uartState[UART1].rxbuffDex]), &uartState[UART1].rxbuffDex, &uartState[UART1].dataSize);

			/* If checksum valid */
			if (pixyState.spiChecksum == 2)
			{
				strcpy((char*)(&(uartState[UART1].rxbufferAdd[uartState[UART1].rxbuffDex + uartState[UART1].dataSize])),(char*)&(tpixBuffer[13]));
			}

			/* If checksum invalid */
			if (pixyState.spiChecksum == 1)
			{
				strcpy((char*)(&(uartState[UART1].rxbufferAdd[uartState[UART1].rxbuffDex + uartState[UART1].dataSize])),(char*)&(tpixBuffer[0]));
			}

			/* If checksum was found */
			if (pixyState.spiChecksum > 0)
			{
				uartState[UART1].dataSize += 12;
				RecordIntoCirc(UART1,FALSE);
			}

			/* Set SPI State to Idle */
			pixyState.spiState = Idle;
		}

		/* If UART2 TxIdle */
		if (uartState[UART2].txState == Idle){
			if(uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Sz] > 0){
			Transmit(UART2, (uartState[UART2].txbufferAdd),uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Index],uartState[UART1].circArray[uartState[UART1].txCircArrayPtr][Sz], TRUE);
			}
		}
	}

	if (stateMachine == TMOT)
	{
		/* If UART2 RxIdle, start Rx */
		if (uartState[UART2].rxState == Idle){StartReceiver(UART2);}

		/* If UART2 RxGotData & Size != 0 */
		if (uartState[UART2].rxState == GotData && uartState[UART2].dataSize!=0){
			RecordIntoCirc(UART2, FALSE);
			uartState[UART2].rxState = Idle;
		}

		/* If UART2 TxIdle */
		if (uartState[UART2].txState == Idle && uartState[UART2].circArray[uartState[UART2].txCircArrayPtr][Sz] > 0)
		{
			MotorRequestHandle();
			VCW();
			if (uwTick >= BufferTick)
			{
				PID();
			}
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Blue_LD_Pin|Red_LD_Pin|Green_LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blue_B1_Pin */
  GPIO_InitStruct.Pin = Blue_B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Blue_B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : left_vcw_Pin right_vcw_Pin */
  GPIO_InitStruct.Pin = left_vcw_Pin|right_vcw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Blue_LD_Pin Red_LD_Pin */
  GPIO_InitStruct.Pin = Blue_LD_Pin|Red_LD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : tact_sw_Pin */
  GPIO_InitStruct.Pin = tact_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(tact_sw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LD2_Pin */
  GPIO_InitStruct.Pin = Green_LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
