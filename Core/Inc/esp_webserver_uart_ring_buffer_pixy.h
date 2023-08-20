/*
 * esp_webserver_uart_ring_buffer_pixy.h
 *
 */

#ifndef INC_ESP_WEBSERVER_UART_RING_BUFFER_PIXY_H_
#define INC_ESP_WEBSERVER_UART_RING_BUFFER_PIXY_H_

/* INCLUDE */
#include "main.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_tim.h"

/* USER CODE INCLUDE */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "liquidcrystal_i2c.h"
//#include "esp_webserver_html.h"

/* USER CODE DEFINE */
#define UART1 			0
#define UART2 			1
#define M_SPI2			2

#define TRUE 			1
#define FALSE 			0

/* uartState.buffer */
#define MaxBufferSize 	4501 //4500 max with 1 null char
#define BuffRewindLimit 2500

/* uartState.circArray[10][i] */
#define CircSize		10
#define Index 			0
#define Sz 				1

/* uartState.rxState uartState.txState */
#define NoData 			0
#define GotData 		1
#define Idle 			2
#define Receiving 		3
#define Transmitting 	4
#define Wait 			5
#define TestAutoCheck 	6
#define Busy 			7
#define Error 			5

/* stateMachine */
#define TEST 			0
#define AUTO_SERVER 	1
#define AUTO_LISTEN 	2
#define TPIX 			3
#define TMOT            4

/* toggleLED.switch(i) */
#define Blue 			0
#define Red 			1

/* Web Request */
#define Spam 			0
#define WebServer 		1
#define ToggleBlueLED 	2
#define ToggleRedLED 	3
#define GetGreenLED 	4

/* ESP COMMANDS */
#define AT 				"AT\r\n"
#define AT_OK			"OK\r\n"
#define RESTORE 		"AT+RESTORE\r\n"
#define RST 			"AT+RST\r\n"
#define CWMODE 			"AT+CWMODE=1\r\n"
#define CWJAP 			"AT+CWJAP=\"kenneth's\",\"rycbar123\"\r\n"
#define CIFSR 			"AT+CIFSR\r\n"
#define CIPMUX 			"AT+CIPMUX=1\r\n"
#define CIPSERVER 		"AT+CIPSERVER=1,80\r\n"
#define CIPSTATUS 		"AT+CIPSTATUS\r\n"
#define CIPSEND 		"AT+CIPSEND= ,1985\r\n"
#define CIPCLOSE 		"AT+CIPCLOSE= \r\n"
#define CIPSENDLED 		"AT+CIPSEND= ,2\r\n"
#define LEFTMOT         "Left: "
#define RIGHTMOT        "Right: "

/* HTML STATES */
#define BlueLEDOFF 		"10\r\n"
#define BlueLEDON 		"11\r\n"
#define RedLEDOFF 		"20\r\n"
#define RedLEDON 		"21\r\n"
#define PixyTurnFEED 	"30\r\n"
#define PixyTurnLEFT	"31\r\n"
#define PixyTurnRIGHT	"32\r\n"
#define PixyTurnSTOP 	"34\r\n"

/* SPI STATES */
#define MaxSpiSize 		132
#define SpiFAILED 		"\r\nC_Failed\r\n"
#define SpiPASSED 		"\r\nC_Passed\r\n"
#define NONE            5
#define MOVE_FORWARD	0
#define MOVE_RIGHT		1
#define MOVE_LEFT		2
#define MOVE_STOP		3

/* BARCODE BY ID */
#define BARCODE_STOP	0
#define BARCODE_RIGHT	2
#define BARCODE_LEFT	4
#define BARCODE_NULL	16


#define WheelRadius 16.875
#define ARR_VALUE 3599
#define CYCLES_PER_ANG_VEL 5.0

/* WEBSERVER APIX */
#define html			"<html lang=\"en\"><head><meta charset=\"UTF-8\"><meta hhtp-equiv=\"X-UA-Compatible\" content=\"IE=edge\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>ESP-01S Client</title></head><body><H3>Click on the Button to toggle the LED</H3><p>Blue LED State<span id=\"BlueLed\"> </span></p><p>Red LED State<span id=\"RedLed\"> </span></p><button id = \"1\" onclick = \"ledToggle(this.id)\">Blue Led</button><button id = \"2\" onclick = \"ledToggle(this.id)\">Red Led</button><p>Pixy State<span id=\"GreenLed\"> __</span></p><button id = \"3\" onclick=\"disable(this.id)\">Update Continuously</button><button id = \"4\" onclick=\"enable(this.id)\" disabled>Stop Update</button><script>var id;function ledToggle(clicked_id){var xhr= new XMLHttpRequest();var url = clicked_id;xhr.open(\"GET\",url,true);xhr.send();xhr.onreadystatechange = function(){if(this.readyState == 4 && this.status == 200){if(this.responseText==\"10\"){document.getElementById(\"BlueLed\").innerHTML=\" OFF\";}else if(this.responseText==\"11\"){document.getElementById(\"BlueLed\").innerHTML=\" ON\";}else if(this.responseText==\"20\"){document.getElementById(\"RedLed\").innerHTML=\" OFF\";}else if(this.responseText==\"21\"){document.getElementById(\"RedLed\").innerHTML=\" ON\";}}};}function disable(clicked_id){id = setInterval(function interval(){var xhr= new XMLHttpRequest();var url = clicked_id;xhr.open(\"GET\",url,true);xhr.send();xhr.onreadystatechange = function(){if(this.readyState == 4 && this.status == 200){if(this.responseText==\"30\"){document.getElementById(\"GreenLed\").innerHTML=\" FEED\";}else if(this.responseText==\"31\"){document.getElementById(\"GreenLed\").innerHTML=\" Rotate Right\";}else if(this.responseText==\"32\"){document.getElementById(\"GreenLed\").innerHTML=\" Rotate Left\";}}};document.getElementById(\"3\").disabled = true;document.getElementById(\"4\").disabled = false;}, 3000);}function enable(clicked_id){clearInterval(id);document.getElementById(\"3\").disabled = false;document.getElementById(\"4\").disabled = true;}</script></body></html>"

/* USER CODE TYPEDEF STRUCT */
typedef struct PixyState{
	SPI_HandleTypeDef*	hspi;
	uint8_t* 			rxbufferAdd;
	uint8_t 			spiState;
	uint8_t 			spiChecksum;
	uint16_t 			dataSize;

	uint8_t*			getVersion;
	uint8_t*			setLamp;
	uint8_t*			getAllFeatures;
	uint8_t*			getMainFeatures;
	uint8_t*            IntersectionTurnLeft;
	uint8_t*            IntersectionTurnRight;
	uint8_t*            IntersectionGoStraight;


	uint8_t             IntersectionDecision;
	uint8_t 			vectorX;
	uint8_t             vectorY;
	uint8_t				branchCount;
	uint8_t				barcodeId;
	uint8_t             barcodeY;
	uint8_t 			spiTurnState;
}PixyState;

typedef struct UartState{
	UART_HandleTypeDef* huartAdd;
	DMA_HandleTypeDef* 	rxdmaAdd;
	DMA_HandleTypeDef* 	txdmaAdd;
	uint8_t 			txDex;
	uint8_t 			rxDex;
	uint8_t* 			txbufferAdd;
	uint8_t* 			rxbufferAdd;
	uint8_t 			rxState;
	uint8_t 			txState;
	uint8_t 			InError;
	uint16_t 			dataSize;
	uint16_t 			rxbuffDex;
	uint16_t 			remainSize;
	uint8_t 			rxCircArrayPtr;
	uint8_t 			txCircArrayPtr;
	uint16_t 			circArray[10][2];
}UartState;

typedef struct StateFlags{
	uint8_t rxFlag;
	uint8_t autoinitFlag;
}StateFlags;

typedef struct WebRequest{
	uint8_t process;
	uint8_t requestAction;
	uint8_t request;
	uint8_t action;
	uint8_t ipd;
	uint8_t get;
}WebRequest;

typedef struct MotorState{
	int leftSpd;
	int rightSpd;
	int velocity;

}MotorState;
typedef struct MotorCmd{
	uint8_t startMotor;
	uint8_t stopMotor;
	uint8_t bothMotors;
	uint8_t leftMotor;
	uint8_t rightMotor;
	uint8_t vel;

}MotorCmd;

typedef struct VCWState{
	uint32_t RiseCount;
	uint32_t FallCount;
	uint32_t RiseTime;
	uint32_t TimeDiff;
	uint32_t velocity;
	uint32_t Distance;
}VCWState;

typedef struct PIDState{
	float kp;
	uint8_t ErrorChange;
	uint8_t TotalError;
	uint8_t TotalBase;
}PIDState;

/* EXTERN VARIABLES */
extern I2C_HandleTypeDef 	hi2c1;
extern SPI_HandleTypeDef 	hspi2;
extern UART_HandleTypeDef 	huart1;
extern UART_HandleTypeDef 	huart2;
extern DMA_HandleTypeDef 	hdma_usart1_rx;
extern DMA_HandleTypeDef 	hdma_usart1_tx;
extern DMA_HandleTypeDef 	hdma_usart2_rx;
extern DMA_HandleTypeDef 	hdma_usart2_tx;
extern TIM_HandleTypeDef    htim3;

/* USER CODE EXTERN VARIABLES */
extern uint8_t 	buffArray[2][MaxBufferSize];
extern uint8_t 	spiBuffArray[MaxSpiSize];
extern uint8_t 	stateMachine;
extern uint16_t autoServerSize;
extern uint8_t 	buffer[4500];
extern uint8_t 	tpixBuffer[26];
extern uint8_t	junctionTurn;
extern uint8_t stateMachine;
extern uint8_t leftmotbuff[20];
extern uint8_t rightmotbuff[20];

extern uint32_t bufferTick;
extern PixyState pixyState;
extern UartState uartState[2];
extern StateFlags stateFlag;
extern WebRequest webRequest;
extern MotorState motorState;
extern MotorCmd motorCmd;
extern VCWState leftVCW;
extern VCWState rightVCW;
extern PIDState pidState;

extern uint8_t pixyGetVersion[4];
extern uint8_t pixySetLamp[6];
extern uint8_t pixyGetMainFeatures[6];
extern uint8_t pixyGetAllFeatures[6];

extern uint32_t BufferTick;

/* NON-BLOCKING DELAY */
void GetExpiryValTick(uint32_t * maxTickVal, uint32_t delayMs);
//static uint32_t DelayTick;

/* USER CODE FUNCTION PROTOTYPES */
void Uart_Init(void);
void Flag_Init(void);
void Pixy_Init(void);
void Auto_Init(uint8_t IPbuffer[]);
void String_Init(uint8_t* buffer);

void ClearAllData(uint8_t nUART);
void StartReceiver(uint8_t nUART);
void Transmit(uint8_t nUART, uint8_t* buffer, uint16_t index, uint16_t size, uint8_t updateCirc);
void TransmitESP(uint8_t* buffer, uint16_t size, uint8_t updateCirc);

void RecordIntoCirc(uint8_t nUART, uint8_t delimiter);

void WordSearch(uint8_t huart);
void prepend(int huart);
uint8_t AFC1Search(uint8_t* buffer, uint16_t* pose, uint16_t* size);
void findFeature(uint8_t* buffer, uint16_t start, uint16_t end);
void MotorRequestHandle(void);
void VCW(void);
void PID(void);
void RequestHandle(void);

void ToggleLED(uint8_t led);
void printLCD(uint8_t IPbuffer[]);
void PrintError(uint8_t errorCode, uint8_t nUART);

#endif /* INC_ESP_WEBSERVER_UART_RING_BUFFER_PIXY_H_ */
