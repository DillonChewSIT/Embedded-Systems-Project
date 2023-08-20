/*
 * esp_webserver_uart_ring_buffer_motor.c
 *
 *  Created on: Mar 30, 2023
 *      Author: dillo
 */
#include "esp_webserver_uart_ring_buffer_pixy.h"

void MotorRequestHandle(void){
	uint8_t circPtr = uartState[UART2].txCircArrayPtr;
	uint16_t buffIdx = uartState[UART2].circArray[circPtr][Index];
	char* buf = (char*)&(uartState[UART2].rxbufferAdd[buffIdx]);
	memset(leftmotbuff, 0, 20);
	memset(rightmotbuff, 0, 20);
	memcpy(leftmotbuff, LEFTMOT, 6);
	memcpy(rightmotbuff, RIGHTMOT, 7);

	if(strstr(buf, "at+start\r\n") !=NULL)
	{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		int left = motorState.leftSpd;
		int right = motorState.rightSpd;
		sprintf(leftmotbuff+strlen(leftmotbuff), "%d", left);
		strcpy(leftmotbuff+strlen(leftmotbuff), "\r\n");
		sprintf(rightmotbuff+strlen(rightmotbuff), "%d", right);
		strcpy(rightmotbuff+strlen(rightmotbuff), "\r\n");
		strcpy(leftmotbuff+strlen(leftmotbuff), rightmotbuff);
		TransmitMOT(leftmotbuff, strlen(leftmotbuff), TRUE);
	}
	else if (strstr(buf, "at+stop\r\n")!=NULL)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		TransmitMOT("All Stopped\r\n", 13, TRUE);
	}
	else if (sscanf(buf, "at+m1a=%u+m2a=%u", &motorState.leftSpd, &motorState.rightSpd) == 2)
	{
		//printf("left wheel running at: %u, right wheel running at: %u", motorState.leftSpd, motorState.rightSpd);
		TIM3->CCR3 = motorState.leftSpd;
		TIM3->CCR1 = motorState.rightSpd;
		int left = motorState.leftSpd;
		int right = motorState.rightSpd;
		sprintf(leftmotbuff+strlen(leftmotbuff), "%d", left);
		strcpy(leftmotbuff+strlen(leftmotbuff), "\r\n");
		sprintf(rightmotbuff+strlen(rightmotbuff), "%d", right);
		strcpy(rightmotbuff+strlen(rightmotbuff), "\r\n");
		strcpy(leftmotbuff+strlen(leftmotbuff), rightmotbuff);
		TransmitMOT(leftmotbuff, strlen(leftmotbuff), TRUE);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	}
	else if (sscanf(buf, "at+m1a=%u", &motorState.leftSpd) == 1)
	{
		TIM3->CCR3 = motorState.leftSpd;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		int left = motorState.leftSpd;
		sprintf(leftmotbuff+strlen(leftmotbuff), "%d", left);
		strcpy(leftmotbuff+strlen(leftmotbuff), "\r\n");
		TransmitMOT(leftmotbuff, strlen(leftmotbuff), TRUE);
	}
	else if (sscanf(buf, "at+m2a=%u", &motorState.rightSpd) == 1)
	{
		TIM3->CCR1 = motorState.rightSpd;
		int right = motorState.rightSpd;
		sprintf(rightmotbuff+strlen(rightmotbuff), "%d", right);
		strcpy(rightmotbuff+strlen(rightmotbuff), "\r\n");
		TransmitMOT(rightmotbuff, strlen(rightmotbuff), TRUE);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		TransmitMOT(rightmotbuff, strlen(rightmotbuff), TRUE);
	}
	else if (strcmp(buf, "at\r\n") == 0)
	{
		int left = motorState.leftSpd;
		int right = motorState.rightSpd;
		sprintf(leftmotbuff+strlen(leftmotbuff), "%d", left);
		strcpy(leftmotbuff+strlen(leftmotbuff), "\r\n");
		sprintf(rightmotbuff+strlen(rightmotbuff), "%d", right);
		strcpy(rightmotbuff+strlen(rightmotbuff), "\r\n");
		strcpy(leftmotbuff+strlen(leftmotbuff), rightmotbuff);
		TransmitMOT(leftmotbuff, strlen(leftmotbuff), TRUE);
	}
	else if (sscanf(buf, "at+mvel=%u", &motorState.velocity) == 1)
	{
		return;
	}
	else
	{
		TransmitMOT("ERROR\r\n", 7, TRUE);
	}
}

void VCW(void)
{
	//Finding the distance
	uint8_t LeftTotal = leftVCW.RiseCount + leftVCW.FallCount;
	uint8_t RightTotal = rightVCW.RiseCount + rightVCW.FallCount;
	leftVCW.Distance = LeftTotal * WheelRadius;
	rightVCW.Distance = RightTotal * WheelRadius;

	//Finding velocity
	if (uwTick - leftVCW.RiseTime > 500){leftVCW.TimeDiff = 0;}
	if (uwTick - rightVCW.RiseTime > 500){rightVCW.TimeDiff = 0;}

	uint8_t leftTimeDiff = leftVCW.TimeDiff;
	uint8_t rightTimeDiff = rightVCW.TimeDiff;
	if(leftVCW.TimeDiff)
	{
		leftVCW.velocity = WheelRadius / leftTimeDiff;
	}
	else{
		leftVCW.velocity = 0;
	}

	if(rightVCW.TimeDiff)
	{
		rightVCW.velocity = WheelRadius / rightTimeDiff;
	}
	else{
		rightVCW.velocity = 0;
	}
}


void PID(void)
{
	int leftAngError = motorState.velocity - leftVCW.velocity;
	int rightAngError = motorState.velocity - rightVCW.velocity;
	int leftError = pidState.kp * leftAngError * CYCLES_PER_ANG_VEL;
	int rightError = pidState.kp * rightAngError * CYCLES_PER_ANG_VEL;
	if(TIM3->CCR3 + leftError < ARR_VALUE) TIM3->CCR3 += leftError;
	if(TIM3->CCR1 + rightError < ARR_VALUE) TIM3->CCR1 += rightError;

	printf("Current left velocity: %d; Target velocity: %d; Diff: %d; PWM: %d; \r\n" , leftVCW.velocity, motorState.velocity, leftAngError, TIM3->CCR1);
	printf("Current right velocity: %d; Target velocity: %d; Diff: %d; PWM: %d; \r\n" , rightVCW.velocity, motorState.velocity, rightAngError, TIM3->CCR3);

//	printf("Right Velocity: %u; (Diff) = %d ;(Target) - %u ;(Tracking). PWM: %d\r\n;", rightAngError, motorState.velocity, rightVCW.velocity, TIM3->CCR3);
//	printf("left Velocity: %u; (Diff) = %d ;(Target) - %u ;(Tracking). PWM: %d\r\n;", leftAngError, motorState.velocity, leftVCW.velocity, TIM3->CCR1);
	GetExpiryValTick(&BufferTick, 10);
}

