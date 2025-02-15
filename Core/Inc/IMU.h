/*
 * IMU.h
 *
 *  Created on: Jan 30, 2025
 *      Author: user
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "ICM20948.h"
#include "oled.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_uart.h"

extern UART_HandleTypeDef huart3;//to send debug info using uart

//store settings for the IMU
#define GYRO_SENS  GYRO_FULL_SCALE_2000DPS
#define ACCEL_SENS ACCEL_FULL_SCALE_2G


typedef struct {
    float x, y, z;
} vec3;

//float magOffset[] = {14.7,-5.7,-50.92};
float magOffset[] = {3,-20.18,-33.65};

void SendMagDataUART(float mag[3])
{
	char buf[8]={};
	char axis[3]= {'X','Y','Z'};
	static float minmaxmag[3][2]= {
			{100, 0},
		    {100, 0},
		    {100, 0}};

	minmaxmag[0][0]=fmin(minmaxmag[0][0],mag[0]);
	minmaxmag[0][1]=fmax(minmaxmag[0][1],mag[0]);

	minmaxmag[1][0]=fmin(minmaxmag[1][0],mag[1]);
	minmaxmag[1][1]=fmax(minmaxmag[1][1],mag[1]);

	minmaxmag[2][0]=fmin(minmaxmag[2][0],mag[2]);
	minmaxmag[2][1]=fmax(minmaxmag[2][1],mag[2]);
	//snprintf(buf,sizeof(buf),"%d",0);

	for(int i=0;i<3;i++)
	{
		HAL_UART_Transmit(&huart3,(uint8_t *)&axis[i],1,0xFFFF);
		HAL_UART_Transmit(&huart3,(uint8_t *)" Min: ",6,0xFFFF);
		snprintf(buf,sizeof(buf),"%.2f",minmaxmag[i][0]);
		HAL_UART_Transmit(&huart3,(uint8_t *)buf,sizeof(buf),0xFFFF);
		HAL_UART_Transmit(&huart3,(uint8_t *)" Max: ",6,0xFFFF);
		snprintf(buf,sizeof(buf),"%.2f",minmaxmag[i][1]);
		HAL_UART_Transmit(&huart3,(uint8_t *)buf,sizeof(buf),0xFFFF);
		HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n",2,0xFFFF);
	}

	}

void ShowMagDataOled(float mag[3])
{
	char buf[15]={};

	static float minmaxmag[3][2]= {
			{100, 0},
		    {100, -30},
		    {100, -30}};

	minmaxmag[0][0]=fmin(minmaxmag[0][0],mag[0]);
	minmaxmag[0][1]=fmax(minmaxmag[0][1],mag[0]);

	minmaxmag[1][0]=fmin(minmaxmag[1][0],mag[1]);
	minmaxmag[1][1]=fmax(minmaxmag[1][1],mag[1]);

	minmaxmag[2][0]=fmin(minmaxmag[2][0],mag[2]);
	minmaxmag[2][1]=fmax(minmaxmag[2][1],mag[2]);
	//snprintf(buf,sizeof(buf),"%d",0);

	for(int i=0;i<3;i++)
	{
		snprintf(buf,sizeof(buf),"%5.1f, %5.1f",minmaxmag[i][0],minmaxmag[i][1]);

		OLED_ShowString(0, 10*i, buf);
	}

	}

#endif /* INC_IMU_H_ */
