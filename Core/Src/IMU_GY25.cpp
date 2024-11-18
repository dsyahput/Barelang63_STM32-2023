/*
 * IMU_GY25.cpp
 *
 *  Created on: Feb 22, 2023
 *      Author: abiq
 */


#include "IMU_GY25.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

void IMU_GY25::KalibrasiImu(){
	//Kalibrasi A5 dan 54
	HAL_UART_Transmit(&huart1, kalibrasi, 1, 10);
	HAL_UART_Transmit(&huart1, kalibrasi + 2, 1, 10);
	HAL_Delay(100);

	//Kalibrasi A5 dan 55
	HAL_UART_Transmit(&huart1, kalibrasi, 1, 10);
	HAL_UART_Transmit(&huart1, kalibrasi + 1, 1, 10);
	HAL_Delay(100);

	//Kalibrasi A5 dan 52 (Convert to ASCII)
	HAL_UART_Transmit(&huart1, kalibrasi, 1, 10);
	HAL_UART_Transmit(&huart1, kalibrasi + 3, 1, 10);
	HAL_Delay(100);
	}

uint16_t IMU_GY25::processingIMU(int imu){
//	int mumu = imu;
	if(imu < 0) imu += 36000;
	return imu;
}
