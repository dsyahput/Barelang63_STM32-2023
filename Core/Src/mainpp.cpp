/*
 * mainpp.cpp
 *
 *  Created on: Feb 22, 2023
 *      Author: abiq
 */

#include "main.h"
#include "mainpp.h"
#include "DataStruct.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "IMU_GY25.h"
#include "motor.h"
#include "Encoder.h"
#include "Kinematic.h"

// Declaring Variable
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

PC Command;
STM32 Send;
Point2D anu;
Point2D Posrobot;

IMU_GY25 *IMU;
MotorClass *Motor1;
MotorClass *Motor2;
MotorClass *Motor3;
MotorClass *Motor4;

Encoder *Encoder1;
Encoder *Encoder2;
Encoder *Encoder3;
Encoder *Encoder4;

kinematic::Motor *kinematicaja;

#define TX_Buff_Size 32
uint8_t TX_Buff[TX_Buff_Size];

#define IMU_RX_Buff_Size 16
uint8_t IMU_RX_Buff[IMU_RX_Buff_Size];
int16_t rawImu;

unsigned long tick = 0;

unsigned long rxTick = 0;

int16_t hehe = 200;

// Void
void init(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	IMU = new IMU_GY25();

	Motor1 = new MotorClass(M1_Port, M1_EnbA, M1_EnbB, 1);
	Motor2 = new MotorClass(M2_Port, M2_EnbA, M2_EnbB, 2);
	Motor3 = new MotorClass(M3_Port, M3_EnbA, M3_EnbB, 3);
	Motor4 = new MotorClass(M4_PortA, M4_PortB, M4_EnbA, M4_EnbB, 4);

//	Motor1 = new MotorClass(En1A_GPIO_Port, En1A_Pin, En1B_Pin, 1);
//	Motor2 = new MotorClass(En2A_GPIO_Port, En2A_Pin, En2B_Pin, 2);
//	Motor3 = new MotorClass(En3A_GPIO_Port, En3A_Pin, En3B_Pin, 3);

	Encoder1 = new Encoder(Enc1A_GPIO_Port, Enc1B_GPIO_Port, Enc1A_Pin, Enc1B_Pin);
	Encoder2 = new Encoder(Enc2A_GPIO_Port, Enc2B_GPIO_Port, Enc2A_Pin, Enc2B_Pin);
	Encoder3 = new Encoder(Enc4A_GPIO_Port, Enc4B_GPIO_Port, Enc4A_Pin, Enc4B_Pin);
	Encoder4 = new Encoder(Enc3A_GPIO_Port, Enc3B_GPIO_Port, Enc3A_Pin, Enc3B_Pin);
//	Motor1 = new MotorClass(En1A_GPIO_Port, En1A_Pin, En1B_Pin, 4);

	IMU->KalibrasiImu();

	IMU_RX_Interrupt_Start();
}


void loop(){
//	SendDatatoPC();
	if(HAL_GetTick() - tick >= 10){
	kinematicaja->inverseKinematic(5, 0, 0,anu);
	Motor2->Motor(anu.a);
	Motor3->Motor(anu.b);
	Motor4->Motor(anu.c);
	}
	tick = HAL_GetTick();

	kinematicaja->calcOdom();
	TakeEncoder();

}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART1){//Serial Imu-Gy25 to STM32
		rawImu = -(IMU_RX_Buff[1] << 8 | IMU_RX_Buff[2]);

		Send.Imu = IMU->processingIMU(rawImu);

		IMU_RX_Interrupt_Start();

	}

}


void receiveData(uint8_t* buf, uint32_t len){
	memcpy((uint8_t*)&Command, buf, sizeof(Command));
	rxTick = HAL_GetTick();
}


void SendDatatoPC(){
	if(HAL_GetTick() - tick >= 10){
//	Motor1->Motor(50);
//	Motor2->Motor(5);
//	Motor3->Motor(5);
//	Motor4->Motor(5);
	if(HAL_GetTick()- rxTick < 300){
		Motor2->Motor(Command.motorSpeed[0]);
		Motor3->Motor(Command.motorSpeed[1]);
		Motor4->Motor(Command.motorSpeed[2]);
	}
	else{
			Motor1->Motor(0);
			Motor2->Motor(0);
			Motor3->Motor(0);
			Motor4->Motor(0);
	}

//	Send.Imu = hehe++;
	tick = HAL_GetTick();
	TakeEncoder();
	memcpy(TX_Buff, (uint8_t*)&Send, sizeof(Send));
	CDC_Transmit_FS(TX_Buff, sizeof(Send));
	}
}


void TakeEncoder(){
	Send.Encoder[0] = Encoder1->GetCounter();
//	Encoder1->ResetCounter();
	Send.Encoder[1] = Encoder2->GetCounter();
//	Encoder2->ResetCounter();
	Send.Encoder[2] = Encoder3->GetCounter();
//	Encoder3->ResetCounter();
}


void IMU_RX_Interrupt_Start(){
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, IMU_RX_Buff, IMU_RX_Buff_Size);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_TC);
}
