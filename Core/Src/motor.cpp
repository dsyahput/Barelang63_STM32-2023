/*
 * motor.cpp
 *
 *  Created on: Feb 22, 2023
 *      Author: abiq
 */
#include "motor.h"

extern TIM_HandleTypeDef htim1;

MotorClass::MotorClass(GPIO_TypeDef *a, GPIO_TypeDef *b, uint16_t a2, uint16_t b2, int pwmNo){
	enA = a;
	enB = b;

	enA_pin = a2;
	enB_pin = b2;

	pwmNum = pwmNo;
}

MotorClass::MotorClass(GPIO_TypeDef *a, uint16_t a2, uint16_t b2, int pwmNo){
	enA = a;
	enB = a;

	enA_pin = a2;
	enB_pin = b2;

	pwmNum = pwmNo;
}


void MotorClass::Motor(int pwm){
	if(pwm == 0){
//		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState)
		HAL_GPIO_WritePin(enA, enA_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(enB, enB_pin, GPIO_PIN_RESET);
	}

	else if(pwm > 0){
		HAL_GPIO_WritePin(enA, enA_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(enB, enB_pin, GPIO_PIN_SET);
	}

	else{
		HAL_GPIO_WritePin(enA, enA_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(enB, enB_pin, GPIO_PIN_RESET);
		pwm = -pwm;

	}
	pwm+=9;
//pwm+=9;
//	pwm += 24;
	if(pwmNum == 1) htim1.Instance->CCR1 = pwm;
	else if(pwmNum == 2) htim1.Instance->CCR2 = pwm;
	else if(pwmNum == 3) htim1.Instance->CCR3 = pwm;
	else if(pwmNum == 4) htim1.Instance->CCR4 = pwm;

}
