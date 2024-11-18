/*
 * motor.h
 *
 *  Created on: Feb 22, 2023
 *      Author: abiq
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

//#include "main.h"
#include "stm32f4xx_hal.h"
//#include "mainpp.h"

class MotorClass{
private:
	GPIO_TypeDef *enA, *enB;
	uint16_t pwm_pin, enA_pin, enB_pin;
	int8_t pwmNum;

public:
	MotorClass(GPIO_TypeDef *a, GPIO_TypeDef *b, uint16_t a2, uint16_t b2, int pwmNo);

	MotorClass(GPIO_TypeDef *a, uint16_t a2, uint16_t b2, int pwmNo);

	void Motor(int pwm);
};




#endif /* INC_MOTOR_H_ */
