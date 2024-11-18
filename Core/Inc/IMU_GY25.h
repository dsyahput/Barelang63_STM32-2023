/*
 * IMU_GY25.h
 *
 *  Created on: Feb 22, 2023
 *      Author: abiq
 */

#ifndef INC_IMU_GY25_H_
#define INC_IMU_GY25_H_

#include "stm32f4xx_hal.h"


class IMU_GY25{
private:
	int imu;
	// Kalibrasi A5, 55, 54, 52
	uint8_t kalibrasi[4] = {165,85,84,82};

public:
	void KalibrasiImu();

	uint16_t processingIMU(int imu);
};



#endif /* INC_IMU_GY25_H_ */
