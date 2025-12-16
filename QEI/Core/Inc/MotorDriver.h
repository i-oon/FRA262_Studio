/*
 * MotorDriver.h
 *
 *  Created on: May 6, 2025
 *      Author: ioonz
 */

#ifndef INC_MOTORDRIVER_H_
#define INC_MOTORDRIVER_H_

#include "main.h"
#include <math.h>

typedef struct {
	TIM_HandleTypeDef* htim;
	uint16_t tim_ch1;
	uint16_t tim_ch2;
}MOTOR;

typedef struct {
	TIM_HandleTypeDef* htim;
	uint16_t tim_ch1;
}SERVO;


void MotorDriver_Init(MOTOR *motor, TIM_HandleTypeDef *htim, uint16_t tim_ch1, uint16_t tim_ch2);
void MotorDriver_Set(MOTOR *motor, float speed);

void ServoDriver_Init(SERVO *motor, TIM_HandleTypeDef *htim, uint16_t tim_ch1);
void ServoDriver_Set(SERVO *motor, float speed);

#endif /* INC_MOTORDRIVER_H_ */
