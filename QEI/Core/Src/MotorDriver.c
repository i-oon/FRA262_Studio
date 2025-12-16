/*
 * MotorDriver.c
 *
 *  Created on: May 6, 2025
 *      Author: ioonz
 */

#include "MotorDriver.h"
#include "math.h"

int check_motor = 0;

void MotorDriver_Init(MOTOR *motor, TIM_HandleTypeDef *htim, uint16_t tim_ch1,
		uint16_t tim_ch2) {
	motor->htim = htim;
	motor->tim_ch1 = tim_ch1;
	motor->tim_ch2 = tim_ch2;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, tim_ch1);
	HAL_TIM_PWM_Start(htim, tim_ch2);

}

void MotorDriver_Set(MOTOR *motor, float speed) {
	check_motor = 1;
	if (speed < 0) {
		__HAL_TIM_SET_COMPARE(motor->htim, motor->tim_ch1, 0);
		__HAL_TIM_SET_COMPARE(motor->htim, motor->tim_ch2, -(speed));
	} else if (speed > 0) {
		__HAL_TIM_SET_COMPARE(motor->htim, motor->tim_ch1, speed);
		__HAL_TIM_SET_COMPARE(motor->htim, motor->tim_ch2, 0);
	} else {
		__HAL_TIM_SET_COMPARE(motor->htim, motor->tim_ch1, 0);
		__HAL_TIM_SET_COMPARE(motor->htim, motor->tim_ch2, 0);
	}
}

void ServoDriver_Init(SERVO *motor, TIM_HandleTypeDef *htim, uint16_t tim_ch1){
	motor->htim = htim;
	motor->tim_ch1 = tim_ch1;

	HAL_TIM_Base_Start(htim);
	HAL_TIM_PWM_Start(htim, tim_ch1);
}
void ServoDriver_Set(SERVO *motor, float speed){
	check_motor = 2;
	__HAL_TIM_SET_COMPARE(motor->htim, motor->tim_ch1, speed);

}


