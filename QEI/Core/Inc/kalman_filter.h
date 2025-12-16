/*
 * kalman_filter.h
 *
 *  Created on: May 12, 2025
 *      Author: User
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

#include "arm_math.h"
#include "main.h"

// External Kalman variables
typedef struct{
	float32_t theta_kalman; //position
	float32_t omega_kalman; //
	float32_t torque_kalman;
	float32_t current_kalman;
	////phase lead
	float32_t theta_kalman_lead;
	float32_t theta_kalman_prev;
	float32_t theta_lead_prev;
	////
	float U_f32[1];
	float Xsensor_f32[1];
}kalman;


float PhaseLead(kalman* KALMAN, float phase_deg);
//arm_matrix_instance_f32 Xsensor;
//arm_matrix_instance_f32 Xsensor;

// Function to initialize matrices
void Kalman_Init(kalman* KALMAN);

// Kalman Filter calculation function
void Kalman_Filter(kalman* KALMAN , float voltage, float encoder);


#endif /* INC_KALMAN_FILTER_H_ */
