/*
 * kalman_prs.h
 *
 *  Created on: May 19, 2025
 *      Author: gunda
 */

#ifndef INC_KALMAN_PRS_H_
#define INC_KALMAN_PRS_H_

#include "arm_math.h"

// External Kalman variables
typedef struct{
	float32_t theta_kalman_prs; //position
	float32_t omega_kalman_prs; //
	float32_t torque_kalman_prs;
	float32_t current_kalman_prs;
	float U_f32_prs[1];
	float Xsensor_f32_prs[1];
}kalman_prs;



//arm_matrix_instance_f32 Xsensor;
//arm_matrix_instance_f32 Xsensor;

// Function to initialize matrices
void Kalman_Init_prs(kalman_prs* KALMAN);

// Kalman Filter calculation function
void Kalman_Filter_prs(kalman_prs* KALMAN , float voltage, float encoder);


#endif /* INC_KALMAN_PRS_H_ */
