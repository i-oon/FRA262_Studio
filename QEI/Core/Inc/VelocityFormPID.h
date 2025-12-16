/*
 * VelocityFormPID.h
 *
 *  Created on: May 29, 2025
 *      Author: Kaitniyom
 */

#ifndef INC_VELOCITYFORMPID_H_
#define INC_VELOCITYFORMPID_H_

#include "TrajectoryGenerator.h"

typedef struct{
	float output[2];
	float feedback[3];
	float error[3];

	float proportional_term;
	float integral_term;
	float derivative_term;

	float Kp,Ki,Kd;
	float dt;

	float max;

}VelocityFormPIDHandler;

void InitializeVelocityFormPID(VelocityFormPIDHandler* vfPID,
		float Kp, float Ki, float Kd, float dt, float max);

void InitializePositionFormPID(VelocityFormPIDHandler* vfPID,
		float Kp, float Ki, float Kd, float dt, float max);

void ResetVelocityFormPID(VelocityFormPIDHandler* vfPID);

void ResetPositionFormPID(VelocityFormPIDHandler* vfPID);

void ComputePositionFormPID(VelocityFormPIDHandler* vfPID
		,float error, float fb/*process variable*/, float Wt);

void ComputeVelocityFormPID_B(VelocityFormPIDHandler* vfPID
		,float error, float fb/*process variable*/);

void ComputeVelocityFormPID_C(VelocityFormPIDHandler* vfPID
		,float error, float fb/*process variable*/);

#endif /* INC_VELOCITYFORMPID_H_ */
