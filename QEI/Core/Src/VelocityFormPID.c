/*
 * VelocityFormPID.c
 *
 *  Created on: May 29, 2025
 *      Author: Kaitniyom
 */
#include "VelocityFormPID.h"
#include "stm32g4xx_hal.h"
#include "math.h"
#include "stdio.h"
#include "stdbool.h"

void InitializeVelocityFormPID(VelocityFormPIDHandler* vfPID,
		float Kp, float Ki, float Kd, float dt, float max);

void InitializePositionFormPID(VelocityFormPIDHandler* vfPID,
		float Kp, float Ki, float Kd, float dt, float max);

void ResetVelocityFormPID(VelocityFormPIDHandler* vfPID);

void ResetPositionFormPID(VelocityFormPIDHandler* vfPID);

void ComputePositionFormPID(VelocityFormPIDHandler* vfPID
		,float error, float fb/*process variable*/,float Wt);

void ComputeVelocityFormPID_B(VelocityFormPIDHandler* vfPID
		,float error, float fb/*process variable*/);

void ComputeVelocityFormPID_C(VelocityFormPIDHandler* vfPID
		,float error, float fb/*process variable*/);

void InitializeVelocityFormPID(VelocityFormPIDHandler* vfPID,
                               float Kp, float Ki, float Kd,
                               float dt, float max)
{
    // Zero all relevant arrays
    for (int i = 0; i < 3; ++i) {
        vfPID->output[i] = 0.0f;
        vfPID->feedback[i] = 0.0f;
        vfPID->error[i] = 0.0f;
    }

    vfPID->proportional_term = 0.0f;
    vfPID->integral_term     = 0.0f;
    vfPID->derivative_term   = 0.0f;

    vfPID->Kp = Kp;
    vfPID->Ki = Ki;
    vfPID->Kd = Kd;
    vfPID->dt = dt;
    vfPID->max = max;
}

void InitializePositionFormPID(VelocityFormPIDHandler* vfPID,
		float Kp, float Ki, float Kd, float dt, float max){
	// Zero all relevant arrays
	for (int i = 0; i < 3; ++i) {
		vfPID->output[i] = 0.0f;
		vfPID->feedback[i] = 0.0f;
		vfPID->error[i] = 0.0f;
	}

	vfPID->proportional_term = 0.0f;
	vfPID->integral_term     = 0.0f;
	vfPID->derivative_term   = 0.0f;

	vfPID->Kp = Kp;
	vfPID->Ki = Ki;
	vfPID->Kd = Kd;
	vfPID->dt = dt;
	vfPID->max = max;
}

void ResetVelocityFormPID(VelocityFormPIDHandler* vfPID)
{
    for (int i = 0; i < 3; ++i) {
        vfPID->output[i] = 0.0f;
        vfPID->feedback[i] = 0.0f;
        vfPID->error[i] = 0.0f;
    }

    vfPID->proportional_term = 0.0f;
    vfPID->integral_term     = 0.0f;
    vfPID->derivative_term   = 0.0f;
}

void ResetPositionFormPID(VelocityFormPIDHandler* vfPID)
{
    for (int i = 0; i < 3; ++i) {
        vfPID->output[i] = 0.0f;
        vfPID->feedback[i] = 0.0f;
        vfPID->error[i] = 0.0f;
    }

    vfPID->proportional_term = 0.0f;
    vfPID->integral_term     = 0.0f;
    vfPID->derivative_term   = 0.0f;
}

void ComputePositionFormPID(VelocityFormPIDHandler* vfPID
		,float error, float fb/*feedback*/,float Wt){

	vfPID->feedback[Current] = fb;
	vfPID->error[Current] = error;
	// Proportional
	vfPID->proportional_term = vfPID->Kp * error;
	// Integral
//	float wind_up = (vfPID->feedback[Current] - vfPID->output[Current]) / Wt;



	if (vfPID->output[Current] > vfPID->max || vfPID->output[Current] < -vfPID->max) {
		vfPID->integral_term += 0.0f;
	}
	else{
		vfPID->integral_term += error * vfPID->dt;
	}
	//Derivative
	float derivative = (vfPID->error[Current] - vfPID->error[Previous]) / vfPID->dt;


	vfPID->derivative_term   = vfPID->Kd * derivative;
	//Output
	vfPID->output[Current] = vfPID->proportional_term
						   + vfPID->Ki * vfPID->integral_term
						   + vfPID->derivative_term;
	//Clamp output
	if (vfPID->output[Current] > vfPID->max)
		vfPID->output[Current] = vfPID->max;
	else if (vfPID->output[Current] < -vfPID->max)
		vfPID->output[Current] = -vfPID->max;

	// Update history
	vfPID->output[Previous]   = vfPID->output[Current];
	vfPID->error[Previous]    = vfPID->error[Current];
	vfPID->feedback[Previous] = vfPID->feedback[Current];
}

void ComputeVelocityFormPID_B(VelocityFormPIDHandler* vfPID
		,float error, float fb/*feedback*/){

	vfPID->feedback[Current] = fb;
	vfPID->error[Current] = error;

	vfPID->proportional_term = vfPID->Kp * (vfPID->error[Current] - vfPID->error[Previous]);
	vfPID->integral_term     = vfPID->Ki * vfPID->error[Current] * vfPID->dt;
	vfPID->derivative_term   = vfPID->Kd * (
		(vfPID->feedback[Current] - 2.0f * vfPID->feedback[Previous] + vfPID->feedback[Prior]) / vfPID->dt
	);

	vfPID->output[Current] = vfPID->output[Previous]
						   + vfPID->proportional_term
						   + vfPID->integral_term
						   - vfPID->derivative_term;

	if (vfPID->output[Current] > vfPID->max) {
		vfPID->output[Current] = vfPID->max;
	}
	else if (vfPID->output[Current] < -vfPID->max) {
		vfPID->output[Current] = -vfPID->max;
	}

	vfPID->feedback[Prior]    = vfPID->feedback[Previous];
	vfPID->feedback[Previous] = vfPID->feedback[Current];

	vfPID->error[Prior]       = vfPID->error[Previous];
	vfPID->error[Previous]    = vfPID->error[Current];

	vfPID->output[Previous]   = vfPID->output[Current];
}

void ComputeVelocityFormPID_C(VelocityFormPIDHandler* vfPID, float error, float fb /*process variable*/) {
    vfPID->feedback[Current] = fb;
    vfPID->error[Current] = error;

    // Proportional term: based on change in feedback (velocity form)
    vfPID->proportional_term = vfPID->Kp * (vfPID->feedback[Current] - vfPID->feedback[Previous]);

    // Derivative term: second-order difference (velocity form)
    vfPID->derivative_term = vfPID->Kd * (
        (vfPID->feedback[Current] - 2.0f * vfPID->feedback[Previous] + vfPID->feedback[Prior]) / vfPID->dt
    );

    // Tentative integral term
    float integral_candidate = vfPID->Ki * vfPID->error[Current] * vfPID->dt;

    // Predict output before clamping
    float temp_output = vfPID->output[Previous]
                        - vfPID->proportional_term
                        + integral_candidate
                        - vfPID->derivative_term;

    // Apply anti-windup by conditionally accepting the integral term
    if (temp_output > vfPID->max || temp_output < -vfPID->max) {
        vfPID->integral_term = 0.0f;
    } else {
        vfPID->integral_term = integral_candidate;
    }

    // Compute final output
    vfPID->output[Current] = vfPID->output[Previous]
                             - vfPID->proportional_term
                             + vfPID->integral_term
                             - vfPID->derivative_term;

    // Clamp output to max/min
    if (vfPID->output[Current] > vfPID->max) {
        vfPID->output[Current] = vfPID->max;
    } else if (vfPID->output[Current] < -vfPID->max) {
        vfPID->output[Current] = -vfPID->max;
    }

    // Shift state for next iteration
    vfPID->feedback[Prior]    = vfPID->feedback[Previous];
    vfPID->feedback[Previous] = vfPID->feedback[Current];

    vfPID->error[Previous]    = vfPID->error[Current];
    vfPID->output[Previous]   = vfPID->output[Current];
}


//void ComputeVelocityFormPID_C(VelocityFormPIDHandler* vfPID
//		,float error, float fb/*process variable*/){
//
//	vfPID->feedback[Current] = fb;
//	vfPID->error[Current] = error;
//
//	vfPID->proportional_term = vfPID->Kp * (vfPID->feedback[Current] - vfPID->feedback[Previous]);
//
//	if(!(vfPID->output[Current] > vfPID->max || vfPID->output[Current] < -vfPID->max)){
//		vfPID->integral_term     = vfPID->Ki * vfPID->error[Current] * vfPID->dt;
//	}
//	else{
//		vfPID->integral_term = 0;
//	}
//
//	vfPID->derivative_term   = vfPID->Kd * (
//		(vfPID->feedback[Current] - 2.0f * vfPID->feedback[Previous] + vfPID->feedback[Prior]) / vfPID->dt
//	);
//
//	vfPID->output[Current] = vfPID->output[Previous]
//						   - vfPID->proportional_term
//						   + vfPID->integral_term
//						   - vfPID->derivative_term;
//
//	if (vfPID->output[Current] > vfPID->max) {
//		vfPID->output[Current] = vfPID->max;
//	}
//	else if (vfPID->output[Current] < -vfPID->max) {
//		vfPID->output[Current] = -vfPID->max;
//	}
//
//	vfPID->feedback[Prior]    = vfPID->feedback[Previous];
//	vfPID->feedback[Previous] = vfPID->feedback[Current];
//
//	vfPID->error[Previous]    = vfPID->error[Current];
//
//	vfPID->output[Previous]   = vfPID->output[Current];
//}
