/*
 * velocity_control.c
 *
 *  Created on: May 6, 2025
 *      Author: ioonz
 */

/* velocity_pid.c */
#include "velocity_control.h"
#include "main.h"
//#include "main.h"
//#include "arm_math.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

//void PID_Init(PID_Handle_t *h,
//              float Kp, float Ki, float Kd,
//              float Ts, float Umax)
//{
//    h->Kp     = Kp;
//    h->Ki     = Ki;
//    h->Kd     = Kd;
//    h->Ts     = Ts;
//    h->Umax   = Umax;
//
//    h->e_k1   = 0.0f;
//    h->e_k2   = 0.0f;
//    h->pv_k1  = 0.0f;
//    h->pv_k2  = 0.0f;
//    h->u_prev = 0.0f;
//    h->output = 0.0f;
//}


void VelocityPID_Init(VelocityPID* pid, float Kp, float Ki, float Kd, float T) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->T = T;

    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;

    pid->output = 0.0f;
    pid->prev_output = 0.0f;

    pid->output_min = -FLT_MAX;
    pid->output_max = FLT_MAX;
    pid->delta_max = FLT_MAX;
    pid->delta_u = 0;

    pid->p = 0; pid->i =0; pid->d = 0;
}

//int32_t VelocitySAT(float error, int32_t limits) {
//	if (error > limits) return limits;
//	else if (error < -limits) return -limits;
//	return limits;
//}

void VelocityPID_Update(VelocityPID* pid, float setpoint, float measurement,float max)  {

//	float error = setpoint - measurement;
//	if (!(pid->output >= pid->output_max && error > 0) || (pid->output <= -pid->output_max && error < 0)) {
//		pid->output += ((pid->Kp+pid->Ki+pid->Kd)*error)
//						- ((pid->Kp+(2*pid->Kd))*pid->prev_error)
//						+ (pid->Kd * pid->prev_prev_error);
//	}
//	pid->prev_prev_error = pid->prev_error;
//	pid->prev_error = error;
//	if(pid->output > max){
//		pid->output = max;
//	}
//	if(pid->output < -12){
//			pid->output = -12;
//		}
//	return pid->output;

	if(fabs(pid->delta_u) < 1e-6){
		pid->prev_prev_error = 0;
		pid->prev_error = 0;
		pid->prev_output = 0;
		pid->delta_u = 0;
		pid->p = 0; pid->i =0; pid->d = 0;
		pid->output = 0;
	}

    pid->error = setpoint - measurement;
//    if (isnan(error) || isinf(error)) {
//    		return pid->prev_output;
//    	}
    pid->p = pid->Kp * (pid->error - pid->prev_error);
    if (pid->Ki != 0.0f) {
        pid->i = (pid->Kp * pid->T / pid->Ki) * pid->error;
    } else {
        pid->i = 0.0f;  // or handle as error
    }

    pid->d = (pid->Kp * pid->Kd / pid->T) *
    		(pid->error - 2.0f * pid->prev_error + pid->prev_prev_error);

    pid->delta_u = pid->p + pid->i + pid->d;

    if (pid->delta_u > pid->delta_max) {
    	pid->delta_u = pid->delta_max;
    } else if (pid->delta_u < -pid->delta_max) {
    	pid->delta_u = -pid->delta_max;
    }

    pid->output = pid->prev_output + pid->delta_u;

    if (pid->output > max) {
    	pid->output = max;
    } else if (pid->output < -max) {
    	pid->output = -max;
    }

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = pid->error;
    pid->prev_output = pid->output;
//
//
//    return output;
//    if (output >= 4.8) { return 4.8;}
//    else if (output < -4.8) {return -4.8;}
//    else {return output;}
//
//    if (output >= 80.0) { return 80.0;}
//        else if (output < -80.0) {return -80.0;}
//        else {return output;}

}

void VelocityPID_Reset(VelocityPID* pid){
	pid->prev_error = 0;
	pid->output = 0;
	pid->prev_prev_error = 0;
	pid->prev_output = 0.0;
	pid->p = 0; pid->i =0; pid->d = 0;
	pid->delta_u = 0;
}


//void PID_Compute(PID_Handle_t *h, float setpoint, float measurement)
//{
//    // current error
//    float e_k = setpoint - measurement;
//
//    // Proportional term on PV (Type C)
//    float dP = -h->Kp * (measurement - h->pv_k1);
//
//    // Integral term on error
//    float dI = h->Ki * e_k * h->Ts;
//
//    // Derivative term on PV
//    float ddPV = (measurement - 2.0f*h->pv_k1 + h->pv_k2);
//    float dD   = -h->Kd * (ddPV / h->Ts);
//
//    // velocity-form update
//    float u_k = h->u_prev + dP + dI + dD;
//
//    // anti-reset windup (clamp)
//    if (u_k > h->Umax) u_k = h->Umax;
//    if (u_k < -(h->Umax)) u_k = -h->Umax;
//
//    // shift histories
//    h->pv_k2   = h->pv_k1;
//    h->pv_k1   = measurement;
//    h->e_k2    = h->e_k1;
//    h->e_k1    = e_k;
//    h->u_prev  = u_k;
//
//    h->output = u_k;
//}

