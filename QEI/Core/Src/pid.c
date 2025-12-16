/*
 * pid.c
 *
 *  Created on: May 9, 2025
 *      Author: ioonz
 */

#include "pid.h"
#include "main.h"

void set_pid(PID *pid, float kp, float ki, float kd, float max_output, float sampling){
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->sampling = sampling;
	pid->max_output = max_output;

}

void reset_pid(PID *pid){
	pid->prev_error = 0;        // Previous error
	pid->output = 0;
	pid->integral = 0;
}

void apply_pid(PID *pid, float error){

//	float error = setpoint - measurement;
	pid->integral += error*(100);
	//wind up
	if (error == 0) {
		pid->integral = 0.0;
	}

	//P + I + D
	pid->output = (pid->Kp*error) +
	            pid->Ki*(float)(pid->integral) +
	            pid->Kd*(float)(error - pid->prev_error)*pid->sampling;

	if(pid->output >= pid->max_output){
		pid->output = pid->max_output;

	}
	if(pid->output <= -pid->max_output){
		pid->output = -pid->max_output;

	}
	pid->prev_error = error;

}

