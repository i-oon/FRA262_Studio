/*
 * pid.h
 *
 *  Created on: May 9, 2025
 *      Author: ioonz
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	float Kp;              // Proportional gain constant
	float Ki;              // Integral gain constant
	float Kd;              // Derivative gain constant
	float sampling;
	float max_output;
	float prev_error;        // Previous error
	float output;
	float integral;
}PID;

void set_pid(PID *pid, float kp, float ki, float kd, float max_output, float sampling);
void reset_pid(PID *pid);
void apply_pid(PID *pid, float error);
void PID_position(PID *pid, float kp, float ki, float kd, float setpoint, float measurement);
void PID_velocity(PID *pid, float kp, float ki, float kd,  float setpoint, float measurement);

#endif /* INC_PID_H_ */
