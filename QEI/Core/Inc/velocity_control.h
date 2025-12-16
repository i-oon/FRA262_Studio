/*
 * velocity_control.h
 *
 *  Created on: May 6, 2025
 *      Author: ioonz
 */

#ifndef INC_VELOCITY_CONTROL_H_
#define INC_VELOCITY_CONTROL_H_


#include "stm32g4xx_hal.h"
#include <stdlib.h>
#include <math.h>
#include <float.h>
//
typedef struct {
    float Kp;     // Proportional gain
    float Ki;     // Integral gain (1/sec)
    float Kd;     // Derivative gain (sec)
    float Ts;     // Sample time (sec)
    float Umax;   // Output upper limit

    float e_k1;   // previous error e[k-1]
    float e_k2;   // error before that e[k-2]
    float pv_k1;  // previous PV pv[k-1]
    float pv_k2;  // pv[k-2]
    float u_prev; // previous output u[k-1]
    float output;
} PID_Handle_t;


void PID_Init(PID_Handle_t *h,
              float Kp, float Ki, float Kd,
              float Ts, float Umax);


void PID_Compute(PID_Handle_t *h, float SP, float measurement);


typedef struct {
    float Kp;        // Proportional gain
    float Ki;        // Integral gain (Ti = Kp/Ki)
    float Kd;        // Derivative gain (Td = Kd/Kp)

    float prev_error;       // Previous error
    float prev_prev_error;  // Error from two steps ago
    float prev_output;      // Previous control output
    float output;
    float output_min;       // Minimum output
    float output_max;       // Maximum output
    float delta_max;        // Maximum change in output per cycle

    float T;                // Sampling period in seconds
  // Enable/disable controller
    float delta_u;
    float p, i, d;

    float error;
} VelocityPID;

int32_t VelocitySAT(float error, int32_t limits);
void VelocityPID_Init(VelocityPID* pid, float Kp, float Ki, float Kd, float T);
void VelocityPID_Update(VelocityPID* pid, float setpoint, float measurement,float max);
void VelocityPID_Reset(VelocityPID* pid);

#endif /* INC_VELOCITY_CONTROL_H_ */
