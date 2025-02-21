/*
 * pid.h
 *
 *  Created on: Feb 13, 2025
 *      Author: user
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32f4xx_hal.h"
typedef struct {
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    float setpoint;     // Desired setpoint
    float integral;     // Integral term
    float prev_output;
    float prev_error;   // Previous error for derivative calculation
    float output_min;   // Minimum output value
    float output_max;   // Maximum output value
    uint32_t prevtick;
} PIDController;

// Initialize PID controller
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, float setpoint, float output_min, float output_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->prevtick=0;
}

// Update PID controller
float PID_Update(PIDController *pid, float measured_value) {
	if(pid->prevtick==0)
		pid->prevtick=HAL_GetTick();

	float dt  = (HAL_GetTick() - pid->prevtick) *0.001f;
    float error = pid->setpoint - measured_value;
    float proportional = pid->Kp * error;

    // Integral term with anti-windup
    pid->integral += pid->Ki * error * dt;

    float derivative = pid->Kd * (error - pid->prev_error) / dt;

    float output = proportional + pid->integral + derivative;

    // Apply output limits
    if (output > pid->output_max) {
        output = pid->output_max;
        // Anti-windup: Limit integral term
        pid->integral -= pid->Ki * error * dt;
    } else if (output < pid->output_min) {
        output = pid->output_min;
        // Anti-windup: Limit integral term
        pid->integral -= pid->Ki * error * dt;
    }

    pid->prev_error = error;
    pid->prevtick =HAL_GetTick();
    return output;
}

void PID_Reset(PIDController* pid) {
	pid->setpoint=0;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prevtick = 0;//HAL_GetTick();
}
#endif /* INC_PID_H_ */
