#include "pid.h"

void PID_Init(PID *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->prev_error = 0;
    pid->integral = 0;
}

float PID_Update(PID *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    pid->integral += error * dt;

    float derivative = (error - pid->prev_error) / dt;

    pid->prev_error = error;

    return pid->kp*error +
           pid->ki*pid->integral +
           pid->kd*derivative;
}
