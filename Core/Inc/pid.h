#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;

    float prev_error;
    float integral;

} PID;

void PID_Init(PID *pid, float kp, float ki, float kd);

float PID_Update(PID *pid, float setpoint, float measurement, float dt);

#endif
