/* pid.h - simple PID controller */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float dt; /* sample time in seconds */
    float integrator;
    float prev_error;
    float out_min;
    float out_max;
} PIDController;

void pid_init(PIDController *pid, float Kp, float Ki, float Kd, float out_min, float out_max, float dt);
float pid_compute(PIDController *pid, float setpoint, float measurement);

#endif /* __PID_H */
