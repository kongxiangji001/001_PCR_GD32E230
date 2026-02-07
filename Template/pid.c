/* pid.c - simple PID controller implementation */
#include "pid.h"

void pid_init(PIDController *pid, float Kp, float Ki, float Kd, float out_min, float out_max, float dt)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float pid_compute(PIDController *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;
    /* proportional */
    float P = pid->Kp * error;
    /* integral */
    pid->integrator += 0.5f * pid->Ki * pid->dt * (error + pid->prev_error);
    /* anti-windup: clamp integrator based on output limits */
    if (pid->integrator > pid->out_max) pid->integrator = pid->out_max;
    if (pid->integrator < pid->out_min) pid->integrator = pid->out_min;
    /* derivative (band-limited via simple backward difference) */
    float D = 0.0f;
    if (pid->dt > 0.0f) D = pid->Kd * (error - pid->prev_error) / pid->dt;

    float out = P + pid->integrator + D;

    /* clamp output */
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    pid->prev_error = error;
    return out;
}
