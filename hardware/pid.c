#include "pid.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->d_previous = 0.0f;
}

float PID_Calculate(PID_TypeDef *pid, float target, float current)
{
    pid->target = target;
    pid->current = current;
    pid->error = target - current;

    pid->integral += pid->error;

    if (pid->integral_limit > 0.0f)
    {
        if (pid->integral > pid->integral_limit)
            pid->integral = pid->integral_limit;
        else if (pid->integral < -pid->integral_limit)
            pid->integral = -pid->integral_limit;
    }

    pid->output = pid->kp * pid->error
                 + pid->ki * pid->integral
                 + pid->kd * (pid->error - pid->last_error);

    if (pid->output_limit > 0.0f)
    {
        if (pid->output > pid->output_limit)
            pid->output = pid->output_limit;
        else if (pid->output < -pid->output_limit)
            pid->output = -pid->output_limit;
    }

    pid->last_error = pid->error;

    return pid->output;
}

float PID_Calculate_Increment(PID_TypeDef *pid, float target, float current)
{
    float increment;

    pid->target = target;
    pid->current = current;
    pid->error = target - current;

    increment = pid->kp * (pid->error - pid->last_error)
              + pid->ki * pid->error
              + pid->kd * (pid->error - 2.0f * pid->last_error + pid->d_previous);

    pid->d_previous = pid->last_error;
    pid->last_error = pid->error;

    pid->output += increment;

    if (pid->output_limit > 0.0f)
    {
        if (pid->output > pid->output_limit)
            pid->output = pid->output_limit;
        else if (pid->output < -pid->output_limit)
            pid->output = -pid->output_limit;
    }

    return pid->output;
}

void PID_Reset(PID_TypeDef *pid)
{
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->d_previous = 0.0f;
}

void PID_SetParams(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
