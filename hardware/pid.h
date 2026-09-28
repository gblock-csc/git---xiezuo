#ifndef __PID_H
#define __PID_H

#include "main.h"

#define PID_MODE_POSITION   0
#define PID_MODE_INCREMENT  1

typedef struct
{
    float kp;
    float ki;
    float kd;
    float target;
    float current;
    float error;
    float last_error;
    float integral;
    float integral_limit;
    float output_limit;
    float output;
    float d_previous;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit);
float PID_Calculate(PID_TypeDef *pid, float target, float current);
float PID_Calculate_Increment(PID_TypeDef *pid, float target, float current);
void PID_Reset(PID_TypeDef *pid);
void PID_SetParams(PID_TypeDef *pid, float kp, float ki, float kd);

#endif
