#pragma once

typedef struct {
    float kp;
    float ki;
    float kd;
    float previous_error;
    float integral;
    float output_min;
    float output_max;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float min_output, float max_output);
float pid_compute(pid_controller_t *pid, float setpoint, float measured_value);
