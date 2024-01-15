#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "pid_const.h"

typedef float Real;

typedef struct {
    Real Kp;                 // Proportional gain
    Real Ki;                 // Integral gain
    Real Kd;                 // Derivative gain
    Real T;                  // Sampling time
    Real tau;                // Derivative lowpass constant

    Real prev_measurement;   // Previous measurement
    Real prev_error;         // Previous error

    Real proportional_term;  // Proportional term
    Real integrator_term;    // Integrator term
    Real differentiator_term;// Differentiator term

    Real out;                // Controller output
} PIDController;

// Function to initialize the PID controller
void PIDController_Init(PIDController* pid);

// Function to update the PID controller state and compute the output
float PIDController_Update(PIDController* pid, float setpoint, float measurement);

#endif  // PID_CONTROLLER_H
