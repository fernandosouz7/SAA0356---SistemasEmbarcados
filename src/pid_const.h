#ifndef PID_CONSTANTS_H
#define PID_CONSTANTS_H

// ------------------------- PID GAINS --------------------------
#define PROPORTIONAL_GAIN   0.0f
#define INTEGRAL_GAIN       0.15721f
#define DERIVATIVE_GAIN     0.0f
// ---------------------------------------------------------------

#define SAMPLING_FREQUENCY            1000
#define DERIVATIVE_LOWPASS_CONSTANT   10

// ---------------------------------------------------------------
// Safety boundaries
#define MAX_INTEGRATOR_LIMIT   42.0f
#define MIN_INTEGRATOR_LIMIT  -42.0f
#define MAX_OUTPUT             42.0f
#define MIN_OUTPUT            -42.0f  // Fixed typo and assuming negative limit is intended
// ---------------------------------------------------------------

#endif  // PID_CONSTANTS_H
