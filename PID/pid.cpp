typedef struct {
    // Controller gains
    float Kp;    // Proportional gain
    float Ki;    // Integral gain
    float Kd;    // Derivative gain

    // Low-pass filter derivative time constant
    float tau;

    // Controller memory
    float integrator;        // Accumulates the integral of the error
    float prevError;        // Previous error for integral calculation
    float differentiator;   // Stores the derivative term
    float prevMeasurement;   // Previous measurement for derivative calculation

    // Output limits
    float limMin;           // Minimum output limit
    float limMax;           // Maximum output limit

    // Sample time (in seconds)
    float T;

    // Controller output
    float out;              // Current output of the PID controller

} PIDController;

// Initializes the PID controller by resetting its state variables.
// This function can be extended in the future to accept gains as parameters.
void PIDController_Init(PIDController *pid) {
    // Clear the controller variables
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

// Updates the PID controller and computes the output.
// Takes a PID controller struct, setpoint (desired value), 
// and measurement (actual value) as inputs, returning the computed output.
float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    // Calculate error signal
    float error = setpoint - measurement;

    // Calculate proportional term: P[n] = Kp * e[n]
    float proportional = pid->Kp * error;

    // Calculate integral term
    // Current integrator term depends on the previous value and the current error
    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    // Anti-windup: Calculate dynamic limits for the integrator
    float limMinInt, limMaxInt;

    // Compute the integrator limits based on output limits
    limMaxInt = (pid->limMax > proportional) ? (pid->limMax - proportional) : 0.0f;
    limMinInt = (pid->limMin < proportional) ? (pid->limMin - proportional) : 0.0f;

    // Clamp the integrator to prevent windup
    if (pid->integrator > limMaxInt) {
        pid->integrator = limMaxInt;
    } else if (pid->integrator < limMinInt) {
        pid->integrator = limMinInt;
    }

    // Calculate the derivative term using a low-pass filter (band-limited differentiator)
    pid->differentiator = (2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                          + (2.0f * pid->tau - pid->T) * pid->differentiator)
                          / (2.0f * pid->tau + pid->T);

    // Compute output and apply output limits
    pid->out = proportional + pid->integrator + pid->differentiator;

    // Clamp the output to the specified limits
    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    // Store the current error and measurement for the next update
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    // Return the computed PID output
    return pid->out;
}