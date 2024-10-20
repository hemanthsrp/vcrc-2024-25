typedef struct{
    //controller gains
        float Kp;
        float Ki;
        float Kd;

    //Low pass filter derivative time constant
        float tau;

    //controler memory 
        float integrator;
        float prevError;   //required for integrator 
        float differentiator;
        float prevMeasurement;  //required for differentiator 

    //Output limits
        float limMin;
        float limMax;

    //Sample time (in seconds) 
    float T;

    //Controller output
    float out;



} PIDController;



//initialization, basically to reset the controller, could also use this function to pass in the gains and so forth, but not there yet
void PIDController_Init(PIDController *pid) {
//Clear the controller variables 

pid->integrator = 0.0f;
pid->prevError = 0.0f;

pid->differentiator = 0.0f;
pid->prevMeasurement = 0.0f;

pid->out = 0.0f;

}



//Updates the function and computes the controller output
//Takes the PID controller struct, setpoint value or the reference, and it also takes the measurement (which we get via feeback from the PIDs output or the systems output) and returns a float which is the controller output
float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

//error signal 
float error = setpoint - measurement;

//proportional from the equation p[n] = Kp*e[n] in notebook 
float proportional = pid->Kp * error;

//integral 
//Curent integrator term depends on the previous one plus the previous error from the previous integration, so the end of this function rule will store the error term in pid previous error 
pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error  + pid->prevError);

//Anti-windup (dynamic integrator clamping) it's essentially just figuring out the limits that we need to set on the integrator (it's not to saturate the output and make it overdrive)
float limMinInt, limMaxInt;

//Compute the integrator limits
if (pid->limMax > proportional) {
    limMaxInt = pid->limMax - proportional;

} else {

    limMaxInt = 0.0f;

}

if(pid->limMin < proportional) {

    limMinInt = pid->limMin - proportional;

} else {

    limMinInt = 0.0f;

}

//Now we have to actually clamp the integrator, the previous part above was actually used to figure out the limits we need to set on the integrator, this part implements it, so it will limit the integrator value








}