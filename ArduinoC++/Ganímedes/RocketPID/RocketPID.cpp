/*

  RocketPID.cpp
  PID controller for High-Power Rockets. (Direct mode)
  Autor: Kaleb Ramos, Lucas Wu, Mateus Stano
  1st version: 21/02/2023
 
*/

#include "RocketPID.h"
#include "Arduino.h"

RocketPID::RocketPID(int setPoint, float Kp, float Ki, float Kd){
    this->setPoint = setPoint;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    dt = 0.01;
    lastInput = 0.0;
    lastErr = 0.0;
}

RocketPID::~RocketPID(){
}


float RocketPID::computePID(double input){
    //Compute error terms
    err = setPoint - input;
    d_input = input - lastInput;
    d_error = err - lastErr; 

    //Compute the proportional term
    proportional = Kp * err;

    //Compute integrative and derivative terms
    integrative += Ki * err * dt;
    integrative = clamp(integrative) * 180/3.1416;  //Avoid integrative windup

    derivative = -Kd * d_input / dt;

    //Compute final output
    output = proportional + integrative + derivative;
    output = clamp(output);

    //Keep track of state
    lastInput = input;
    lastErr = err;
return output;
}

void RocketPID::setLimits(float lowerInput, float upperInput){
    lower = lowerInput * 3.1416/180;
    upper = upperInput * 3.1416/180;
}

void RocketPID::setDt(float dtInput){
    dt = dtInput;
}

float RocketPID::clamp(float value){
    if (value > upper)
        return upper;
    if (value < lower)
        return lower;
return value;
}

float RocketPID::getSetPoint(){
    return setPoint;
}

float RocketPID::getDt(){
    return dt;
}

float RocketPID::getProportional(){
    return proportional;
}

float RocketPID::getIntegrative(){
    return integrative;
}

float RocketPID::getDerivative(){
    return derivative;
}
