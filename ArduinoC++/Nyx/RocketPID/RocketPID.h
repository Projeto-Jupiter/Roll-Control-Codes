/*

  RocketPID.h
  PID controller for High-Power Rockets. (Direct mode)
  Autor: Kaleb Ramos, Lucas Wu, Mateus Stano
  1st version: 21/02/2023
 
*/

#ifndef ROCKETPID_H
#define ROCKETPID_H

#include "Arduino.h"

class RocketPID{
  public:
    RocketPID(int setPoint, float Kp, float Ki, float Kd);
    ~RocketPID();

    float computePID(double input);
    void setLimits(float lowerInput, float upperInput);
    void setDt(float dtInput);
    float clamp(float value);
    float getSetPoint();
    float getDt();
    float getKp();
    float getKi();
    float getKd();
  private:
    int setPoint;
    float dt;
    float Kp, Ki, Kd;
    float proportional, integrative, derivative;
    float lastInput, lastErr;
    float err, d_input, d_error, output;
    float lower, upper;
};

#endif
