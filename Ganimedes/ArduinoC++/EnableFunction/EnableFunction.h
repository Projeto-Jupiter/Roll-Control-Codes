/*

  EnableFunction.h
  Autor: Bruno Sorban, Kaleb Ramos, Lucas Wu, Mateus Stano
  Fevereiro de 2023
 
*/

#ifndef ENABLEFUNCTION_H
#define ENABLEFUNCTION_H

const int enableLenght = 100;

class EnableFunction {
public:
    EnableFunction(double altMotorOff, float minAltitude);
    ~EnableFunction();
    void addValues(double acceleration, double measuredAltitude);
    bool getControlOn();
    bool getMotorOn();
  
private:
    double altMotorOff;
    float minAltitude;
    double meanAcc;
    double meanAlt;
    double pastMeanAlt;
    double altitudeDifference;
    double Acc[enableLenght];
    double Alt[enableLenght];
    int counter;
    bool motorOn;
    bool controlOn;
};

#endif