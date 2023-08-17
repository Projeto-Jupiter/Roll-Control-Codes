/*

  EnableFunction.cpp
  Autor: Bruno Sorban, Kaleb Ramos, Lucas Wu, Mateus Stano
  Fevereiro de 2023
 
*/

#include "EnableFunction.h"

EnableFunction::EnableFunction(double altMotorOff, float minAltitude) {
    this->altMotorOff = altMotorOff;
    this->minAltitude = minAltitude;
    for(int i = 0; i < enableLenght; i++) {
        Alt[i] = 0;
        Acc[i] = 0;
    }
    meanAcc = 0.0;
    meanAlt = 0.0;
    pastMeanAlt = 0.0;
    altitudeDifference = 0.0;
    counter = 0;
    motorOn = false;
    controlOn = false;
}

EnableFunction::~EnableFunction() {

}

void EnableFunction::addValues(double acceleration, double measuredAltitude) {
    //calculo das medias moveis
    meanAcc = meanAcc - (Acc[0] - acceleration)/enableLenght;
    for(int i = 0; i < enableLenght - 1; i++) {
        Acc[i] = Acc[i + 1];
    }
    Acc[enableLenght - 1] = acceleration;

    pastMeanAlt = meanAlt;
    meanAlt = meanAlt - (Alt[0] - measuredAltitude)/enableLenght;
    for(int i = 0; i < enableLenght - 1; i++) {
        Alt[i] = Alt[i + 1];
    }
    Alt[enableLenght - 1] = measuredAltitude;

    altitudeDifference = meanAlt - pastMeanAlt; //calcula diferenca de altitude para ver se a velocidade e positiva ou negativa

    //condicoes atuais
    if((meanAcc > 0 && motorOn == false) || meanAlt >= minAltitude) { //aceleracao positiva indica motor ligado, minAltitude por redundância
        motorOn = true;
    }
    if((meanAcc < 0 && motorOn == true) || meanAlt >= altMotorOff) { //aceleracao negativa indica motor desligado (chegou na altitude de burnout)
        motorOn = false;
        controlOn = true;
    }
}

bool EnableFunction::getControlOn() {
    return controlOn;
}

bool EnableFunction::getMotorOn() {
    return motorOn;
}