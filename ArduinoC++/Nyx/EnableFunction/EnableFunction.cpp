/*

  EnableFunction.cpp
  Autor: Bruno Sorban, Kaleb Ramos, Lucas Wu, Mateus Stano
  Fevereiro de 2023
 
*/

#include "EnableFunction.h"

EnableFunction::EnableFunction(double minAltitude, double apogeeAltitude) {
    this->minAltitude = minAltitude;
    this->apogeeAltitude = apogeeAltitude;
    for(int i = 0; i < enableLenght; i++) {
        Alt[i] = 0.0;
        Acc[i] = 0.0;
    }
    meanAcc = 0.0;
    meanAlt = 0.0;
    pastMeanAlt = 0.0;
    altitudeDifference = 0.0;
    counter = 0;
    motorOn = false;
    controlOn = false;
    apogeeAchieved = false;
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
    if(meanAcc > 0 && motorOn == false) { //aceleracao positiva indica motor ligado
    counter++;
        if(counter >= minCounters) {
            motorOn = true;
            counter = 0;
        }
    }
    if(meanAcc < 0 || meanAlt >= minAltitude) { //aceleracao negativa indica motor desligado (chegou na altitude de burnout)
        motorOn = false;
        counter++;
        if(counter >= minCounters && apogeeAchieved == false) { //o controle so fica ligado antes de atingir o apogeu
            controlOn = true;
        }
    }
    if(meanAlt >= apogeeAltitude || altitudeDifference < 0) { //atingiu o apogeu e desliga o controle
        apogeeAchieved == true;
        controlOn = false;
    }
}

bool EnableFunction::getControlOn() {
    return controlOn;
}

bool EnableFunction::getApogeeAchieved() {
    return apogeeAchieved;
}
