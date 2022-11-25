/*

  EnableFunction.cpp
  Autor: Bruno Sorban, Kaleb Ramos, Lucas Wu, Mateus Stano
  Novembro de 2022
 
*/

#include "Arduino.h"
#include "EnableFunction.h"

EnableFunction::EnableFunction(){
  for (int i = 0; i < 5; i++){
    v[i] = 0.0;
  }

  mean = 0.0;
  counter = 0;
  motorOn = 0;
}

double EnableFunction::addAcceleration(double az) {
  mean = mean - (v[0] - az) / 5;
  v[0] = v[1];
  v[1] = v[2];
  v[2] = v[3];
  v[3] = v[4];
  v[4] = az;

  // esperando motor ligar
  if (mean > (10.0) && motorOn == 0) // se a media for superior a 2g (significa que o motor ligou) - motor e gs tem sentidos diferentes
    {                                                      // && motor desligado (garante primeiro estado)
      counter++;      

      if (counter >= 50) { // considera uma margem de meio segundo
        motorOn = 1;     // liga o motor
        counter = 0;     // reseta o counter para reciclar variaveis
      }
    }
      
    // motor ligado
  if (motorOn == 1) {
    counter++; // considera counter por meio segundo para evitar erros no momento que o foguete liga  
    if (counter >= 50){
      if (mean >= 0.0) { // se a aceleracao esta para cima (menor que 1g) controle fica desligado
        return 0; // controle desligado;
      }
      else { // quando a aceleracao fica igual a 1g (motor desligou de novo) controle liga
        return 1; // controle ligado;
      }
    }
  }

  else {
    counter = 0;
  }
}
