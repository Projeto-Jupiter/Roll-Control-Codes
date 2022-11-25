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
