/*

  LowPass.h
  Low pass chebyshev filter order=2 alpha1=0.445
  Autor: Bruno Sorban, Kaleb Ramos, Lucas Wu, Mateus Stano
  Novembro de 2022
 
*/

#ifndef LOWPASS.H
#define LOWPASS.H

#include "Arduino.h"

class LowPass {
  public:
    LowPass();
    ~LowPass();
    
    double addData(double x);
  private:
    double data[42];
};

#endif
