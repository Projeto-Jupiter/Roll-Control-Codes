/*

  LowPass.cpp
  Low pass chebyshev filter order=2 alpha1=0.445
  Autor: Bruno Sorban, Kaleb Ramos, Lucas Wu, Mateus Stano
  Novembro de 2022
 
*/

#include "Arduino.h"
#include "LowPass.h"

LowPass::LowPass(){
  for (int i = 0; i < 42; i++){
    data[i] = 0.0;
  }
}

double LowPass::addData(double x){
  for (int i = 0; i < 41; i++){
    data[i] = data[i+1];
  }
  data[41] = x

  return (0.000730542097152697 * data[0]   -0.000819759833501920 * data[1]   -0.00166082645353546 * data[2]   
        +1.99328430876226e-18 * data[3]    +0.00300608142740516 * data[4]    +0.00251888639362088 * data[5]    
        -0.00337138525783167 * data[6]    -0.00718539449856550 * data[7]    +5.99889752552229e-18 * data[8]    
        +0.0119162828308361 * data[9]    +0.00931337264805999 * data[10]    -0.0116804690875208 * data[11]    
        -0.0235865615284306 * data[12]    +1.13604360012867e-17 * data[13]    +0.0368157010550922 * data[14]    
        +0.0287966766635483 * data[15]    -0.0372274978187327 * data[16]    -0.0809518950838493 * data[17]    
        +1.50810312779462e-17 * data[18]    +0.199488764925137 * data[19]    +0.373897481521116 * data[20]    
        +0.373897481521116 * data[21]    +0.199488764925137 * data[22]    +1.50810312779462e-17 * data[23]    
        -0.0809518950838493 * data[24]    -0.0372274978187327 * data[25]    +0.0287966766635483 * data[26]    
        +0.0368157010550922 * data[27]    +1.13604360012867e-17 * data[28]    -0.0235865615284306 * data[29]    
        -0.0116804690875208 * data[30]    +0.00931337264805999 * data[31]    +0.0119162828308361 * data[32]    
        +5.99889752552229e-18 * data[33]    -0.00718539449856550 * data[34]    -0.00337138525783167 * data[35]    
        +0.00251888639362088 * data[36]    +0.00300608142740516 * data[37]    +1.99328430876226e-18 * data[38]    
        -0.00166082645353546 * data[39]    -0.000819759833501920 * data[40]    +0.000730542097152697 * data[41]);
}