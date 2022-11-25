/*

  EnableFunction.h
  Autor: Bruno Sorban, Kaleb Ramos, Lucas Wu, Mateus Stano
  Novembro de 2022
 
*/

#ifndef EnableFunction.h
#define EnableFunction.h

class EnableFunction(){
  public:
    double addAcceleration(double az);
    
  private:
    double v[5];
    double mean;
    int counter;
    int motorOn; 
}

#endif
