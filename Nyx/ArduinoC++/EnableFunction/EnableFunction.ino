/*

  EnableFunction
  Autor: Bruno Sorban, Kaleb Ramos, Lucas Wu, Mateus Stano
  Fevereiro de 2023
 
*/

#include "EnableFunction.h"
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>

//Giroscopios
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);
int16_t ax1_Raw, ay1_Raw, az1_Raw, gx1_Raw, gy1_Raw, gz1_Raw;
int16_t ax2_Raw, ay2_Raw, az2_Raw, gx2_Raw, gy2_Raw, gz2_Raw;
double AcX, AcY, AcZ, GyX, GyY, GyZ; //valores apos a conversao com o fundo de escala

//Barometro
Adafruit_BMP280 bmp;
float pressure;
float altitude;

//Enable
double minAltitude = 0.0; //dado pelo RocketPy
double apogeeAltitude = 0.0; //dado pelo RocketPy
EnableFunction *enable = new EnableFunction(minAltitude, apogeeAltitude);

void setup() {
  Wire.begin();
  
  mpu1.initialize();
  mpu2.initialize();

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {

  mpu1.getMotion6(&ax1_Raw, &ay1_Raw, &az1_Raw, &gx1_Raw, &gy1_Raw, &gz1_Raw);
  AcX = ax1_Raw / 2048;
  AcY = ay1_Raw / 2048;
  AcZ = az1_Raw / 2048;
  GyX = gx1_Raw / 16.4;
  GyY = gy1_Raw / 16.4;
  GyZ = gz1_Raw / 16.4;

  mpu2.getMotion6(&ax2_Raw, &ay2_Raw, &az2_Raw, &gx2_Raw, &gy2_Raw, &gz2_Raw);
  AcX = ax2_Raw / 2048;
  AcY = ay2_Raw / 2048;
  AcZ = az2_Raw / 2048;
  GyX = gx2_Raw / 16.4;
  GyY = gy2_Raw / 16.4;
  GyZ = gz2_Raw / 16.4;

  pressure = bmp.readPressure();
  altitude = bmp.readAltitude();

  enable->addValues(AcX, altitude); //ver a orientacao do giroscopio
  if(enable->getControlOn() == true) {
    //PID
  }
}
