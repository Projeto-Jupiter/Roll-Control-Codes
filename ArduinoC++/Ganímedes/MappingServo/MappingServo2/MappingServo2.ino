#include <Wire.h>               // Biblioteca para comunicação I2C
#include <SPIFlash.h>           // Biblioteca para escrita de dados na memória
#include <SPI.h>                // Biblioteca para seleção HSPI
#include <MPU6050.h>            // Biblioteca para uso do sensor
#include <Adafruit_BMP280.h>    // Biblioteca para uso do barômetro
#include <ESP32Servo.h>         // Biblioteca para uso dos servos
#include <math.h>               // math.h
#include "RocketPID.h"          // Biblioteca do PID Implementado pelo Jupiter
#include <map>                  // Biblioteca para o mapeamento das posições dos servos
#include "LowPass.h"            // Biblioteca com implementação do filtro passa-baixa
#include "EnableFunction.h"     // Biblioteca responsável por ligar e desligar os sistemas de controle

Servo servo1;
Servo servo2;

std::map<int, float> myMap2 = { //25T para cima
  {-10, 1475}, // -10 -> 1465 - 1484
  {-9, 1492}, // -9 -> 1485 - 1503
  {-8, 1515}, // -8 -> 1504 - 1523
  {-7, 1535}, // -7 -> 1524 - 1542
  {-6, 1555}, // -6 -> 1543 - 1562
  {-5, 1575}, // -5 -> 1563 - 1582
  {-4, 1593}, // -4 -> 1583 - 1601
  {-3, 1612}, // -3 -> 1602 - 1621
  {-2, 1632}, // -2 -> 1622 - 1640
  {-1, 1651}, // -1 -> 1641 - 1660
  {0, 1671},  //  0 -> 1661 - 1679 
  {1, 1690},  //  1 -> 1680 - 1699
  {2, 1710},  //  2 ->  1700 - 1718
  {3, 1729},  //  3 -> 1719 - 1738
  {4, 1748},  //  4 -> 1739 - 1757
  {5, 1768},  //  5 -> 1758 - 1777
  {6, 1788},  //  6 -> 1778 - 1796
  {7, 1808},  //  7 -> 1797 - 1816
  {8, 1827},  //  8 -> 1817 - 1835
  {9, 1842},  // 9 -> 1836 - 1855
  {10, 1866}, // 10 -> 1856 - 1874
};

void setup() {
  Serial.begin(115200);
// Inicializando variaveis dos giroscopios
  servo1.attach(33);
  servo2.attach(18);

  //"Oi" das cannards
  servo1.writeMicroseconds(0);
  servo2.writeMicroseconds(0);
  delay(100);
  for (int pos = 0; pos <= 180; pos++){
    servo1.write(pos);
    servo2.write(pos);
  }
  delay(1000);
  for (int pos = 180; pos >= 0; pos--){
    servo1.write(pos);
    servo2.write(pos);
  }
  delay(100);
  servo1.writeMicroseconds(0);
  servo2.writeMicroseconds(0);
}

void loop() {
  servo2.writeMicroseconds(myMap2[0]);
  delay(1000);
  Serial.println("Inicio");
  int POS = 0;
  
  for(int i = POS; i < POS + 11; i++) {
    Serial.println(i);
    servo2.writeMicroseconds(myMap2[i]);
    delay(1500);
  } 
  POS = 0;
  for(int i = POS + 10; i > POS - 1; i--) {
    Serial.println(i);
    servo2.writeMicroseconds(myMap2[i]);
    delay(1500);
  } 
  POS = 0;
  for(int i = POS; i > POS - 11; i--) {
    Serial.println(i);
    servo2.writeMicroseconds(myMap2[i]);
    delay(1500);
  } 
  POS = 0;
  for(int i = POS - 10; i < POS + 1; i++) {
    Serial.println(i);
    servo2.writeMicroseconds(myMap2[i]);
    delay(1500);
  } 

}
