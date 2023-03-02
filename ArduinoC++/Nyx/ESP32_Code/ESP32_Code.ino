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

// Inicializando variaveis dos giroscopios
const int mpuAddr1 = 0x68;   // Endereço I2C do primeiro giroscópio (AD0 low)
const int mpuAddr2 = 0x69;   // Endereço I2C do segundo giroscópio (AD0 HIGH)

MPU6050 mpu1(mpuAddr1);     
MPU6050 mpu2(mpuAddr2);
int16_t ax1_Raw, ay1_Raw, az1_Raw, gx1_Raw, gy1_Raw, gz1_Raw;
int16_t ax2_Raw, ay2_Raw, az2_Raw, gx2_Raw, gy2_Raw, gz2_Raw;
double AcX1, AcY1, AcZ1, GyX1, GyY1, GyZ1;                               // Valores finais (float?)
double AcX2, AcY2, AcZ2, GyX2, GyY2, GyZ2;

// Inicializando variaveis do barometro
Adafruit_BMP280 bmp;        // debuggar para encontrar a variavel de 2 bytes
const int bmpAddr = 0x76;
int16_t altitudeBytes;
float altitude;

// Variaveis da memoria
#define HSPI_MISO   12    // pino de saída da memória, entrada do ESP
#define HSPI_MOSI   13    // pino de entrada da memória, saída do ESP
#define HSPI_SCLK   14    // pino do clock
#define HSPI_SS     15    // pino do chipselect

SPIClass *hspi = new SPIClass(HSPI);

int16_t value;            // variável a ser armazenada
int8_t byte0;             // Byte de value menos significativo
int8_t byte1;             // Byte de value mais significativo

SPIFlash flash(HSPI_SS, hspi);

// inicializa variaveis dos servos
Servo servo1;
Servo servo2;

std::map<int, float> myMap = {  // Mapeamento das posições dos servos
  {-8, 1340}, // -8 -> 1330 - 1348
  {-7, 1358}, // -7 -> 1349 - 1368
  {-6, 1378}, // -6 -> 1369 - 1387
  {-5, 1398}, // -5 -> 1388 - 1407
  {-4, 1418}, // -4 -> 1408 - 1426
  {-3, 1438}, // -3 -> 1427 - 1446
  {-2, 1458}, // -2 -> 1447 - 1465
  {-1, 1477}, // -1 -> 1466 - 1485
  {0, 1495},  //  0 -> 1487 - 1502
  {1, 1513},  //  1 -> 1503 - 1522
  {2, 1532},  //  2 -> 1523 - 1541 
  {3, 1552},  //  3 -> 1542 - 1561
  {4, 1572},  //  4 -> 1562 - 1581
  {5, 1591},  //  5 -> 1582 - 1600
  {6, 1612},  //  6 -> 1601 - 1620 
  {7, 1630},  //  7 -> 1621 - 1639
  {8, 1650}   //  8 -> 1640 - 1659
};

// Criando o Filtro ??
double dado_filtrado;
LowPass filtro;

// Inicializa variaveis do controlador
float cantAngle;
int16_t cantAngleBytes;        
float Kp = 0.005, Ki = 0.01, Kd = 0.000;
int setPoint = 0;
RocketPID pid(setPoint, Kp, Ki, Kd);
float Output;
float dutyCycle;                          // usado como input do servo, em microssegundos
int lastCantAngle = 0;                    // em graus
float lower = -8.0;
float upper = 8.0;
float dt = 0.01;

// Controle dos estados do foguete
double minAltitude = ;                    // Altitude de burnout em (m) (ASL)
double apogeeAltitude = ;                 // Apogeu em (m) (ASL)
EnableFunction *enable = new EnableFunction(minAltitude, apogeeAltitude);

// Variaveis do controle de frequencia
uint32_t t0, t1;
int16_t delta = 0;
long memCont = 0;
int16_t periodo;

// ===========================================================================================================
void setup() {
  //Inicia a comunicaçao serial (para exibir os valores lidos)
  Serial.begin(115200);

  //Configuração da memória
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS
  Wire.begin();
  flash.begin();
  //flash.eraseChip(); //Executar uma vez para limpar a memória

  //Configuração dos sensores
  mpu1.initialize();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();
  
  mpu1.setXAccelOffset(-2289);  // calibrações
  mpu1.setYAccelOffset(-616);
  mpu1.setZAccelOffset(832);
  mpu1.setXGyroOffset(19);
  mpu1.setYGyroOffset(31);
  mpu1.setZGyroOffset(28);

  mpu2.initialize();

  Wire.beginTransmission(0x69);
  Wire.write(0x1B);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();

  Wire.beginTransmission(0x69);
  Wire.write(0x1C);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();

  mpu2.setXAccelOffset(2146);   // calibrações
  mpu2.setYAccelOffset(-231);
  mpu2.setZAccelOffset(1622);
  mpu2.setXGyroOffset(12);
  mpu2.setYGyroOffset(1);
  mpu2.setZGyroOffset(71);

  bmp.begin(bmpAddr);
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode. 
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling 
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling 
                  Adafruit_BMP280::FILTER_X16,      // Filtering. 
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time. 
  
  //Configuração dos servos
  servo1.attach(33); 
  servo2.attach(18);

  //Configuração do controlador
  pid.setLimits(lower, upper);
  pid.setDt(dt);

  //"Oi" das cannards
  servo1.writeMicroseconds(myMap[0]);
  servo2.writeMicroseconds(myMap[0]);
  delay(100);
  for (int pos = 1496; pos <= 2000; pos++){
    servo1.writeMicroseconds(pos);
    servo2.writeMicroseconds(pos);
    delay(2);
  }
  delay(1000);
  for (int pos = 1999; pos >= 1496; pos--){
    servo1.writeMicroseconds(pos);
    servo2.writeMicroseconds(pos);
    delay(2);
  }
  delay(100);
  servo1.writeMicroseconds(myMap[0]);
  servo2.writeMicroseconds(myMap[0]);

  delay(60000);         // delay para inicialização da memória
  t0 = millis();
}

void loop() {

  t1 = micros();
  if ((t1 - t0) >= 10) {
    
    //Obtem os dados dos sensores
    //mpu1.getMotion6(&ax1_Raw, &ay1_Raw, &az1_Raw, &gx1_Raw, &gy1_Raw, &gz1_Raw);
    mpu2.getMotion6(&ax2_Raw, &ay2_Raw, &az2_Raw, &gx2_Raw, &gy2_Raw, &gz2_Raw);
   
    //AcX1 = ax1_Raw / 2048;
    //AcY1 = ay1_Raw / 2048;
    //AcZ1 = az1_Raw / 2048;
    //GyX1 = gx1_Raw / 16.4;
    //GyY1 = gy1_Raw / 16.4;
    //GyZ1 = gz1_Raw / 16.4;

    AcX2 = ax2_Raw / 2048;
    AcY2 = ay2_Raw / 2048;    
    AcZ2 = az2_Raw / 2048;
    GyX2 = gx2_Raw / 16.4;    
    GyY2 = gy2_Raw / 16.4;
    GyZ2 = gz2_Raw / 16.4;

    altitude = bmp.readAltitude() + 45;   //offset de 45m para correção
    //Serial.println(altitude);
    enable->addValues(AcX2, altitude);

    if(enable->getControlOn()){
      dado_filtrado = filtro.addData(GyX2);
      cantAngle = pid.computePID(dado_filtrado*(3.1416/180))*180/3.1416;
      //Serial.print(cantAngle);
      //Serial.print(",");
      
      if (round(cantAngle) > lastCantAngle) {
        lastCantAngle++;
        servo1.writeMicroseconds(myMap[lastCantAngle]);
        servo2.writeMicroseconds(myMap[lastCantAngle]);
      }
      else if(round(cantAngle) < lastCantAngle) {
        lastCantAngle--;
        servo1.writeMicroseconds(myMap[lastCantAngle]);
        servo2.writeMicroseconds(myMap[lastCantAngle]);
      }
      else {
        servo1.writeMicroseconds(myMap[lastCantAngle]);
        servo2.writeMicroseconds(myMap[lastCantAngle]);
      }

      if (memCont < 8388608) {

        periodo = int16_t(t1-t0);           //2bytes do periodo
        byte0 = (periodo & 255);
        //Serial.print(byte0);
        //Serial.print(",");
        byte1 = ((periodo >> 8) & 255);
        //Serial.print(byte1);
        //Serial.print(",");
        delay(1);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes do accx do sensor 2
        byte0 = (ax2_Raw & 255);
        //Serial.print(byte0);
        //Serial.print(",");
        byte1 = ((ax2_Raw >> 8) & 255);
        //Serial.print(byte1);
        //Serial.print(",");
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes do accy do sensor 2
        byte0 = (ay2_Raw & 255);
        //Serial.print(byte0);
        //Serial.print(",");
        byte1 = ((ay2_Raw >> 8) & 255);
        //Serial.print(byte1);
        //Serial.print(",");
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes do accz do sensor 2
        byte0 = (az2_Raw & 255);
        //Serial.print(byte0);
        //Serial.print(",");
        byte1 = ((az2_Raw >> 8) & 255);
        //Serial.print(byte1);
        //Serial.print(",");
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;
        
        //2bytes do w1 do sensor 2
        byte0 = (gx2_Raw & 255);
        //Serial.print(byte0);
        //Serial.print(",");
        byte1 = ((gx2_Raw >> 8) & 255);
        //Serial.print(byte1);
        //Serial.print(",");
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;
    
        //2bytes do w2 do sensor 2
        byte0 = (gy2_Raw & 255);
        //Serial.print(byte0);
        //Serial.print(",");
        byte1 = ((gy2_Raw >> 8) & 255);
        //Serial.print(byte1);
        //Serial.print(",");
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;
    
        //2bytes do w3 do sensor 2
        byte0 = (gz2_Raw & 255);
        //Serial.print(byte0);
        //Serial.print(",");
        byte1 = ((gz2_Raw >> 8) & 255);
        //Serial.print(byte1);
        //Serial.print(",");
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes do sinal enviado aos servos
        cantAngleBytes = int16_t(cantAngle * 1000);
        byte0 = (cantAngleBytes & 255);
        //Serial.print(byte0);
        //Serial.print(",");
        byte1 = ((cantAngleBytes >> 8) & 255);
        //Serial.print(byte1);
        //Serial.print(",");
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes da altitude
        altitudeBytes = int16_t(altitude * 10);
        byte0 = (altitudeBytes & 255);
        //Serial.print(byte0);
        //Serial.print(",");
        byte1 = ((altitudeBytes >> 8) & 255);
        //Serial.print(byte1);
        //Serial.print(",");
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;
      }
    }
    t0 = t1;
  }
}
