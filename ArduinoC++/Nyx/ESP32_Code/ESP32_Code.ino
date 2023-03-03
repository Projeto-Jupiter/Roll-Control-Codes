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
double AcX1, AcY1, AcZ1, GyX1, GyY1, GyZ1;                              
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

std::map<int, float> myMap = {
  {-8, 1320}, // -8 -> 1348 - 1330
  {-7, 1340}, // -7 -> 1368 - 1349
  {-6, 1358}, // -6 -> 1387 - 1369
  {-5, 1378}, // -5 -> 1407 - 1388
  {-4, 1398}, // -4 -> 1426 - 1408
  {-3, 1418}, // -3 -> 1446 - 1427
  {-2, 1438}, // -2 -> 1465 - 1447
  {-1, 1458}, // -1 -> 1485 - 1466
  {0, 1477},  //  0 -> 1487 - 1502
  {1, 1495},  //  1 -> 1503 - 1522
  {2, 1513},  //  2 -> 1523 - 1541 
  {3, 1532},  //  3 -> 1542 - 1561
  {4, 1552},  //  4 -> 1562 - 1581
  {5, 1572},  //  5 -> 1582 - 1600
  {6, 1595},  //  6 -> 1601 - 1620 
  {7, 1612},  //  7 -> 1621 - 1639
  {8, 1633}   //  8 -> 1640 - 1659
};

std::map<int, float> myMap2 = {
  {-8, 1275}, // -8 -> 1348 - 1330
  {-7, 1295}, // -7 -> 1368 - 1349
  {-6, 1315}, // -6 -> 1387 - 1369
  {-5, 1335}, // -5 -> 1407 - 1388
  {-4, 1355}, // -4 -> 1426 - 1408
  {-3, 1375}, // -3 -> 1446 - 1427
  {-2, 1398}, // -2 -> 1465 - 1447
  {-1, 1418}, // -1 -> 1485 - 1466
  {0, 1438},  //  0 -> 1487 - 1502
  {1, 1458},  //  1 -> 1503 - 1522
  {2, 1477},  //  2 -> 1523 - 1541 
  {3, 1495},  //  3 -> 1542 - 1561
  {4, 1513},  //  4 -> 1562 - 1581
  {5, 1532},  //  5 -> 1582 - 1600
  {6, 1552},  //  6 -> 1601 - 1620 
  {7, 1572},  //  7 -> 1621 - 1639
  {8, 1592}   //  8 -> 1640 - 1659
};

// Criando o Filtro 
double dado_filtrado;
LowPass filtro;

// Inicializa variaveis do controlador
float cantAngle;
float Kp = 0.005, Ki = 0.01, Kd = 0.000;
int setPoint = 0;
RocketPID pid(setPoint, Kp, Ki, Kd);
float Output;
float dutyCycle;                          // usado como input do servo, em microssegundos
int16_t lastCantAngle = 0;                    // em graus
float lower = -8.0;
float upper = 8.0;
float dt = 0.01;

// Controle dos estados do foguete
double altMotorOff = 1020.0;                    // Altitude de burnout - em (m) (ASL)
float minAltitude = 720.0;                       // Altitude redundante caso acelerômetro falhe - em (m) (ASL)
EnableFunction *enable = new EnableFunction(altMotorOff, minAltitude);

// Variaveis do controle de frequencia
uint32_t t0, t1;
int16_t delta = 0;
long memCont = 0;
long alternador = 2;
int16_t periodo;

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
  
  mpu1.setXAccelOffset(-5869);  // calibrações
  mpu1.setYAccelOffset(-545);
  mpu1.setZAccelOffset(737);
  mpu1.setXGyroOffset(20);
  mpu1.setYGyroOffset(31);
  mpu1.setZGyroOffset(22);

  mpu2.initialize();

  Wire.beginTransmission(0x69);
  Wire.write(0x1B);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();

  Wire.beginTransmission(0x69);
  Wire.write(0x1C);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();

  mpu2.setXAccelOffset(-1241);   // calibrações
  mpu2.setYAccelOffset(-203);
  mpu2.setZAccelOffset(1677);
  mpu2.setXGyroOffset(14);
  mpu2.setYGyroOffset(-4);
  mpu2.setZGyroOffset(69);

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
  for (int pos = 1478; pos <= 2000; pos++){
    servo1.writeMicroseconds(pos);
    servo2.writeMicroseconds(pos);
    delay(2);
  }
  delay(1000);
  for (int pos = 1999; pos >= 1478; pos--){
    servo1.writeMicroseconds(pos);
    servo2.writeMicroseconds(pos);
    delay(2);
  }
  delay(100);
  servo1.writeMicroseconds(myMap[0]);
  servo2.writeMicroseconds(myMap2[0]);

  Serial.print("Configuração Completa");
  delay(60000);                                     // delay para inicialização da memória
  t0 = millis();
}

void loop() {
  
  t1 = millis();
  if ((t1 - t0) >= 10) {
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

    altitude = bmp.readAltitude() + 33;   //offset de 33m para correção do barômetro    
    enable->addValues(AcX2, altitude);

    if(enable->getControlOn()){
      dado_filtrado = filtro.addData(GyX2);
      cantAngle = pid.computePID(dado_filtrado*(3.1416/180))*180/3.1416;
      
      if (round(cantAngle) > lastCantAngle) {
        lastCantAngle++;
        servo1.writeMicroseconds(myMap[lastCantAngle]);
        servo2.writeMicroseconds(myMap2[lastCantAngle]);
      }
      else if(round(cantAngle) < lastCantAngle) {
        lastCantAngle--;
        servo1.writeMicroseconds(myMap[lastCantAngle]);
        servo2.writeMicroseconds(myMap2[lastCantAngle]);
      }
      else {
        servo1.writeMicroseconds(myMap[lastCantAngle]);
        servo2.writeMicroseconds(myMap2[lastCantAngle]);
      }
    }
    
    if(enable->getMotorOn() || enable->getControlOn()){
      if (memCont < 8388608 && (alternador % 2) == 0) {
        periodo = int16_t(t1-t0);           //2bytes do periodo
        byte0 = (periodo & 255);
        Serial.print(periodo);
        Serial.print(",");
        byte1 = ((periodo >> 8) & 255);
        delay(1);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes do accx do sensor 2
        Serial.print(ax2_Raw);
        Serial.print(",");
        byte0 = (ax2_Raw & 255);
        byte1 = ((ax2_Raw >> 8) & 255);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes do accy do sensor 2
        Serial.print(ay2_Raw);
        Serial.print(",");
        byte0 = (ay2_Raw & 255);
        byte1 = ((ay2_Raw >> 8) & 255);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes do accz do sensor 2
        Serial.print(az2_Raw);
        Serial.print(",");
        byte0 = (az2_Raw & 255);
        byte1 = ((az2_Raw >> 8) & 255);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;
        
        //2bytes do w1 do sensor 2
        Serial.print(gx2_Raw);
        Serial.print(",");
        byte0 = (gx2_Raw & 255);
        byte1 = ((gx2_Raw >> 8) & 255);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;
    
        //2bytes do w2 do sensor 2
        Serial.print(gy2_Raw);
        Serial.print(",");
        byte0 = (gy2_Raw & 255);
        byte1 = ((gy2_Raw >> 8) & 255);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;
    
        //2bytes do w3 do sensor 2
        Serial.print(gz2_Raw);
        Serial.print(",");
        byte0 = (gz2_Raw & 255);
        byte1 = ((gz2_Raw >> 8) & 255);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes do sinal enviado aos servos
        Serial.print(lastCantAngle);
        Serial.print(",");
        byte0 = (lastCantAngle & 255);
        byte1 = ((lastCantAngle >> 8) & 255);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        //2bytes da altitude
        altitudeBytes = int16_t(altitude * 10);
        Serial.print(altitudeBytes);
        Serial.println(",");
        byte0 = (altitudeBytes & 255);
        byte1 = ((altitudeBytes >> 8) & 255);
        flash.writeByte(memCont, byte0);
        delay(1);
        memCont++;
        flash.writeByte(memCont, byte1);
        delay(1);
        memCont++;

        alternador++;
      }
      else if(memCont < 8388608)
        alternador++;
    }
    t0 = t1;
  }
}
