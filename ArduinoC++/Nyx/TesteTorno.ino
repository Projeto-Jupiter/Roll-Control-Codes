#include <Wire.h>
#include <SPIFlash.h>
#include <SPI.h>
#include "MPU6050.h"
#include <Adafruit_BMP280.h>

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

// Variáveis dos sensores
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);
int16_t ax1_Raw, ay1_Raw, az1_Raw, gx1_Raw, gy1_Raw, gz1_Raw;
int16_t ax2_Raw, ay2_Raw, az2_Raw, gx2_Raw, gy2_Raw, gz2_Raw;

// Variáveis da memória
#define HSPI_MISO   12
#define HSPI_MOSI  13
#define HSPI_SCLK   14
#define HSPI_SS     15

SPIClass *hspi = new SPIClass(HSPI);

int16_t value; //variavel a ser armazenada
int8_t byte0;
int8_t byte1;
uint8_t chipSelect = 15;

SPIFlash flash(chipSelect, hspi);

// Variáveis do controle de frequência
uint32_t t0, t1, t0mem, t1mem;
int16_t delta = 0;
int16_t periodo = 0;
uint32_t memCont = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("Iniciando Configuração.");  
  
  //Configuração da memória
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS
  Wire.begin();
  flash.begin();
  //flash.eraseChip(); //Executar uma vez para limpar a memoria

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

  mpu1.setXAccelOffset(-2338);
  mpu1.setYAccelOffset(-652);
  mpu1.setZAccelOffset(843);
  mpu1.setXGyroOffset(19);
  mpu1.setYGyroOffset(32);
  mpu1.setZGyroOffset(21);

  mpu2.initialize();

  Wire.beginTransmission(0x69);
  Wire.write(0x1B);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();

  Wire.beginTransmission(0x69);
  Wire.write(0x1C);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();

  mpu2.setXAccelOffset(2120);
  mpu2.setYAccelOffset(-279);
  mpu2.setZAccelOffset(1622);
  mpu2.setXGyroOffset(11);
  mpu2.setYGyroOffset(0);
  mpu2.setZGyroOffset(71);
  
  t0 = millis();
  Serial.println("Configuração Completa.");  
  delay(10000);
}

void loop() {
    
    t1 = millis();
    if ((t1 - t0) >= 5 && memCont < 8388592) {    // frequência de amostragem = 200hz, T = 5ms
      
      t0mem = millis();

      mpu1.getMotion6(&ax1_Raw, &ay1_Raw, &az1_Raw, &gx1_Raw, &gy1_Raw, &gz1_Raw);
      mpu2.getMotion6(&ax2_Raw, &ay2_Raw, &az2_Raw, &gx2_Raw, &gy2_Raw, &gz2_Raw);
      
      periodo = int16_t(t1-t0);           //2bytes do periodo
      Serial.print(periodo);
      Serial.print(",");
      byte0 = (periodo & 255);
      byte1 = ((periodo >> 8) & 255);
      flash.writeByte(memCont, byte0);
      memCont++;
      flash.writeByte(memCont, byte1);
      memCont++;
      
      value = gx1_Raw;                     //2bytes do w1 do sensor 1
      Serial.print(value);
      Serial.print(",");
      byte0 = (value & 255);
      byte1 = ((value >> 8) & 255);
      flash.writeByte(memCont, byte0);
      memCont++;
      flash.writeByte(memCont, byte1);
      memCont++;
  
      value = gy1_Raw;                     //2bytes do w2 do sensor 1
      Serial.print(value);
      Serial.print(",");
      byte0 = (value & 255);
      byte1 = ((value >> 8) & 255);
      flash.writeByte(memCont, byte0);
      memCont++;
      flash.writeByte(memCont, byte1);
      memCont++;
  
      value = gz1_Raw;                     //2bytes do w3 do sensor 1
      Serial.print(value);
      Serial.print(",");
      byte0 = (value & 255);
      byte1 = ((value >> 8) & 255);
      flash.writeByte(memCont, byte0);
      memCont++;
      flash.writeByte(memCont, byte1);
      memCont++;
  
      value = gx2_Raw;                     //2bytes do w1 do sensor 2
      Serial.print(value);
      Serial.print(",");
      byte0 = (value & 255);
      byte1 = ((value >> 8) & 255);
      flash.writeByte(memCont, byte0);
      memCont++;
      flash.writeByte(memCont, byte1);
      memCont++;
  
      value = gy2_Raw;                     //2bytes do w2 do sensor 2
      Serial.print(value);
      Serial.print(",");
      byte0 = (value & 255);
      byte1 = ((value >> 8) & 255);
      flash.writeByte(memCont, byte0);
      memCont++;
      flash.writeByte(memCont, byte1);
      memCont++;
  
      value = gz2_Raw;                     //2bytes do w3 do sensor 2
      Serial.print(value);
      Serial.print(",");
      byte0 = (value & 255);
      byte1 = ((value >> 8) & 255);
      flash.writeByte(memCont, byte0);
      memCont++;
      flash.writeByte(memCont, byte1);
      memCont++;
  
      t1mem = millis();
      
      delta = int16_t(t1mem-t0mem);
      Serial.print(delta);
      Serial.println(",");
      byte0 = (delta & 255);
      byte1 = ((delta >> 8) & 255);
      flash.writeByte(memCont, byte0);
      memCont++;
      flash.writeByte(memCont, byte1);
      memCont++;
      
      t0 = t1;
    }
    else if(memCont == 524288){
      Serial.println("Memória Preenchida.");
      memCont++;
    } 
}
