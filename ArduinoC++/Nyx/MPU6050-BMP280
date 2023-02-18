#include <Wire.h>
#include "MPU6050.h"
#include <Adafruit_BMP280.h>

//Giroscopios
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);
int16_t ax1_Raw, ay1_Raw, az1_Raw, gx1_Raw, gy1_Raw, gz1_Raw;
int16_t ax2_Raw, ay2_Raw, az2_Raw, gx2_Raw, gy2_Raw, gz2_Raw;

//Barometro
Adafruit_BMP280 bmp;
float pressure;
float altitude;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  mpu1.initialize();
/*   mpu1.setXAccelOffset(-411);
  mpu1.setYAccelOffset(-645);
  mpu1.setZAccelOffset(-1169);
  mpu1.setXGyroOffset(-35);
  mpu1.setYGyroOffset(282);
  mpu1.setZGyroOffset(-17); */

  mpu2.initialize();
/*   mpu2.setXAccelOffset(-411);
  mpu2.setYAccelOffset(-645);
  mpu2.setZAccelOffset(-1169);
  mpu2.setXGyroOffset(-35);
  mpu2.setYGyroOffset(282);
  mpu2.setZGyroOffset(-17); */
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
              Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
              Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
              Adafruit_BMP280::FILTER_X16,      /* Filtering. */
              Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {

  mpu1.getMotion6(&ax1_Raw, &ay1_Raw, &az1_Raw, &gx1_Raw, &gy1_Raw, &gz1_Raw);
  mpu2.getMotion6(&ax2_Raw, &ay2_Raw, &az2_Raw, &gx2_Raw, &gy2_Raw, &gz2_Raw);

  /* Serial.println(informacao) */

  pressure = bmp.readPressure();
  altitude = bmp.readAltitude();

  /* Serial.println(informacao) */
}
