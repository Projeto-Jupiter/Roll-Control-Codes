/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/
/*
   Notes from Brian Patton 4/19/18
   connection to the Teensy
   MPU 6050     Teensy
   Vcc          3.3V
   GND          GND
   SCL          pin A5
   SDA          pin A4
   XDA          NC  (no connection)
   XCL          NC  (no connection)
   ADO          NC
   INT          pin 2
*/

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>

MPU6050 accelgyro;
unsigned long time;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float heading;
#define LED_PIN 13
bool blinkState = false;

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

int pos = 0;    // variable to store the servo position 

void setup() {
  Wire.begin();   // join I2C bus (I2Cdev library doesn't do this automatically)
  Serial.begin(115200); // Set baud rate to 115200 (figure out ways to double it)
  while (!Serial);

  Serial.println("Initializing I2C devices...");
  delay(500);
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(500);
  accelgyro.setFullScaleAccelRange(0);
  pinMode(LED_PIN, OUTPUT);   // configure Arduino LED

  myservo.attach(2);  // attaches the servo on pin 2
}


void loop() {
  time = millis();  //prints time since program started
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  // accelgyro.getAcceleration(&ax, &ay, &az);
  // accelgyro.getRotation(&gx, &gy, &gz);

//  printData();
  plotAccel();
//  plotGyro();
  blinkState = !blinkState; // blink LED to indicate activity
  digitalWrite(LED_PIN, blinkState);
  //  delay(100); // run at ~100 Hz

  if (az<=0){
   pos = 180;
   myservo.write(pos);
  }
  else{
    pos = 0;
    myservo.write(pos);
  }
}

void printData() {
  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g/m/Time:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.print("\t");

  Serial.println(time);
  delay(100); // run at ~100 Hz
}

void plotAccel() {
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.println(az);
  delay(100); // run at ~100 Hz
}

void plotGyro() {
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);
  delay(200); //
}
