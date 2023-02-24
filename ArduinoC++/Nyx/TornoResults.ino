#include <Wire.h>
#include <SPIFlash.h>
#include <SPI.h>

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#define HSPI_MISO   12
#define HSPI_MOSI  13
#define HSPI_SCLK   14
#define HSPI_SS     15

SPIClass *hspi = new SPIClass(HSPI);
uint8_t chipSelect = 15;

SPIFlash flash(chipSelect, hspi);

unsigned int addr = 0; //endereco de armazenamento
int input;

void setup() {
  Serial.begin(115200);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS
  Wire.begin();
  flash.begin();  
}

void loop() {
  //Leitura de dados
  Serial.println("Selecione 1 para o primeiro quarto, 2 para o segundo, 3 para o terceiro e 4 para o quarto quarto de dados");  
  while (Serial.available() == 0){}
  input = Serial.parseInt();
  if(input == 1){
    addr = 0;
    for(int i = 0; i < 65536; i++){
      for(int j = 0; j < 7; j++){
        Serial.print((flash.readByte(addr+1) << 8) | flash.readByte(addr));
        Serial.print(",");
        addr = addr + 2;
      }
      Serial.print((flash.readByte(addr+1) << 8) | flash.readByte(addr));
      Serial.println(",");
      addr = addr + 2;
    }
  }
  else if(input == 2){
    addr = 1048576;
    for(int i = 0; i < 65536; i++){
      for(int j = 0; j < 7; j++){
        Serial.print((flash.readByte(addr+1) << 8) | flash.readByte(addr));
        Serial.print(",");
        addr = addr + 2;
      }
      Serial.print((flash.readByte(addr+1) << 8) | flash.readByte(addr));
      Serial.println(",");
      addr = addr + 2;
    }
  }
  else if(input == 3){
    addr = 2097152;
    for(int i = 0; i < 65536; i++){
      for(int j = 0; j < 7; j++){
        Serial.print((flash.readByte(addr+1) << 8) | flash.readByte(addr));
        Serial.print(",");
        addr = addr + 2;
      }
      Serial.print((flash.readByte(addr+1) << 8) | flash.readByte(addr));
      Serial.println(",");
      addr = addr + 2;
    }
  }
  else if(input == 4){
    addr = 3145728;
    for(int i = 0; i < 65536; i++){
      for(int j = 0; j < 7; j++){
        Serial.print((flash.readByte(addr+1) << 8) | flash.readByte(addr));
        Serial.print(",");
        addr = addr + 2;
      }
      Serial.print((flash.readByte(addr+1) << 8) | flash.readByte(addr));
      Serial.println(",");
      addr = addr + 2;
    }
  }
  else
    Serial.println("Número inválido.");
}
