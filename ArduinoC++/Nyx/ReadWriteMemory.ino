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

int16_t var = -39; //variavel a ser armazenada
int16_t concatenada = 0; //variavel de leitura
int8_t byte1 = (var & 255);
int8_t byte2 = ((var >> 8) & 255);
uint8_t chipSelect = 15;

SPIFlash flash(chipSelect, hspi);

uint addr = 0; //endereco de armazenamento

void setup() {
  Serial.begin(115200);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS
  Wire.begin();
  flash.begin();
  flash.eraseChip(); //Executar uma vez para limpar a memoria
  
//Escrita de dados
  //flash.writeByte(addr, byte1);
  //flash.writeByte(addr + 1, byte2);
  
//Leitura de dados
  //concatenada = (flash.readByte(addr+1) << 8) | flash.readByte(addr);
  Serial.print("Done");  
}

void loop() {
  
}
