#include<Wire.h> //Biblioteca para comunicação I2C
#include<MPU6050.h> // Biblioteca para uso do sensor
#include <Servo.h>
#include <PID_v1.h> // Biblioteca com a implementação do controlador PID
#include <EEPROM.h>

// Defining filter function. Low pass chebyshev filter order=2 alpha1=0.445
class FilterChLp2
{
  public:
    FilterChLp2()
    {
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
    }

    double addData(double x) //class II
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (8.319393674533791527e-1 * x)
             + (-0.73546840593695905763 * v[0])
             + (-1.59228906387655766430 * v[1]);
      return (v[0] + v[2]) + 2 * v[1];
    }

  private:
    double v[3];
};

// Inicializando variaveis do sensor
MPU6050 mpu;
const int MPU_addr = 0x68; //Endereco I2C do sensor MPU_addr-6050
int16_t AcX_raw, AcY_raw, AcZ_raw, Tmp_raw, GyX_raw, GyY_raw, GyZ_raw; //Variaveis para pegar os valores medidos
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //Valores finais

// inicializa variaveis do servo
Servo myservo;  // create servo object to control a servo

// Criando o Filtro
double dado_filtrado;
FilterChLp2 filtro;

// inicializa variaveis do controlador
double Setpoint, Output, ServoOutput, K_convertion = 1.62; // Variables of interest
double Kp = 0.015, Ki = 0.315, Kd = 0;
PID myPID(&dado_filtrado, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Enable
int Enable_eletronica = 1;

// ===========================================================================================================
void setup() {
  //Inicia a comunicaçao serial (para exibir os valores lidos)
  Serial.begin(9600);

  Wire.begin(); //Inicia a comunicação I2C
  Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
  Wire.write(0x6B); // registrador PWR_MGMT_1
  Wire.write(0); // Manda 0 e "acorda" o MPU_addr 6050
  Wire.endTransmission(true);

  // Configura Giroscópio
  /*
    Wire.write(0b00000000); // fundo de escala em +/-250°/s
    Wire.write(0b00001000); // fundo de escala em +/-500°/s
    Wire.write(0b00010000); // fundo de escala em +/-1000°/s
    Wire.write(0b00011000); // fundo de escala em +/-2000°/s
  */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();

  // Configura Acelerometro
  /*
      Wire.write(0b00000000); // fundo de escala em +/-2g
      Wire.write(0b00001000); // fundo de escala em +/-4g
      Wire.write(0b00010000); // fundo de escala em +/-8g
      Wire.write(0b00011000); // fundo de escala em +/-16g
  */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0b00011000);  // fundo de escala
  Wire.endTransmission();

  // Atribui pino ao servo
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  // Prepara controlador
  Setpoint = 0; // Velocidade angular desejada
  myPID.SetMode(AUTOMATIC); //turn the PID on
  myPID.SetOutputLimits(-8, 8);
  // myPID.SetSampleTime(10); // The sample time is being manually controlled on the loop

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-411);
  mpu.setYAccelOffset(-645);
  mpu.setZAccelOffset(-1169);
  mpu.setXGyroOffset(-35);
  mpu.setYGyroOffset(282);
  mpu.setZGyroOffset(-17);

  myservo.write(86);
  delay(300);
  myservo.write(94);
  delay(300);
  myservo.write(90);
}


// ===========================================================================================================
// Para Debugagem
unsigned long inicio = 0; // <-------------------
unsigned long fim = 0; // <-------------------

// Gerenciamento de tempo
unsigned long intervalo_de_obtencao_e_filtragem_de_dados = 100000; // Em microssegundos
unsigned long intervalo_de_salvamento_de_dados = 4000000; // Em microssegundos
unsigned long micros_atual = 0;
unsigned long micros_anterior_obtencao_e_filtragem = 0;
unsigned long micros_anterior_salvamento_de_dados = 0;

// EEPROM
int address = 0;
int ServoOutputSaved;
int dadoFiltradoSaved;

void loop() {

  micros_atual = micros();
  if ((micros_atual - micros_anterior_obtencao_e_filtragem) >= intervalo_de_obtencao_e_filtragem_de_dados ) {
    micros_anterior_obtencao_e_filtragem = micros_atual;
    Serial.print("micros_atual: ");
    Serial.println(micros_atual);

    inicio = micros(); // <-------------------

    // Obtem os dados do sensor
    Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
    Wire.write(0x3B); // registrador dos dados medidos (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // faz um "pedido" para ler 14 registradores, que serão os registrados com os dados medidos
    AcX_raw = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY_raw = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ_raw = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp_raw = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX_raw = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY_raw = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ_raw = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    AcX = AcX_raw / 2048;
    AcY = AcY_raw / 2048;
    AcZ = AcZ_raw / 2048;
    GyX = GyX_raw / 16.4;
    GyY = GyY_raw / 16.4;
    GyZ = GyZ_raw / 16.4;

    //  float dado_filtrado = filter.addData(GyZ);
    dado_filtrado = filtro.addData(GyZ);

    if (Enable_eletronica == 1) {
      myPID.Compute();

      // Convert canard output to servo output
      ServoOutput = K_convertion * Output + 90;

      // Write output on the servp
      myservo.write(ServoOutput); // Output
    }

    else {
      myservo.write(90);
      // Adicionar rotina que verifica o enable
    }

    if (address < EEPROM.length()) {
      if ((micros_atual - micros_anterior_salvamento_de_dados) >= intervalo_de_salvamento_de_dados ) {
        micros_anterior_salvamento_de_dados = micros_atual;
        
        ServoOutputSaved = 100 * ServoOutput; // double to int
        dadoFiltradoSaved = 100 * dado_filtrado;

        writeIntEEPROM(address, dado_filtrado); // Salva dado filtrado
        address = address + 2; // move a pduas posições na memoria
        writeIntEEPROM(address, ServoOutput); // Salva output para Servo
        address = address + 2;
        writeIntEEPROM(address, AcZ_raw);
        Serial.println('Saved on EEPROM');
      }
    }

    fim = micros(); // <-------------------
    Serial.print("Tempo lendo dados: "); // <-------------------
    Serial.println(fim - inicio); // <-------------------

    inicio = micros(); // <-------------------

    Serial.print("GyZ = "); Serial.print(GyZ);
    Serial.print(" | GyZ filtrado = "); Serial.println(dado_filtrado);
    Serial.print(" | Servo Position = "); Serial.println(ServoOutput);

    fim = micros(); // <-------------------
    Serial.print("Tempo gravando dados: "); // <-------------------
    Serial.println(fim - inicio); // <-------------------

  }
}

// =================================================================================================================================
void writeIntEEPROM(int address, int number)
{
  /* writes a int into the an addres in the eeprom by converting
  */
  byte byte1 = number >> 8;
  byte byte2 = number & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}
