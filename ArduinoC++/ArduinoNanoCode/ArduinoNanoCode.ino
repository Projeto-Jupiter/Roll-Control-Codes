#include<Wire.h> //Biblioteca para comunicação I2C
#include<MPU6050.h> // Biblioteca para uso do sensor
#include <Servo.h>
#include <PID_v1.h> // Biblioteca com a implementação do controlador PID
#include <EEPROM.h>

// Defining filter function. Low pass chebyshev filter order=2 alpha1=0.445
class LowPass
{
  public:
    LowPass()
    {
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
      v[3] = 0.0;
      v[4] = 0.0;
      v[5] = 0.0;
      v[6] = 0.0;
      v[7] = 0.0;
      v[8] = 0.0;
      v[9] = 0.0;
      v[10] = 0.0;
      v[11] = 0.0;
      v[12] = 0.0;
      v[13] = 0.0;
      v[14] = 0.0;
      v[15] = 0.0;
      v[16] = 0.0;
      v[17] = 0.0;
      v[18] = 0.0;
      v[19] = 0.0;
      v[20] = 0.0;
      v[21] = 0.0;
      v[22] = 0.0;
      v[23] = 0.0;
      v[24] = 0.0;
      v[25] = 0.0;
      v[26] = 0.0;
      v[27] = 0.0;
      v[28] = 0.0;
      v[29] = 0.0;
      v[30] = 0.0;
      v[31] = 0.0;
      v[32] = 0.0;
      v[33] = 0.0;
      v[34] = 0.0;
      v[35] = 0.0;
      v[36] = 0.0;
      v[37] = 0.0;
      v[38] = 0.0;
      v[39] = 0.0;
      v[40] = 0.0;
      v[41] = 0.0;
    }

    double addData(double x) //class II
<<<<<<< HEAD
    {   v[0] = v[1];
        v[1] = v[2];
        v[2] = v[3];
        v[3] = v[4];
        v[4] = v[5];
        v[5] = v[6];
        v[6] = v[7];
        v[7] = v[8];
        v[8] = v[9];
        v[9] = v[10];
        v[10] = v[11];
        v[11] = v[12];
        v[12] = v[13];
        v[13] = v[14];
        v[14] = v[15];
        v[15] = v[16];
        v[16] = v[17];
        v[17] = v[18];
        v[18] = v[19];
        v[19] = v[20];
        v[20] = v[21];
        v[21] = v[22];
        v[22] = v[23];
        v[23] = v[24];
        v[24] = v[25];
        v[25] = v[26];
        v[26] = v[27];
        v[27] = v[28];
        v[28] = v[29];
        v[29] = v[30];
        v[30] = v[31];
        v[31] = v[32];
        v[32] = v[33];
        v[33] = v[34];
        v[34] = v[35];
        v[35] = v[36];
        v[36] = v[37];
        v[37] = v[38];
        v[38] = v[39];
        v[39] = v[40];
        v[40] = v[41];
        v[41] = x;
      return (0.000730542097152697 * v[0]   -0.000819759833501920 * v[1]   -0.00166082645353546 * v[2]   
        +1.99328430876226e-18 * v[3]    +0.00300608142740516 * v[4]    +0.00251888639362088 * v[5]    
        -0.00337138525783167 * v[6]    -0.00718539449856550 * v[7]    +5.99889752552229e-18 * v[8]    
        +0.0119162828308361 * v[9]    +0.00931337264805999 * v[10]    -0.0116804690875208 * v[11]    
        -0.0235865615284306 * v[12]    +1.13604360012867e-17 * v[13]    +0.0368157010550922 * v[14]    
        +0.0287966766635483 * v[15]    -0.0372274978187327 * v[16]    -0.0809518950838493 * v[17]    
        +1.50810312779462e-17 * v[18]    +0.199488764925137 * v[19]    +0.373897481521116 * v[20]    
        +0.373897481521116 * v[21]    +0.199488764925137 * v[22]    +1.50810312779462e-17 * v[23]    
        -0.0809518950838493 * v[24]    -0.0372274978187327 * v[25]    +0.0287966766635483 * v[26]    
        +0.0368157010550922 * v[27]    +1.13604360012867e-17 * v[28]    -0.0235865615284306 * v[29]    
        -0.0116804690875208 * v[30]    +0.00931337264805999 * v[31]    +0.0119162828308361 * v[32]    
        +5.99889752552229e-18 * v[33]    -0.00718539449856550 * v[34]    -0.00337138525783167 * v[35]    
        +0.00251888639362088 * v[36]    +0.00300608142740516 * v[37]    +1.99328430876226e-18 * v[38]    
        -0.00166082645353546 * v[39]    -0.000819759833501920 * v[40]    +0.000730542097152697 * v[41]);
    }

  private:
    double v[42];
=======
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
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf
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
<<<<<<< HEAD
LowPass filtro;
=======
FilterChLp2 filtro;
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf

// inicializa variaveis do controlador
double Setpoint, Output, ServoOutput, K_convertion = 1.62; // Variables of interest
double Kp = 0.015, Ki = 0.315, Kd = 0;
PID myPID(&dado_filtrado, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Enable
int Enable_eletronica = 1;

// ===========================================================================================================
void setup() {
  //Inicia a comunicaçao serial (para exibir os valores lidos)
<<<<<<< HEAD
  Serial.begin(250000);
=======
  Serial.begin(9600);
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf

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
<<<<<<< HEAD
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object
=======
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf

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

<<<<<<< HEAD
  myservo.write(98);
  delay(500);
  myservo.write(82);
  delay(500);
=======
  myservo.write(86);
  delay(300);
  myservo.write(94);
  delay(300);
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf
  myservo.write(90);
}


// ===========================================================================================================
// Para Debugagem
unsigned long inicio = 0; // <-------------------
unsigned long fim = 0; // <-------------------

// Gerenciamento de tempo
<<<<<<< HEAD
unsigned long intervalo_de_obtencao_e_filtragem_de_dados = 10000; // Em microssegundos
unsigned long intervalo_de_salvamento_de_dados = 250000; // Em microssegundos
=======
unsigned long intervalo_de_obtencao_e_filtragem_de_dados = 100000; // Em microssegundos
unsigned long intervalo_de_salvamento_de_dados = 4000000; // Em microssegundos
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf
unsigned long micros_atual = 0;
unsigned long micros_anterior_obtencao_e_filtragem = 0;
unsigned long micros_anterior_salvamento_de_dados = 0;

// EEPROM
<<<<<<< HEAD
int addr = 0;
=======
int address = 0;
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf
int ServoOutputSaved;
int dadoFiltradoSaved;

void loop() {

  micros_atual = micros();
  if ((micros_atual - micros_anterior_obtencao_e_filtragem) >= intervalo_de_obtencao_e_filtragem_de_dados ) {
    micros_anterior_obtencao_e_filtragem = micros_atual;
<<<<<<< HEAD
//    Serial.print("micros_atual: ");
//    Serial.println(micros_atual);
=======
    Serial.print("micros_atual: ");
    Serial.println(micros_atual);
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf

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

<<<<<<< HEAD
    if (addr < EEPROM.length()) {
=======
    if (address < EEPROM.length()) {
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf
      if ((micros_atual - micros_anterior_salvamento_de_dados) >= intervalo_de_salvamento_de_dados ) {
        micros_anterior_salvamento_de_dados = micros_atual;
        
        ServoOutputSaved = 100 * ServoOutput; // double to int
<<<<<<< HEAD
        dadoFiltradoSaved = 100 * dado_filtrado;        

        writeIntEEPROM(addr, dado_filtrado); // Salva dado filtrado
        addr = addr + 2; // move a pduas posições na memoria
        writeIntEEPROM(addr, ServoOutput); // Salva output para Servo
        addr = addr + 2;
        writeIntEEPROM(addr, AcZ_raw);
        addr = addr + 2;
//        Serial.println("Saved EEPROM");
=======
        dadoFiltradoSaved = 100 * dado_filtrado;

        writeIntEEPROM(address, dado_filtrado); // Salva dado filtrado
        address = address + 2; // move a pduas posições na memoria
        writeIntEEPROM(address, ServoOutput); // Salva output para Servo
        address = address + 2;
        writeIntEEPROM(address, AcZ_raw);
        Serial.println('Saved on EEPROM');
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf
      }
    }

    fim = micros(); // <-------------------
<<<<<<< HEAD
//    Serial.print("Tempo lendo dados: "); // <-------------------
//    Serial.println(fim - inicio); // <-------------------
=======
    Serial.print("Tempo lendo dados: "); // <-------------------
    Serial.println(fim - inicio); // <-------------------
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf

    inicio = micros(); // <-------------------

    Serial.print("GyZ = "); Serial.print(GyZ);
    Serial.print(" | GyZ filtrado = "); Serial.println(dado_filtrado);
<<<<<<< HEAD
//    Serial.print(" | Servo Position = "); Serial.println(ServoOutput);

    fim = micros(); // <-------------------
//    Serial.print("Tempo gravando dados: "); // <-------------------
//    Serial.println(fim - inicio); // <-------------------
=======
    Serial.print(" | Servo Position = "); Serial.println(ServoOutput);

    fim = micros(); // <-------------------
    Serial.print("Tempo gravando dados: "); // <-------------------
    Serial.println(fim - inicio); // <-------------------
>>>>>>> 7f45ba2d12f6b8534902a0c24ff865f10b8e76bf

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
