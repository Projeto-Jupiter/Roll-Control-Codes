#include <Arduino.h>
#include<Wire.h>//Biblioteca para comunicação I2C
#include<MPU6050.h>
#include <Servo.h>
#include <PID_v1.h>
#include <SD.h>
#include <SPI.h>

template<size_t N>
float dot(float u[], float v[])
{
  float res = 0;

  for (unsigned int i = 0; i < N; i++)
  {
    res += u[i] * v[i];
  }
  return res;
}

template<size_t N>
float forwardConcatenate(float v[], float newData)
{
  float last = v[N - 1];
  for (int i = N - 1; i > 0; i--)
  {
    v[i] = v[i - 1];
  }
  v[0] = newData;
  return last;
}


template<size_t M, size_t L>
class NLMS
{
  public:
    NLMS(float mu, float eps): mu(mu), eps(eps), u2(0)
    {
      for (unsigned int i = 0; i < M; i++)
      {
        u[i] = 0;
        w[i] = 0;
      }
    }

    float addData(float newData)
    {
      float out, nextU, oldU, e;

      nextU = forwardConcatenate<L>(d, newData);
      oldU = forwardConcatenate<M>(u, nextU);
      out = dot<M>(w, u);
      e = newData - out;
      u2 += nextU * nextU - oldU * oldU;
      updateWeights(e);

      return out;
    }
  private:
    float u[M], w[M], d[L];
    const float mu, eps;
    float u2;

    void updateWeights(float e)
    {
      float passo = mu * e / (eps + u2);
      for (unsigned int i = 0; i < M; i++)
      {
        w[i] += passo * u[i];
      }
    }
};

class EnableFunction
{
  public:
    EnableFunction()
    {
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
      v[3] = 0.0;
      v[4] = 0.0;
      mean = 0.0;
    }

    double addAcceleration(double az)
    {
      mean = mean - (v[0] + az) / 5;
      v[0] = v[1];
      v[1] = v[2];
      v[2] = v[3];
      v[3] = v[4];
      v[4] = az;

      if (mean > 10)
      {
        return true;
      }
      else{
        return false;
      }
    }

    private:
    double v[5];
    double mean;
};

// Inicializando variaveis do sensor
MPU6050 mpu;
const int MPU_addr = 0x68; //Endereco I2C do sensor MPU_addr-6050
int16_t AcX_raw, AcY_raw, AcZ_raw, Tmp_raw, GyX_raw, GyY_raw, GyZ_raw; //Variaveis para pegar os valores medidos
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //Valores finais

// inicializa variaveis do servo
Servo myservo;  // create servo object to control a servo

// Criando o Filtro
const float mu = 0.03, eps = 1e-6;
const int M = 40, L = 50;
double dado_filtrado;
NLMS<M, L> filtro(mu, eps);

// inicializa variaveis do controlador
double Setpoint, Output; // Variables of interest
double Kp = 0.015, Ki = 0.315, Kd = 0;
PID myPID(&dado_filtrado, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Inicializando cartão SD
File myFile;
File myFileSD;
int flag=0;
const int chipSelect = BUILTIN_SDCARD;

// Inicializando variável do Enable
boolean Enable = true;
EnableFunction EF;

// Para Debugagem
unsigned long inicio = 0; // <-------------------
unsigned long fim = 0; // <-------------------

// Gerenciamento de tempo
unsigned long intervalo_de_obtencao_e_filtragem_de_dados = 100000; // Em microssegundos
unsigned long micros_atual = 0;
unsigned long micros_anterior = 0;

// ===========================================================================================================


void setup() {
  //Inicia a comunicaçao serial (para exibir os valores lidos)
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }

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
  myPID.SetSampleTime(10);

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-411);
  mpu.setYAccelOffset(-645);
  mpu.setZAccelOffset(-1169);
  mpu.setXGyroOffset(-35);
  mpu.setYGyroOffset(282);
  mpu.setZGyroOffset(-17);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
}

void loop() {
  micros_atual = micros();
  if ((micros_atual - micros_anterior) >= intervalo_de_obtencao_e_filtragem_de_dados ) {
    micros_anterior = micros_atual;
    Serial.print("micros_atual: ");
    Serial.println(micros_atual);

    inicio = micros(); // <-------------------

    // // Obtem os dados do sensor
    // Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
    // Wire.write(0x3B); // registrador dos dados medidos (ACCEL_XOUT_H)
    // Wire.endTransmission(false);
    // Wire.requestFrom(MPU_addr, 14, true); // faz um "pedido" para ler 14 registradores, que serão os registrados com os dados medidos
    // AcX_raw = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    // AcY_raw = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    // AcZ_raw = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    // Tmp_raw = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    // GyX_raw = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    // GyY_raw = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    // GyZ_raw = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    // AcX = AcX_raw / 2048;
    // AcY = AcY_raw / 2048;
    // AcZ = AcZ_raw / 2048;
    // GyX = GyX_raw / 16.4;
    // GyY = GyY_raw / 16.4;
    // GyZ = GyZ_raw / 16.4;

  // Faz a leitura do cartao SD
  if (myFile.available()) { 
      char letter = myFile.read(); //read next character from file
      GyZ = 0 * 57.29577951; // converte de radianos para graus
  }

  else {
    GyZ = 0;
  }
    
  //  float dado_filtrado = filter.addData(GyZ);
  dado_filtrado = filtro.addData(GyZ);

  // Caso o Enable esteja desativado, realiza apenas a filtragem
  if (Enable == false){
    myservo.write(90);
    Enable = EF.addAcceleration(AcZ);
  }

  // Caso o Enable esteja ativado, opera o controle
  else {
    myPID.Compute(); // Atualiza o output do PID
    myservo.write(Output); // Envia o Output para o servo. Falta a conversão da pushrod
  }

    fim = micros(); // <-------------------
    Serial.print("Tempo lendo dados: "); // <-------------------
    Serial.println(fim - inicio); // <-------------------
    inicio = micros(); // <-------------------

    // Salva dados no SD
    myFileSD = SD.open("TesteBruno.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFileSD) {
      if (flag == 0) {
        myFileSD.println("acx,acy,acz,tmp,gyx,gyy,gyz,dado_filtrado,tempo");
        flag = 1;
      }
     myFileSD.print(AcX_raw); myFileSD.print(','); myFileSD.print(AcY_raw); myFileSD.print(',');
     myFileSD.print(AcZ_raw); myFileSD.print(','); myFileSD.print(Tmp); myFileSD.print(',');
     myFileSD.print(GyX_raw); myFileSD.print(','); myFileSD.print(GyY_raw); myFileSD.print(',');
     myFileSD.print(GyZ_raw); myFileSD.print(','); myFileSD.print(dado_filtrado);
      myFileSD.write(','); myFileSD.write(micros_atual + '\n');
      Serial.println("Aberto E Salvo");
      // close the file:
      myFile.close();
    }
    else {
      // if the file didn't open, print an error:
      Serial.println("error opening TesteBruno.txt");
    }

    Serial.print("GyZ = "); Serial.print(GyZ);
    Serial.print(" | GyZ filtrado = "); Serial.println(dado_filtrado);
    //  Serial.print(" | Servo Position = "); Serial.println(10 * Output + 90);

    fim = micros(); // <-------------------
    Serial.print("Tempo gravando dados: "); // <-------------------
    Serial.println(fim - inicio); // <-------------------

  }
}