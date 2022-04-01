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
char filename[] = "w3_controle_amostrado.csv";
int recNum = 0; // We have read 0 records so far

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
  Serial.begin(500000);
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  // Atribui pino ao servo
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object

  // Prepara controlador
  Setpoint = 0; // Velocidade angular desejada
  myPID.SetMode(AUTOMATIC); //turn the PID on
  myPID.SetOutputLimits(-8, 8);
  myPID.SetSampleTime(10);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  myFileSD = SD.open(filename, FILE_READ);
  Serial.println("initialization done.");
}

void loop() {
  micros_atual = micros();
  if ((micros_atual - micros_anterior) >= intervalo_de_obtencao_e_filtragem_de_dados ) {
    micros_anterior = micros_atual;
    inicio = micros(); // <-------------------
    
  // Faz a leitura do cartao SD
  if (myFileSD.available()) { 
      String list = myFileSD.readStringUntil('\n');
//      Serial.println(list);
      recNum++; // Count the record
      GyZ = list.toFloat() * 57.29577951; // converte de radianos para graus
  }

  else {
    GyZ = 0;
    Serial.println("Not Available");
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
    myservo.write(Output * 10 + 90); // Envia o Output para o servo. Falta a conversão da pushrod
//    Serial.print("Output = ");
//    Serial.println(Output);
  }
    Serial.print("GyZ = "); Serial.print(GyZ);
    Serial.print(" | GyZ filtrado = "); Serial.println(dado_filtrado);
      Serial.print(" | Servo Position = "); Serial.println(10 * Output + 90);

    fim = micros(); // <-------------------
  }
}