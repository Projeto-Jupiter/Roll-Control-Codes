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

// Inicializando variaveis de fitlragem
const float mu = 0.03, eps = 1e-6;
const int M = 40, L = 50;

// Criando o filtro
NLMS<M, L> filter(mu, eps);

// Inicializando variaveis do sensor
#include<Wire.h>//Biblioteca para comunicação I2C
const int MPU_addr = 0x68; //Endereco I2C do sensor MPU_addr-6050
int16_t AcX_raw, AcY_raw, AcZ_raw, Tmp_raw, GyX_raw, GyY_raw, GyZ_raw; //Variaveis para pegar os valores medidos
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //Valores finais

// Inicializando cartão SD
#include <SD.h>
#include <SPI.h>
File myFile;
File myFileSD;
int flag=0;
const int chipSelect = BUILTIN_SDCARD;

void setup() {

  //Inicia a comunicaçao serial (para exibir os valores lidos)
  Serial.begin(9600);
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


  // supply your own gyro offsets here, scaled for min sensitivity
  //MPU_addr.setXAccelOffset(-2847);
  //MPU_addr.setYAccelOffset(-175);
  //MPU_addr.setZAccelOffset(1269);
  //MPU_addr.setXGyroOffset(-771);
  //MPU_addr.setYGyroOffset(61);
  //MPU_addr.setZGyroOffset(1);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
 }

void loop() {
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

  // Converte valores para unidades reais
  /* Alterar divisão conforme fundo de escala escolhido:
      Acelerômetro
      +/-2g = 16384
      +/-4g = 8192
      +/-8g = 4096
      +/-16g = 2048

      Giroscópio
      +/-250°/s = 131
      +/-500°/s = 65.6
      +/-1000°/s = 32.8
      +/-2000°/s = 16.4
  */
  /*
  AcX = AcX_raw / 2048;
  AcY = AcY_raw / 2048;
  AcZ = AcZ_raw / 2048;
  Tmp = Tmp_raw;
  GyX = GyX_raw / 16.4;
  GyY = GyY_raw / 16.4;
  GyZ = GyZ_raw / 16.4;*/
 
  float dado_filtrado = filter.addData(GyZ_raw);
  /*Serial.print("AcX_raw = "); Serial.print(AcX_raw);
  Serial.print(" | AcY_raw = "); Serial.print(AcY_raw);
  Serial.print(" | AcZ_raw = "); Serial.print(AcZ_raw);
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print("GyX_raw = "); Serial.print(GyX_raw);
  Serial.print(" | GyY_raw = "); Serial.print(GyY_raw);
  Serial.print(" | GyZ_raw = "); Serial.print(GyZ_raw);
  Serial.print("GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.print(GyZ);
  Serial.print(" | dado_filtrado = "); Serial.println(dado_filtrado/16.4);*/
  
  // Salva dados no SD
  myFileSD = SD.open("TesteCalibrado6.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFileSD) {
    if (flag == 0){
      myFileSD.println("acx,acy,acz,tmp,gyx,gyy,gyz,dado_filtrado");
      flag = 1;
    }
    myFileSD.print(AcX_raw);myFileSD.print(',');myFileSD.print(AcY_raw);myFileSD.print(',');
    myFileSD.print(AcZ_raw);myFileSD.print(',');myFileSD.print(Tmp);myFileSD.print(',');
    myFileSD.print(GyX_raw);myFileSD.print(',');myFileSD.print(GyY_raw);myFileSD.print(',');
    myFileSD.print(GyZ_raw);myFileSD.print(',');myFileSD.println(dado_filtrado);
    Serial.println("Aberto E Salvo");
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening TesteCalibrado.txt");
  }
  delay(100);
  
  //  controle();

}
