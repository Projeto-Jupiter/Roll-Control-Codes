#include<Wire.h>//Biblioteca para comunicação I2C
#include<MPU6050.h>
#include <Servo.h>
#include <PID_v1.h>

// Defining filter function. Low pass chebyshev filter order=2 alpha1=0.445 
class FilterChLp2
{
  public:
    FilterChLp2()
    {
      v[0]=0.0;
      v[1]=0.0;
      v[2]=0.0;
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
double Setpoint, Output; // Variables of interest
double Kp=0.015, Ki=0.315, Kd=0;
PID myPID(&dado_filtrado, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

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
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
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
  mpu.setXAccelOffset(-760);
  mpu.setYAccelOffset(-1668);
  mpu.setZAccelOffset(1772);
  mpu.setXGyroOffset(105);
  mpu.setYGyroOffset(-152);
  mpu.setZGyroOffset(32);
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
  
  AcX = AcX_raw / 2048;
  AcY = AcY_raw / 2048;
  AcZ = AcZ_raw / 2048;
  GyX = GyX_raw / 16.4;
  GyY = GyY_raw / 16.4;
  GyZ = GyZ_raw / 16.4;

//  float dado_filtrado = filter.addData(GyZ);
  dado_filtrado = filtro.addData(GyZ);
  myPID.Compute();
  myservo.write(10 * Output + 90); // Output ampliado em 10x
  delay(1);

  Serial.print("GyZ = "); Serial.print(GyZ);
  Serial.print(" | GyZ filtrado = "); Serial.print(dado_filtrado);
  Serial.print(" | Servo Position = "); Serial.println(10 * Output + 90);
}
