#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

// ——— Configuración de pines ———
const int pinPot   = A0;   // Potenciómetro de feedback
const int pinServo = 4;    // PWM del servo

// ——— Objetos ———
Adafruit_MPU6050 mpu;
Servo miServo;

// ——— Parámetros de temporización ———
const unsigned long T_MUESTREO = 20;     // ms entre cada envío a Simulink
const unsigned long T_SERVO     = 1000; // ms para cambio de posición

// ——— Variables internas de tiempo ———
unsigned long t_prev_servo = 0;

// ——— Parámetros de servo ———
bool    servoPosAlta = true;
const int ANG_ALTA   = 120;
const int ANG_BAJA   = 60;

// ——— Filtro complementario eje X ———
float angAntX = 0;                   // ángulo previo [rad]
const float alpha  = 0.02;
float theta0x = 0;                   // sesgo sistemático IMU [°]
bool  sesgo_inicializado = false;


//cosas nuevas modificadas----------------------------------------------------------
const float theta_ref = 0.0;  // vertical


// Variables del controlador
float e_prev = 0.0;
float u_prev = 90.0;  // posición neutral del servo
float integral = 0.0;

// ganancias controlador pi
const float Kp = 0.794;
const float Ki = 7.94;
const float Ts = 0.01;  // 10 ms


bool unaVez = true;

void setup() {
  Serial.begin(115200);
  while (!mpu.begin()) {
    delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  
  miServo.attach(pinServo, 500, 2500);
  miServo.write(60);
  delay(5000);
  miServo.write(120);
  delay(800);
}

void loop() {
  // 1) Marcar inicio de ciclo
  unsigned long t_start = millis();

  // 2) Leer IMU y calcular ángulo filtrado
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gyroX = g.gyro.x; // [°/s]

  // Complementario
  float angG = angAntX + gyroX * (T_MUESTREO / 1000.0);
  float angA = atan2(a.acceleration.y, a.acceleration.z);
  float angF = (1 - alpha) * angG + alpha * angA;
  angAntX = angF;

  // Inicializar sesgo en la primera iteración
  if (!sesgo_inicializado) {
    theta0x = angF * 180.0 / PI;
    sesgo_inicializado = true;
  }
  float thetaX = angF * 180.0 / PI - theta0x +2; // [°]

  // 3) Leer potenciómetro (feedback)
  int valPot = analogRead(pinPot);
  float phiFeedback = map(valPot, 0, 1023, 0, 180);


  //// Calcular error
  float e =thetaX;

  // Integración
  integral = e * Ts;

  // Controlador PI
  float u = Kp * e + Ki * integral;

  // Saturar señal (por ejemplo, servo de 0 a 180 grados)
  u = constrain(u + 90, 0, 180);  // desplazamiento desde el centro

  // Enviar señal al servo
  miServo.write(u);

  // Actualizar estado
  e_prev = e;
  u_prev = u;

  delay(10);  // espera para mantener Ts = 0.01 s
  Serial.print("accion de control");
  Serial.println(u);
  Serial.print("tita:");
  Serial.println(thetaX);
  
}
