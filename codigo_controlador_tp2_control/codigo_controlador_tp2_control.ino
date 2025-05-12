#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

// ——— Configuración de pines ———
const int pinPot    = A0;   // Potenciómetro de feedback
  const int pinServo  = 3;    // PWM del s  ervo

// ——— Objetos ———
Adafruit_MPU6050 mpu;
Servo miServo;

// ——— Parámetros de temporización ———
const unsigned long T_MUESTREO = 20;    // ms entre cada envío a Simulink
unsigned long t_prev_muestreo  = 0;

const unsigned long T_SERVO     = 10000; // ms para cambio de posición
unsigned long t_prev_servo      = 0;
bool          servoPosAlta      = true;
const int     ANG_ALTA          = 120;
const int     ANG_BAJA          = 60;

// ——— Filtro complementario eje X ———
float angAntX = 0;           // ángulo previo
const float alpha  = 0.02;  
const float theta0x = -3;    // sesgo sistemático IMU




// ——— cosas del controlador———
float e = 0, e1 = 0, e2 = 0;
float u = 0, u1 = 0, u2 = 0;
// coeficientes de la ecuacion en diferencias
const float b0 = 0.1480;
const float b1 = 0.0141;
const float b2 = -0.1339;
const float a1 = 2.0;
const float a2 = -1.0;

const float ref = 0.0;  // Ángulo deseado del péndulo (vertical)

void setup() {
  Serial.begin(115200);
  while (!mpu.begin()) {
    delay(10);
  }
  // IMU
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Servo
  miServo.attach(pinServo, 500, 2500);
  miServo.write(ANG_BAJA);
}

void loop() {
  unsigned long t = millis();

  // ——— 1) Envío de datos a Simulink cada T_MUESTREO ———
  if (t - t_prev_muestreo >= T_MUESTREO) {
    t_prev_muestreo = t;

    // Leer IMU
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float gyroX = g.gyro.x;
    // filtro complementario
    float angG = angAntX + gyroX * (T_MUESTREO/1000.0);
    float angA = atan2(a.acceleration.y, a.acceleration.z);
    float angF = (1 - alpha) * angG + alpha * angA;
    angAntX = angF;
    float thetaX = angF * 180.0/PI - theta0x;

    // Leer potenciómetro (feedback)
    int valPot = analogRead(pinPot);
    float phiFeedback = map(valPot, 0, 1023, 0, 180);

    // Enviar 4 bytes de header + 3 floats (igual que tenías en tu bloque Serial Receive)
    Serial.write('a');
    Serial.write('b');
    Serial.write('c');
    Serial.write('d');
    Serial.write((byte*)&thetaX,       4);
    Serial.write((byte*)&phiFeedback,   4);
    float angGrados = 1;
    Serial.write((byte*)&angGrados, 4);

  }

e = ref - thetaX;

u = -a1 * u1 - a2 * u2 + b0 * e + b1 * e1 + b2 * e2;

// Actualizar historial
e2 = e1;
e1 = e;
u2 = u1;
u1 = u;

// Saturar y enviar al servo
u = constrain(u, -50, 50); // ajustá según lo que soporte el sistema
int angServo = map(u, -50, 50, 60, 120); // o el rango real del servo
miServo.write(angServo);



}
