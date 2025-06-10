// control observador

//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

//-------------------------Variables---------------------------------
const int servoPin = 4;
const int pinServo = 4;
const int potPin = A0;
//
//int potMin = 1023;
//int potMax = 0;

//int potMin = 137;
//int potMax = 900;
int potMin = 136;
int potMax = 843;

const int rangos[] = {0,180};
const int periodo = 20;             //20ms de periodo del servo
const int minAnchoPulso = 1210;     //minimo ancho de pulso (0 grados)
const int maxAnchoPulso = 5200;     //maximo ancho de pulso (180 grados)

Adafruit_MPU6050 mpu;
float anguloRotAnt = 0;
float alpha = 0.02;

Servo miServo;

const int periodoMuestreo = 20;   //cada cuanto tiempo se toman muestras en milisegundos
unsigned long tiempoMuestra = 0;

//int angulos[] = {120,110,90,70,100};
int i = 0;

const float angServoEq = 90;
const float angRef = 15;

float ang_error_ant2 = 0;
float ang_error_ant = 0;
float ang_error_act = 0;


const float theta0 = 0;// theta del pendulo en equilibrio lo que muesta la IMU
float theta   = 0;//theta actual
float theta_1 = 0;//theta -1
float u = 0;//u actual
float u_1=0; //u-1 anterior
float u_print=0; //para enviar a imprimir
float theta_e  = 0;//theta estimada
float theta_ref = 15;//entrada de referencia del sistema es un regulador

float xe[4]={0  , 0 , 0  , 0};// vector de estados estimados Xe
float xe_1[4]={0  , 0 , 0  , 0};// vector de estados estimados Xe-1 anterior
float K[4]={2.4233,   8.7420 ,  -0.3398   ,-1.0827};// vector K del controller -4+-j2 -70+-j70
float L[4]={-0.0215 , 0.0817 ,  0.1144   , 0.0923};   // vector L del observer

float A_fila1[4]={0.4172  ,  0.5451  , -0.0224 ,   0.0021};// matriz A del observador Aodd
float A_fila2[4]={0.0238 ,   0.0498 ,  -0.0014 ,  -0.0004};
float A_fila3[4]={2.4248 , -24.0329  , -0.1074 ,   0.3195};
float A_fila4[4]={4.8634 ,  11.1881  , -0.2824 ,  -0.1164};
    
float B_fila1[2]= {0.0281  , -0.0045}; //matriz del observador Bodd
float B_fila2[2]= {0.0020  ,  0.0101};
float B_fila3[2]= {1.2959  ,  0.2508};
float B_fila4[2]= {0.8495  , -0.0976};

float B_d[4]={0.0016 , 0.0130, 1.2458, 1.1725};

float C_fila1[4]={  -6.2601   ,94.1305         ,0         ,0};// matriz C del observer
 
float Cd[4]={-6.2601   ,94.1305         ,0         ,0};// armado de theta estimada

float ang_error_acum = 0;

float accion_actual = 0;
float accion_ant   = 0;

//float angMedido_ant = 0;

const float k_p = 1.1;
const float k_i = 0.35;
const float k_d = 0.005;

const float T = 0.02;
const float Ts = 0.02;



//--------------------------------------------------------------------

void setup() {

  //--------------------Configuracion del Servo-------------------------------------
  pinMode(servoPin, OUTPUT);
  //Configuro el timer1 en modo Fast PWM con frec = 50Hz y Duty Cycle = 10%
  TCCR1A = _BV(COM1A1) | _BV(WGM11);  // Fast PWM, clear OC1A on compare match, set OC1A at BOTTOM
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);  // Fast PWM, prescaler 8
  ICR1 = 39999;  // Frecuencia de PWM de 50 Hz (20 ms)
  delay(5000);
  Serial.begin(115200);
delay(1000);
  //------------------------Inicializacion de la IMU--------------------------------
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 5-10-21-44-94-184-260 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(1000);

//  //-------------------------Calibracion---------------------------------
  
  miServo.attach(pinServo, 500, 2500);
  miServo.write(75);
}

void loop() {
    sensors_event_t a, g, temp;

    unsigned long tiempoActual = millis();

    //if((tiempoActual - tiempoMuestra >= periodoMuestreo) && ((tiempoActual)<30000))
    if((tiempoActual - tiempoMuestra >= periodoMuestreo))
    {

      int potValor = analogRead(potPin);
      float angulo = map(potValor, potMin, potMax, 0, 180);
      float phi = map(potValor, potMin, potMax, 0, 180);
    
//    matlab_send(angulo);

      mpu.getEvent(&a, &g, &temp);
      float gyrox = g.gyro.x;
      float anguloRotG = anguloRotAnt + gyrox*periodoMuestreo/1000;
      //float anguloRotA = atan2(a.acceleration.y,sqrt(pow(a.acceleration.x,2) + pow(a.acceleration.z,2)));
      float anguloRotA = atan2(a.acceleration.y,a.acceleration.z);
      float anguloRot = (1-alpha)*anguloRotG + alpha*anguloRotA;
      
      float anguloMedido = anguloRot*180/PI;
      float theta = (anguloRot*180/PI-theta0);// theta0 el error sistematico de la IMU

      anguloRotAnt = anguloRot;

      control_moderno(theta, &theta_e, &u_1,&u_print);
      ang_error_acum = ang_error_ant - anguloMedido + angRef;
      Serial.print(theta);
      Serial.print(anguloMedido);
      Serial.print(" ,  ");
      Serial.print(theta_e);
      Serial.println(accion_ant);
      Serial.print(" ,  ");
      Serial.print(u_1);
      Serial.print(" ,  ");
      Serial.print(xe[0]);
      Serial.print(" ,  ");
      Serial.print(xe[1]);
      Serial.print(" ,  ");
      Serial.print(xe[2]);
      Serial.print(" ,  ");
      Serial.print(xe[3]);
      Serial.print(" ,  ");
      
      tiempoMuestra = tiempoActual;
      
    }

}

//-------------------Funciones Control-------------------------------



void control_moderno(float theta,  float* theta_e, float* u_1, float* u_print)
{
  // ang_error_act = 1 * (theta_ref - theta);
 // theta = -ang_error_act;
xe[0]= (A_fila1[0]*xe_1[0]+ A_fila1[1]*xe_1[1]+A_fila1[2]*xe_1[2]+A_fila1[3]*xe_1[3]) + B_fila1[0]* (*u_1) + B_fila1[1]*theta ;
xe[1]= (A_fila2[0]*xe_1[0]+ A_fila2[1]*xe_1[1]+A_fila2[2]*xe_1[2]+A_fila2[3]*xe_1[3]) + B_fila2[0]* (*u_1) + B_fila2[1]*theta ;
xe[2]= (A_fila3[0]*xe_1[0]+ A_fila3[1]*xe_1[1]+A_fila3[2]*xe_1[2]+A_fila3[3]*xe_1[3]) + B_fila3[0]* (*u_1) + B_fila3[1]*theta ;
xe[3]= (A_fila4[0]*xe_1[0]+ A_fila4[1]*xe_1[1]+A_fila4[2]*xe_1[2]+A_fila4[3]*xe_1[3]) + B_fila4[0]* (*u_1) + B_fila4[1]*theta ;
  
  float u = -1*( K[0]*xe[0]+K[1]*xe[1]+K[2]*xe[2]+K[3]*xe[3]);
  float theta_est = Cd[0]*xe[0]+Cd[1]*xe[1]+Cd[2]*xe[2]+Cd[3]*xe[3];
    
    xe_1[0]= xe[0] ;
    xe_1[1]= xe[1] ;
    xe_1[2]= xe[2] ;
    xe_1[3]= xe[3] ;
      
       
    *u_1=u ;
    *theta_e=theta_est++;
    *u_print=u ;
    u = -u;
    u = constrain(u + 90, 0, 180);  // desplazamiento desde el centro
    
    miServo.write(u);
      }//de la funcio

//----------------Funciones para movimiento del Servo----------------
void moverServo(int angulo)
{
  int anchoPulso = map(angulo, 0, 180, minAnchoPulso, maxAnchoPulso);
  miServo.write(anchoPulso);
}
