/*
  Aviso Legal y Licencia de Uso

  El siguiente código fue desarrollado por Bogar Gerardo Gallegos Arciniega
  como parte del proyecto VTOLPlane en la Universidad Marista de Guadalajara,
  con fecha 04 de abril de 2025.
  Parte del código fue basado en el trabajo realizado en 2012 por Jeff Rowberg
  para la recopilación de datos del MPU6050.

  Se concede permiso para usar, distribuir y modificar libremente este código,
  sujeto a las siguientes condiciones:

  Uso y distribución: Se permite la copia, modificación y redistribución del código,
  siempre que se mantenga este aviso legal en todas las versiones y derivados del software.

  Garantía: Este software se proporciona "tal cual", sin garantía de ningún tipo,
  expresa o implícita, incluyendo pero no limitada a garantías de comerciabilidad 
  o idoneidad para un propósito particular.

  Limitación de responsabilidad: El autor no se hace responsable de ningún daño,
  directo o indirecto, que pueda derivarse del uso de este software.

  Uso en aeronaves: Este software ha sido desarrollado para su uso en aeronaves de 
  radiocontrol (RC) destinadas a hobbies y experimentación. Queda estrictamente prohibido
  su uso en aeronaves tripuladas sin una evaluación y certificación adecuada, 
  y cualquier uso en dichos entornos será bajo la exclusiva responsabilidad del usuario.

  Al utilizar este código, acepta los términos y condiciones establecidos en este aviso

  /// Conexiones Esp32 

  MPU6050 devkit 1.0
  Board   Lolin         Description
  ======= ==========    ====================================================
  VCC     VU (5V USB)   5V
  GND     G             Ground
  SCL     D1 (GPIO22)   I2C clock
  SDA     D2 (GPIO21)   I2C data
  INT     D8 (GPIO15)   Interrupt pin

  Perifericos
  ======= ==========    =====================================================
  Servo1  GPIO16        Taileron Derecho
  Servo2  GPIO17        Taileron Izq.
  Servo3  GPIO18        Aleron Der.
  Servo4  GPIO19        Aleron Izq.
  Motor   GPIO23        Motor
  Extra   GPIO27        Periferico extra: Tren de aterrizaje, luces Paipi, et.

  Receptor
  ======= ==========    =====================================================
  CH 1    GPIO4         Joystick Izq. Eje Vertical
  CH 2    GPIO5         Joystick Izq. Eje Horizontal
  CH 3    GPIO13        Joystick Der. Eje Vertical
  CH 4    GPIO14        Joystick Der. Eje Horizontal
  CH 5    GPIO25        Palanca

*/

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#include <PID_v1.h>  
#include <ESP32Servo.h>

// Definición de variables PID
double error_Pitch, salidaPID_Pitch, anguloServo_Pitch;
double error_Roll, salidaPID_Roll, anguloServo_Roll;
double kp_Pitch = 2.0, ki_Pitch = 0.5, kd_Pitch = 1.0; 
double kp_Roll = 2.0, ki_Roll = 0.5, kd_Roll = 1.0; 
double EstablePitch, EstableRoll, Pitch, Roll, Yaw;
double anguloServo_Pitch_PID, anguloServo_Roll_PID;
double InputRoll, InputPitch, InputMotor, InputYaw, InputExtra;
double V_r, V_p;

// PID Pitch
PID Pitch_PID(&error_Pitch, &salidaPID_Pitch, 0, kp_Pitch, ki_Pitch, kd_Pitch, DIRECT);

// PID Roll
PID Roll_PID(&error_Roll, &salidaPID_Roll, 0, kp_Roll, ki_Roll, kd_Roll, DIRECT);

Servo servo_Pitch1;  
//Servo servo_Pitch2;  //En caso de tailerons
Servo servo_Roll1;
Servo servo_Roll2;  
Servo Motor;
Servo Extra; //Comentar o descomentar segun el tipo de periferico

//Lectura Receptor///

#define CH1_PIN  4
#define CH2_PIN  5
#define CH3_PIN  13
#define CH4_PIN  14
#define CH5_PIN  25

volatile unsigned long ch1_start, ch2_start, ch3_start, ch4_start, ch5_start;
volatile int ch1_value, ch2_value, ch3_value, ch4_value, ch5_value;

void IRAM_ATTR calc_ch1() {
  if (digitalRead(CH1_PIN) == HIGH) ch1_start = micros();
  else ch1_value = micros() - ch1_start;
}

void IRAM_ATTR calc_ch2() {
  if (digitalRead(CH2_PIN) == HIGH) ch2_start = micros();
  else ch2_value = micros() - ch2_start;
}

void IRAM_ATTR calc_ch3() {
  if (digitalRead(CH3_PIN) == HIGH) ch3_start = micros();
  else ch3_value = micros() - ch3_start;
}

void IRAM_ATTR calc_ch4() {
  if (digitalRead(CH4_PIN) == HIGH) ch4_start = micros();
  else ch4_value = micros() - ch4_start;
}

void IRAM_ATTR calc_ch5() {
  if (digitalRead(CH5_PIN) == HIGH) ch5_start = micros();
  else ch5_value = micros() - ch5_start;
}

float mapPWMtoRange(int pwm) {
  return ((float)(pwm - 1500) / 500.0) * 30.0;
}

///////////////////////////////////////////////////////////

// Es importante recordar que los angulos yaw/pitch/roll sufren de gimbal lock (para mas informacion revisar: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // Use el pin 2 para Arduino y la mayoria de tarjetas
#define LED_PIN 13 // (En Arduino es 13, Teensy es 11, Teensy++ es 6)
bool blinkState = false;

// MPU control/estado vars
bool dmpReady = false;  // Establecer como verdadero si la inicializacion de DMP es exitosa
uint8_t mpuIntStatus;   // almacena el byte de estado real de la interrupción del MPU
uint8_t devStatus;      // devuelve el estado después de cada operación del dispositivo (0 = éxito, !0 = error)
uint16_t packetSize;    // tamaño esperado del paquete DMP (por defecto es 42 bytes)
uint16_t fifoCount;     // cantidad de todos los bytes actualmente en el FIFO
uint8_t fifoBuffer[64]; // búfer de almacenamiento FIFO

// orientación/movimiento vars
Quaternion q;           // [w, x, y, z]         quaternion 
VectorInt16 aa;         // [x, y, z]            accel sensor mediciones
VectorInt16 aaReal;     // [x, y, z]            Mediciones de accel sensor sin gravedad
VectorInt16 aaWorld;    // [x, y, z]            Mediciones world-frame accel sensor 
VectorFloat gravity;    // [x, y, z]            vector de gravedad
float euler[3];         // [psi, theta, phi]    Contenedor de angulos de Euler
float ypr[3];           // [yaw, pitch, roll]   Contenedor de yaw/pitch/roll y gravedad

// Estructura de paquetes para InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===          Rutina de deteccion de interrupciones           ===
// ================================================================

volatile bool mpuInterrupt = false; 
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                       SETUP INICIAL                      ===
// ================================================================

void setup() {
//PID////
  Pitch_PID.SetMode(AUTOMATIC);  // Activar el PID
  Roll_PID.SetMode(AUTOMATIC);  

EstablePitch = 0;  //Calibrar segun corresponda
EstableRoll = 0;   //Calibrar segun corresponda
////////

//Servos//
 servo_Pitch1.attach(16);  // Conectar el servo al pin GPIO16
 //servo_Pitch2.attach(17);  //En caso de tailerons
 servo_Roll1.attach(18); 
 servo_Roll2.attach(19); 
 //////////

 //Motor o extra//
  Motor.attach(23);
  Extra.attach(27);
 ////////

 //Input del Receptor//
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  pinMode(CH4_PIN, INPUT);
  pinMode(CH5_PIN, INPUT);

  attachInterrupt(CH1_PIN, calc_ch1, CHANGE);
  attachInterrupt(CH2_PIN, calc_ch2, CHANGE);
  attachInterrupt(CH3_PIN, calc_ch3, CHANGE);
  attachInterrupt(CH4_PIN, calc_ch4, CHANGE);
  attachInterrupt(CH5_PIN, calc_ch5, CHANGE);
  ////////////////////////////////////////////

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial); 

    Serial.println(F("Inicilizando dispositivos I2C ..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Probando conexiónes del dispositivo..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 conexión exitosa") : F("MPU6050 Conexión fallida"));

    Serial.println(F("Inicializando DMP..."));
    devStatus = mpu.dmpInitialize();

    // Calibration values
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Activando DMP..."));
        mpu.setDMPEnabled(true);

        Serial.print(F("Permitiendo deteccion de interrupcion (Iterrupcion externa al Esp "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP Listo! Esperando primera interrupcion..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = Error en la carga inicial de la memoria
        // 2 = Fallo en la actualización de la configuración del DMP
        // (si va a fallar, generalmente el código será 1)
        Serial.print(F("Inicializacion DMP fallida (codigo "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // Confirar el LED en salida
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                   Loop General de FC                     ===
// ================================================================

void loop() {

//Asignacion de valores Roll_Yaw_Pitch//

    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        #ifdef OUTPUT_READABLE_QUATERNION
            // Mostrar los valores de los Quaterniones en formato de matriz: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // Mostrar angulos de Euler en grados
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // Mostrar angulos de Euler en grados
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // Mostrar aceleracion, ajustada para quitar la Gravedad
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // Mostrar los valores de los Quaterniones en formato de InvenSense Teapot demo:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, cicla en  0xFF intencionalmente
        #endif

        // El LED parpadea para indicar actividad
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

// Control PID //

  //Lectura Control
  InputPitch  = mapPWMtoRange(ch1_value);
  InputRoll   = mapPWMtoRange(ch2_value);
  InputMotor  = ch3_value;
  //InputRoll = ch4_value;  //En caso de tener timon
  InputExtra  = ch5_value;   //En este caso el tren de aterrizaje

  //Datos
  Yaw = ypr[0] * 180/M_PI;
  Pitch = ypr[1] * 180/M_PI;
  Roll = ypr[2] * 180/M_PI;

  //PID
  V_r = 0.2;  //Velocidad de Roll  (0 --> 1)
  V_p = 0.3;  //Velocidad de pitch (0 --> 1)

  EstableRoll = EstableRoll + InputRoll * V_r;

  if (InputPitch != 0) {    //Si el piloto inclina la aeronave, el punto de estabilidad se actualizara al punto en el que se oriente la misma
  EstablePitch = Pitch;
  }
  

  error_Pitch = Pitch - EstablePitch;
  error_Roll = Roll - EstableRoll;

  Pitch_PID.Compute();  // Calcular la salida del PID Pitch
  Roll_PID.Compute();  // Roll

  // PID --> Mapeo de servos
  anguloServo_Pitch_PID = map(salidaPID_Pitch, -30, 30, 0, 180);  
  anguloServo_Pitch_PID = constrain(anguloServo_Pitch_PID, 0, 180);  // Limitar el ángulo del servo

  anguloServo_Roll_PID = map(salidaPID_Roll, -30, 30, 0, 180);  
  anguloServo_Roll_PID = constrain(anguloServo_Roll_PID, 0, 180);  // Limitar el ángulo del servo

  //Asignacion de valores al servo
  servo_Pitch1.write(anguloServo_Pitch_PID + InputPitch * V_p);          // Mover el servo
  //servo_Pitch2.write(anguloServo_Pitch_PID + InputPitch * V_p);        //Descomentar en caso de tailerons
  servo_Roll1.write(anguloServo_Roll_PID + InputRoll);                   //Comentar para activar el modo GoalRoll
  servo_Roll2.write((anguloServo_Roll_PID + InputRoll)*(-1));            //Comentar para activar el modo GoalRoll

//  servo_Roll1.write(anguloServo_Roll_PID);                             //Descomentar para activar el modo GoalRoll
//  servo_Roll2.write(anguloServo_Roll_PID * (-1));                      //Descomentar para activar el modo GoalRoll

  Motor.write(InputMotor);
  Extra.write(InputExtra);

    }


}
