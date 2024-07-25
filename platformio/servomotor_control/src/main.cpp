#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

// Creamos el objeto pca9685
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

// Definimos la freuencia de los servomotores según el datasheet en Hz
#define SERVO_FREQ 50

// Definimos los pines a los que están conectados cada uno de los servomotores
#define SER0 0 // Articulación 1 (base)
#define SER1 1 // Articulación 4 (gripper)
#define SER2 2 // Articulación 3 (elbow)
#define SER4 4 // Articulación 2 (shoulder)

// Definimos el ancho de pulso de trabajo para cada uno de los servomotores
// Para obtener estos dos valores hay que analizar individualmente cada uno de los servomotores (conviene hacerlo con el brazo desarmado para no forzar ninguna articulación)

// Servomotor en pin 0 (base)
#define SERVO0_MIN 100
#define SERVO0_MAX 480

// Servomotor en pin 1 (gripper)
#define SERVO1_MIN 145
#define SERVO1_MAX 540

// Servomotor en pin 2 (elbow)
#define SERVO2_MIN 100
#define SERVO2_MAX 460

// Servomotor en pin 4 (shoulder)
#define SERVO4_MIN 86
#define SERVO4_MAX 460 // Antes estaba en 410

// Variables a usar en el código
int pwm0;
int pwm1;
int pwm2;
int pwm4;

// Variables para guardar la posición angular actual de los servomotores
int currentAngle0 = 90;
int currentAngle1 = 120;
int currentAngle2 = 0;
int currentAngle4 = 90;

void moveServo0(int currentPosition, int targetPosition);
void moveServo1(int currentPosition, int targetPosition);
void moveServo2(int currentPosition, int targetPosition);
void moveServo4(int currentPosition, int targetPosition);

void setup()
{

  Serial.begin(115200);
  Serial.println("PCA9685 Servo Test");

  pca9685.begin();
  pca9685.setPWMFreq(SERVO_FREQ);

  // ANTES DE EJECUTAR EL CÓDIGO, POSICIONAR EL BRAZO EN LA POSICIÓN DE HOME

  pwm0 = map(90, 0, 180, SERVO0_MIN, SERVO0_MAX);  // Articulación 1 (base)
  pwm1 = map(120, 0, 180, SERVO1_MIN, SERVO1_MAX); // Articulación 4 (gripper)
  pwm2 = map(0, 0, 180, SERVO2_MIN, SERVO2_MAX);   // Articulación 3 (elbow)
  pwm4 = map(90, 0, 180, SERVO4_MIN, SERVO4_MAX);  // Articulación 2 (shoulder)

  pca9685.setPWM(SER0, 0, pwm0);
  pca9685.setPWM(SER1, 0, pwm1);
  pca9685.setPWM(SER2, 0, pwm2);
  pca9685.setPWM(SER4, 0, pwm4);

  delay(8000);
}

void loop()
{
  if (Serial.available())
  {
    int angles[5];

    // Los primeros 4 valores de la matriz corresponden a las variables articulares (q1, q2, q3 y q4), mientras que la última corresponde al tipo de movimiento
    // Si angles[5] = 0, entonces se está ejecutando la trayectoria
    // Si angles[5] 0 1, entonces se debe volver al home

    // Es importante hacer esta distinción porque el orden que se está usando para mover los servos en la trayectoria no es compatible con el retorno a home
    // (como q4 es la última que vuelve, se dibujaría accidentalmente en zonas indeseadas de la pizarra, por ejemplo)

    for (int i = 0; i < 5; i++)
    {
      angles[i] = Serial.parseInt();
    }

    if (angles[4] == 0)
    {
      moveServo0(currentAngle0, angles[0]); // Base
      delay(50);
      moveServo2(currentAngle2, angles[2]); // Elbow
      delay(50);
      moveServo4(currentAngle4, angles[1]); // Shoulder
      delay(50);
      moveServo1(currentAngle1, angles[3]); // Gripper
      delay(50);
    } else if (angles[4] == 1){
      moveServo1(currentAngle1, angles[3]); // Gripper
      delay(50);
      moveServo4(currentAngle4, angles[1]); // Shoulder
      delay(50);
      moveServo2(currentAngle2, angles[2]); // Elbow
      delay(50);
      moveServo0(currentAngle0, angles[0]); // Base
      delay(50);
    }

    currentAngle0 = angles[0];
    currentAngle1 = angles[3];
    currentAngle2 = angles[2];
    currentAngle4 = angles[1];

    Serial.println("DONE"); // Se manda "DONE" por puerto serie para que lo reciba el script de Python
  }
}

void moveServo0(int currentPosition, int targetPosition)
{
  if (currentPosition < targetPosition)
  {
    // Incremento la posición
    for (int pos = currentPosition; pos <= targetPosition; pos++)
    {
      pwm0 = map(pos, 0, 180, SERVO0_MIN, SERVO0_MAX);
      pca9685.setPWM(SER0, 0, pwm0);
      delay(50);
    }
  }
  else
  {
    // Decremento la posición
    for (int pos = currentPosition; pos >= targetPosition; pos--)
    {
      pwm0 = map(pos, 0, 180, SERVO0_MIN, SERVO0_MAX);
      pca9685.setPWM(SER0, 0, pwm0);
      delay(50);
    }
  }
}

void moveServo1(int currentPosition, int targetPosition)
{
  if (currentPosition < targetPosition)
  {
    // Incremento la posición
    for (int pos = currentPosition; pos <= targetPosition; pos++)
    {
      pwm1 = map(pos, 0, 180, SERVO1_MIN, SERVO1_MAX);
      pca9685.setPWM(SER1, 0, pwm1);
      delay(50);
    }
  }
  else
  {
    // Decremento la posición
    for (int pos = currentPosition; pos >= targetPosition; pos--)
    {
      pwm1 = map(pos, 0, 180, SERVO1_MIN, SERVO1_MAX);
      pca9685.setPWM(SER1, 0, pwm1);
      delay(50);
    }
  }
}

void moveServo2(int currentPosition, int targetPosition)
{
  if (currentPosition < targetPosition)
  {
    // Incremento la posición
    for (int pos = currentPosition; pos <= targetPosition; pos++)
    {
      pwm2 = map(pos, 0, 180, SERVO2_MIN, SERVO2_MAX);
      pca9685.setPWM(SER2, 0, pwm2);
      delay(50);
    }
  }
  else
  {
    // Decremento la posición
    for (int pos = currentPosition; pos >= targetPosition; pos--)
    {
      pwm2 = map(pos, 0, 180, SERVO2_MIN, SERVO2_MAX);
      pca9685.setPWM(SER2, 0, pwm2);
      delay(50);
    }
  }
}

void moveServo4(int currentPosition, int targetPosition)
{
  if (currentPosition < targetPosition)
  {
    // Incremento la posición
    for (int pos = currentPosition; pos <= targetPosition; pos++)
    {
      pwm4 = map(pos, 0, 180, SERVO4_MIN, SERVO4_MAX);
      pca9685.setPWM(SER4, 0, pwm4);
      delay(50);
    }
  }
  else
  {
    // Decremento la posición
    for (int pos = currentPosition; pos >= targetPosition; pos--)
    {
      pwm4 = map(pos, 0, 180, SERVO4_MIN, SERVO4_MAX);
      pca9685.setPWM(SER4, 0, pwm4);
      delay(50);
    }
  }
}
