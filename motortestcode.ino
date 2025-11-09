#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <math.h>

// MOTOR DRIVER PINS

#define in1 12
#define in2 6
#define in3 7
#define in4 4

#define rightSpeedPin 11
#define leftSpeedPin 5

// MOTOR PROPERTIES

#define rightEncoderPin 2
#define leftEncoderPin 3

volatile unsigned long encoderRightCount = 0;
volatile unsigned long encoderLeftCount = 0;

volatile unsigned long deltaCount = encoderRightCount - encoderLeftCount;

void encoderRightISR() { encoderRightCount++; deltaCount = encoderRightCount - encoderLeftCount;}
void encoderLeftISR() { encoderLeftCount++; deltaCount = encoderRightCount - encoderLeftCount;}


float rightSpeed = 70;
float leftSpeed = 70;

const int avgSpeedSetPoint = 70;

bool moving = false;

// ASYNCHRONOUS TIMERS

TimerHandle_t xAdjustSpeeds;
TimerHandle_t xPrintStatus;
TimerHandle_t xRotateForward;

// FUNCTION PROTOTYPEs

// Motor Motion
void rotateForward();
void rotateBackward();
void rotateRight();
void rotateLeft();
void brake();

// Asyncronous Tasks
void adjustSpeeds(TimerHandle_t xTimer);
void printStatus(TimerHandle_t xTimer);

void setup() 
{
  Serial.begin(9600);
  Serial.println("Starting Arduino...");

  // ESTABLISH MOTOR DRIVER CONTROL
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(rightSpeedPin, OUTPUT);
  pinMode(leftSpeedPin, OUTPUT);

  analogWrite(rightSpeedPin, rightSpeed);
  analogWrite(leftSpeedPin, leftSpeed);

  // ESTABLISH ENCODER COUNTS
  pinMode(rightEncoderPin, INPUT_PULLUP);
  pinMode(leftEncoderPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), encoderRightISR, RISING);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), encoderLeftISR, RISING);

  // ESTABLISH ASYNCHRONOUS TIMERS
  xPrintStatus = xTimerCreate("Print Current Status", pdMS_TO_TICKS(500), pdTRUE, (void *)0, printStatus);
  xAdjustSpeeds = xTimerCreate ("Adjust Speeds", pdMS_TO_TICKS(50), pdTRUE, (void *)1, adjustSpeeds);
  xRotateForward = xTimerCreate("rotate forward", pdMS_TO_TICKS(10000), pdTRUE,(void *)1, rotateForward);

  // START ASYNCHRONOUS TIMERS
  xTimerStart(xPrintStatus, 0);
  xTimerStart(xAdjustSpeeds, 0);
  xTimerStart(xRotateForward, 0);

  vTaskStartScheduler();
}

void loop() {}

// MOTOR MOTION FUNCTIONS

void rotateForward()
{
  Serial.println("rotating forward");
  // RIGHT MOTOR
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // LEFT MOTOR
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  moving = true;
}

void rotateBackward()
{
  // RIGHT MOTOR
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // LEFT MOTOR
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  moving = true;
}

void rotateRight()
{
  // RIGHT MOTOR
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // LEFT MOTOR
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  moving = true;
}

void rotateLeft()
{
  // RIGHT MOTOR
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // LEFT MOTOR
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  moving = true;
}

// ASYNCHRONOUS TIMERS

void printStatus(TimerHandle_t xTimer)
{
  // LABELS
  Serial.print("Moving\t");
  Serial.print("LCount\t");
  Serial.print("LSpeed\t");
  Serial.print("RSpeed\t");
  Serial.print("RCount\t");
  Serial.print("DeltaCount\t");
  Serial.println();

  // VALUES
  if (moving)
  {
    Serial.print("true");
  }
  else
  {
    Serial.print("false");
  }
  Serial.print("\t");
  Serial.print(encoderLeftCount);
  Serial.print("\t");
  Serial.print(int(leftSpeed));
  Serial.print("\t");
  Serial.print(int(rightSpeed));
  Serial.print("\t");
  Serial.print(encoderRightCount);
  Serial.print("\t");
  Serial.print(deltaCount);
  Serial.print("\t");
  Serial.println();
}

void adjustSpeeds(TimerHandle_t xTimer)
{
  if (moving)
  {
    float avgSpeed = (rightSpeed + leftSpeed) / 2.0;

    float k = 1.0 / 400.0;

    deltaCount = encoderRightCount - encoderLeftCount;
    /*
    if (deltaCount > 0)
    {
      rightSpeed = rightSpeed - ( k * deltaCount );
      leftSpeed = leftSpeed + ( k * deltaCount );
    }
    else if (deltaCount < 0)
    {
      rightSpeed = rightSpeed - ( k * deltaCount );
      leftSpeed = leftSpeed + ( k * deltaCount );
    }*/
    if (deltaCount > 0)
    {
      Serial.print("DeltaCount > 0\t");
      rightSpeed--;
      leftSpeed++;
      Serial.print("LS - " + String(leftSpeed) + "\t");
      Serial.println("RS - " + String(rightSpeed) + "\t");
    }
    else if(deltaCount < 0);
    {
      Serial.print("DeltaCount < 0\t");
      rightSpeed++;
      leftSpeed--;
      Serial.print("LS - " + String(leftSpeed) + "\t");
      Serial.print("RS - " + String(rightSpeed) + "\t");
      Serial.println("DC - " + String(deltaCount) + "\t");
    }
  }

  if (rightSpeed > 255)
  {
    rightSpeed = 255;
  }
  else if (rightSpeed < 0)
  {
    rightSpeed = 0;
  }
  else if (leftSpeed > 255)
  {
    leftSpeed = 255;
  }
  else if (leftSpeed < 0)
  {
    leftSpeed = 0;
  }

  // UPDATE SPEEDS
  analogWrite(rightSpeedPin, round(rightSpeed));
  analogWrite(leftSpeedPin, round(leftSpeed));
}