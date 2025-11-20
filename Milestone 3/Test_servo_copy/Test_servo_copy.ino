#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <math.h>

//Servo set-up
#define SERVO_PIN 10
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_FRAME_MS 20
volatile uint16_t servoPulseUs = 1500; 
TimerHandle_t servoTimerHandle = NULL;

//servo function intiliaze
void setServoAngle(uint8_t angle);
void vServoTimerCallback(TimerHandle_t xTimer);
void processCommand(const String &line, char &cmd, float &val);
void taskSerial(void *pvParameters);

void setup() {
  Serial.begin(9600);
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  //servo timer
  servoTimerHandle = xTimerCreate(
    "servoTimer",
    pdMS_TO_TICKS(SERVO_FRAME_MS),
    pdTRUE,                // auto-reload
    NULL,
    vServoTimerCallback
    );

  // Create serial task
  xTaskCreate(
    taskSerial,
    "SerialTask",
    256,         // stack size; increase if needed
    NULL,
    1,           // priority
    NULL
  );
  xTimerStart(servoTimerHandle,  0);
}



void setServoAngle(uint8_t angle){

  // Map angle → pulse width
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  // Update shared variable
  taskENTER_CRITICAL();
  servoPulseUs = pulse;
  taskEXIT_CRITICAL();

}

void vServoTimerCallback(TimerHandle_t xTimer) {
  (void)xTimer;

  uint16_t pulse;

  // Copy current pulse atomically
  taskENTER_CRITICAL();
  pulse = servoPulseUs;
  taskEXIT_CRITICAL();

  // Generate the pulse (1–2 ms HIGH, rest of 20 ms is LOW)
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(SERVO_PIN, LOW);
}

void processCommand(const String& input, char &letter, float &number) {
  String cmd = input; cmd.trim();
  letter = 0; number = 0.0f;
  if (cmd.length() == 0) return;

  letter = cmd.charAt(0);
  int sp = cmd.indexOf(' ');
  if (sp > 0) number = cmd.substring(sp + 1).toFloat();
}


void taskSerial(void*) {
  Serial.setTimeout(50);
  for (;;) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n'); // newline-terminated
      char cmd; float val;
      processCommand(line, cmd, val);

      bool completed = false; // intializes 

      switch (cmd) {
        
        case 's': { // servo control: s <angle>
          int angle = (int)val;   // val already parsed from your "s N" string

          if (angle < 0)   angle = 0;
          if (angle > 180) angle = 180;

          setServoAngle((uint8_t)angle);

          Serial.print(F("[SER] Servo -> "));
          Serial.print(angle);
          Serial.println(F(" deg"));

          completed = true;  // if you’re using this flag
          break;
        }

        default:
          if (line.length() > 0) {
            Serial.print(F("[SER] Unknown: "));
            Serial.println(line);
          }
          break;
      }
      Serial.print(F("[RES] ") ) ;
      Serial.println(completed ? F("Good"): F("Bad") ); //ternary notation
      //if bad it did not reach the set encoder ticks
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
void loop() {

}


