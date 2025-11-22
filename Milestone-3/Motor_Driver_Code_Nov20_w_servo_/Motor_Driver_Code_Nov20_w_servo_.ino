#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <math.h>

/* ===================== MOTOR DRIVER PINS ===================== */
#define in1 12   // Right motor dir A
#define in2 6    // Right motor dir B
#define in3 4    // Left  motor dir A
#define in4 7    // Left  motor dir B

#define rightSpeedPin 5     // PWM (Right)
#define leftSpeedPin  11    // PWM (Left)


// Servo set-up
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


/* ===================== ENCODERS ===================== */
#define rightEncoderPin 2   // INT0 (Uno)
#define leftEncoderPin  3   // INT1 (Uno)

volatile unsigned long encoderRightCount = 0;
volatile unsigned long encoderLeftCount  = 0;

void encoderRightISR() { encoderRightCount++; }
void encoderLeftISR()  { encoderLeftCount++;  }

/* ===================== SPEED CONTROL STATE ===================== */
float rightSpeed = 90.0f;       // PWM duty (0-255)
float leftSpeed  = 90.0f;
const float MIN_PWM = 60.0f;    // raise if a wheel sticks
bool moving = false;

/* ===================== WHEEL / ROBOT CONSTANTS ===================== */
const float WHEEL_DIAMETER = 2.6160;    // your wheel
const float CPR_OUTPUT        = 1346.2f;  // counts per wheel revolution (after gearbox)
const float TRACK_WIDTH_CM    = 16.5f;   // <--- set your robot's wheelbase (wheel-to-wheel)

/* Helpers */
inline float cmToIn(float cm) { return cm / 2.54f; }

/* ===================== FREERTOS TIMER ===================== */
TimerHandle_t xAdjustSpeeds;
TimerHandle_t xPrintStatus;

/* ===================== PROTOTYPES ===================== */
// Motion primitives
void rotateForward();
void rotateBackward();
void rotateRight();
void rotateLeft();
void brake();

// Balancer + telemetry
void adjustSpeeds(TimerHandle_t);
void printStatus(TimerHandle_t);

// Command parsing
void processCommand(const String& input, char &letter, float &number);

// Distance/angle conversions
long inchesToTicks(float inches);
long degreesToTicks(float deg);

// Executors that block until setpoint reached (cooperatively)
long runLinearTicks(long targetTicks, bool backward);
long runTurnTicks(long targetTicks, bool rightTurn);

// Serial reader task (USB Serial)
void taskSerial(void*);

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(9600);           // USB debug
  while (!Serial) {;}

  Serial.println(F("Rover ready. Commands over Serial: w N (in), b N (in), r N (deg), l N (deg), s 0"));

  // Motor control pins
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  // PWM pins
  pinMode(rightSpeedPin, OUTPUT);
  pinMode(leftSpeedPin,  OUTPUT);
  analogWrite(rightSpeedPin, (int)lroundf(rightSpeed));
  analogWrite(leftSpeedPin,  (int)lroundf(leftSpeed));

  //servo
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);



  // Encoders
  pinMode(rightEncoderPin, INPUT_PULLUP);
  pinMode(leftEncoderPin,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin),  encoderLeftISR,  CHANGE);

  // Timers: balancer @50ms, status @500ms
  xAdjustSpeeds = xTimerCreate("Adjust", pdMS_TO_TICKS(50),  pdTRUE, (void*)1, adjustSpeeds);
  xPrintStatus  = xTimerCreate("Status", pdMS_TO_TICKS(500), pdTRUE, (void*)0, printStatus);
  servoTimerHandle = xTimerCreate("servoTimer",pdMS_TO_TICKS(SERVO_FRAME_MS),pdTRUE,NULL,vServoTimerCallback );
  xTimerStart(xAdjustSpeeds, 0);
  xTimerStart(xPrintStatus,  0);
  xTimerStart(servoTimerHandle,  0);

  // Serial command reader
  xTaskCreate(taskSerial, "Serial", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
  

  
}

//servo functions
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




void loop() { /* unused under FreeRTOS */ }

/* ===================== MOTION HELPERS ===================== */
void rotateForward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}
void rotateBackward() {
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
}
void rotateLeft() { // spin in place
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
}
void rotateRight() { // spin in place
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}
void brake() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  moving = false;
}

/* ===================== STRAIGHT-LINE BALANCER ===================== */
static long prevR = 0, prevL = 0;
static float iTerm = 0.0f;

void adjustSpeeds(TimerHandle_t) {
  if (!moving) return;

  noInterrupts();
  unsigned long rCnt = encoderRightCount;
  unsigned long lCnt = encoderLeftCount;
  interrupts();

  long dR = (long)rCnt - prevR;
  long dL = (long)lCnt - prevL;
  prevR = (long)rCnt;
  prevL = (long)lCnt;

  long e = dR - dL;                 // positive => right faster
  const float Kp = 0.40f;           // tune 0.3–0.8
  const float Ki = 0.00f;           // add 0.005–0.015 when stable
  const float iMax = 25.0f;

  if (abs(e) < 1) e = 0;            // deadband
  iTerm += Ki * (float)e;
  if (iTerm > iMax)  iTerm = iMax;
  if (iTerm < -iMax) iTerm = -iMax;

  float u = Kp * (float)e + iTerm;  // correction

  rightSpeed -= u;                  // slow fast side
  leftSpeed  += u;                  // speed slow side

  if (rightSpeed > 255) rightSpeed = 255;
  if (leftSpeed  > 255) leftSpeed  = 255;
  if (rightSpeed < MIN_PWM) rightSpeed = MIN_PWM;
  if (leftSpeed  < MIN_PWM) leftSpeed  = MIN_PWM;

  analogWrite(rightSpeedPin, (int)lroundf(rightSpeed));
  analogWrite(leftSpeedPin,  (int)lroundf(leftSpeed));
}

void printStatus(TimerHandle_t) {
  static uint16_t line = 0;
  if ((line++ % 20) == 0) {
    Serial.println(F("Moving\tLCount\tLS\tRS\tRCount"));
  }
  noInterrupts();
  unsigned long rCnt = encoderRightCount;
  unsigned long lCnt = encoderLeftCount;
  interrupts();

  Serial.print(moving ? F("true") : F("false")); Serial.print('\t');
  Serial.print(lCnt);  Serial.print('\t');
  Serial.print((int)lroundf(leftSpeed));  Serial.print('\t');
  Serial.print((int)lroundf(rightSpeed)); Serial.print('\t');
  Serial.println(rCnt);
}


/* ===================== COMMAND PARSER ===================== */
void processCommand(const String& input, char &letter, float &number) {
  String cmd = input; cmd.trim();
  letter = 0; number = 0.0f;
  if (cmd.length() == 0) return;

  letter = cmd.charAt(0);
  int sp = cmd.indexOf(' ');
  if (sp > 0) number = cmd.substring(sp + 1).toFloat();
}

/* ===================== CONVERSIONS ===================== */
long inchesToTicks(float inches) {
  // const float wheelDiameterIn = cmToIn(WHEEL_DIAMETER_CM);
  // const float circumferenceIn = PI * wheelDiameterIn;
  // float ticks = (inches * CPR_OUTPUT) / circumferenceIn;

  float  ticks = CPR_OUTPUT* inches / ( WHEEL_DIAMETER * PI);
  return lroundf(ticks);
}
// In-place turn: each wheel travels arc = (pi * track_width) * (deg/360)
long degreesToTicks(float deg) {
  const float arc_cm = PI * TRACK_WIDTH_CM * (deg / 360.0f);
  const float arc_in = cmToIn(arc_cm);
  return inchesToTicks(arc_in);
}




/* ===================== EXECUTORS (BLOCK UNTIL DONE) ===================== */
// Straight move until BOTH encoders reach target
long runLinearTicks(long targetTicks, bool backward) {
  unsigned long l,r ;
  noInterrupts();
  encoderLeftCount = 0;
  encoderRightCount = 0;
  interrupts();
  prevL = 0; prevR = 0; iTerm = 0;

  // ensure PWM applied
  rightSpeed = max(rightSpeed, MIN_PWM);
  leftSpeed  = max(leftSpeed,  MIN_PWM);
  analogWrite(rightSpeedPin, (int)lroundf(rightSpeed));
  analogWrite(leftSpeedPin,  (int)lroundf(leftSpeed));

  moving = true;  // start motion tracking

  if (backward) rotateBackward(); else rotateForward();

  while (true) {
    noInterrupts();
    l = encoderLeftCount;
    r = encoderRightCount;
    interrupts();
    
    if (l >= (unsigned long)targetTicks && r >= (unsigned long)targetTicks) {
      break;
    }

    if (Serial.available()) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // cooperative wait
  }
  brake();
  moving = false; // stop motion tracking
  return max( r , l );
}

// In-place turn until BOTH encoders reach arc ticks
long runTurnTicks(long targetTicks, bool rightTurn) {
  unsigned long l,r ;
  noInterrupts();
  encoderLeftCount = 0;
  encoderRightCount = 0;
  interrupts();
  prevL = 0; prevR = 0; iTerm = 0;

  rightSpeed = max(rightSpeed, MIN_PWM);
  leftSpeed  = max(leftSpeed,  MIN_PWM);
  analogWrite(rightSpeedPin, (int)lroundf(rightSpeed));
  analogWrite(leftSpeedPin,  (int)lroundf(leftSpeed));

  moving = true; // start motion tracking

  if (rightTurn) rotateRight(); else rotateLeft();

  while (true) {
    noInterrupts();
    l = encoderLeftCount;
    r = encoderRightCount;
    interrupts();
    
    if (l >= (unsigned long)targetTicks && r >= (unsigned long)targetTicks) {
      break;
    }

    if (Serial.available()) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  brake();
  moving = false; // stop motion tracking
  return max( r , l );
}

/* ===================== USB SERIAL TASK ===================== */
void taskSerial(void*) {
  Serial.setTimeout(50);
  for (;;) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n'); // newline-terminated
      char cmd; float val;
      processCommand(line, cmd, val);

      bool completed = false; // intializes 

      switch (cmd) {
        case 'w': { // forward inches
          long ticks = inchesToTicks(val);
          Serial.print(F("[SER] Forward ")); Serial.print(val);
          Serial.print(F(" in -> ")); Serial.print(ticks); Serial.println(F(" ticks"));
          completed = runLinearTicks(ticks, /*backward=*/false) >= ticks;
          break;
        }
        case 'b': { // backward inches
          long ticks = inchesToTicks(val);
          Serial.print(F("[SER] Backward ")); Serial.print(val);
          Serial.print(F(" in -> ")); Serial.print(ticks); Serial.println(F(" ticks"));
          completed = runLinearTicks(ticks, /*backward=*/true) >= ticks ;
          break;
        }
        case 'r': { // right turn degrees
          long ticks = degreesToTicks(val);
          Serial.print(F("[SER] Right ")); Serial.print(val);
          Serial.print(F(" deg -> ")); Serial.print(ticks); Serial.println(F(" ticks"));
          completed = runTurnTicks(ticks, /*rightTurn=*/true) >= ticks;
          break;
        }
        case 'l': { // left turn degrees
          long ticks = degreesToTicks(val);
          Serial.print(F("[SER] Left ")); Serial.print(val);
          Serial.print(F(" deg -> ")); Serial.print(ticks); Serial.println(F(" ticks"));
          completed = runTurnTicks(ticks, /*rightTurn=*/false) >= ticks;
          break;
        }
        case 'x': { // stop now
          Serial.println(F("[SER] Stop"));
          brake();
          completed = true;
          break;
        }
        case 's': { // servo control: s <angle>
          int angle = (int)val;   // val already parsed from your "s N" string
          if (angle < 0) angle = 0;
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
