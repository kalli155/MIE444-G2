#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <Arduino.h>
#include <NewPing.h>

// -------------------- SERIAL COMMUNICATION --------------------
BluetoothSerial SerialBT;         // Communication with Laptop
HardwareSerial mySerial(2);       // Communication with Arduino

// -------------------- ULTRASONIC SENSOR PINS --------------------
#define TRIGGER_PIN 23
#define ECHO_1 18
#define ECHO_2 19
#define ECHO_3 22
#define ECHO_4 21
#define MAX_DISTANCE 200  // Adjust based on expected range for faster readings


// ------------------------ IR SENSOR PINS -----------------------

#define IR_SIGNAL_PIN 32

// -------------------- SENSOR DATA --------------------
float u1 = 0;
float u2 = 0;
float u3 = 0;
float u4 = 0;

bool IRsense;

// Create sonar objects
NewPing sonar[4] = 
{
  NewPing(TRIGGER_PIN, ECHO_1, MAX_DISTANCE),
  NewPing(TRIGGER_PIN, ECHO_2, MAX_DISTANCE),
  NewPing(TRIGGER_PIN, ECHO_3, MAX_DISTANCE),
  NewPing(TRIGGER_PIN, ECHO_4, MAX_DISTANCE)
};

// -------------------- TASKS AND TIMERS --------------------
TaskHandle_t SerialTaskHandle;
TaskHandle_t SendSensorData;
TimerHandle_t xPrintData;
TimerHandle_t xGetUltrasonicData;
TimerHandle_t xGetIRData;

// Function prototypes
void serialTask(void *pvParameters);
void sendSensorData(void *pvParameters);
void printData(TimerHandle_t xTimer);
void getUltrasonicData(TimerHandle_t xTimer);
void getIRData(TimerHandle_t xTimer);

// -------------------- SETUP --------------------
void setup() 
{
  Serial.begin(9600);
  Serial.println("Starting ESP32...");

  mySerial.begin(9600, SERIAL_8N1, 16, 17);   // RX=16, TX=17
  SerialBT.begin("ESP32_MIE444_Group2");
  Serial.println("Bluetooth device active. Pair with 'ESP32_MIE444_Group2'");

  // Create FreeRTOS task for serial communication
  xTaskCreatePinnedToCore(
    serialTask,
    "SerialTask",
    4096,
    NULL,
    1,
    &SerialTaskHandle,
    1 // Core 1 recommended for comms
  );

  // xTaskCreatePinnedToCore(
  //   sendSensorData,
  //   "SendSensorData",
  //   4096,
  //   NULL,
  //   1,
  //   &SendSensorData,
  //   1
  // );

  // Create asynchronous timer (period = 50 ms)
  xGetUltrasonicData = xTimerCreate(
    "GetUltrasonicData",
    pdMS_TO_TICKS(50),
    pdTRUE,
    (void *)0,
    getUltrasonicData
  );

  xGetIRData = xTimerCreate(
    "GetIRData",
    pdMS_TO_TICKS(100),
    pdTRUE,
    (void *)0,
    getIRData
  );

  xPrintData = xTimerCreate(
    "Print Data",
    pdMS_TO_TICKS(1000),
    pdTRUE,
    (void*)0,
    printData
  );

  if (xGetUltrasonicData != NULL) {
    xTimerStart(xGetUltrasonicData, 0);
  } else {
    Serial.println("Failed to create ultrasonic timer!");
  }


  pinMode(IR_SIGNAL_PIN, INPUT);

  xTimerStart(xPrintData, 0);
  xTimerStart(xGetIRData, 0);
}

void loop() {}

// -------------------- SERIAL RELAY TASK --------------------
void serialTask(void *pvParameters) {
  for (;;) {
    // Forward from Bluetooth → Arduino
    if (SerialBT.available()) 
    {
      String cmd = SerialBT.readStringUntil('\n');
      cmd.trim(); // remove newline or spaces

      if (cmd == "u1") {
        SerialBT.println(u1);
      } else if (cmd == "u2") {
        SerialBT.println(u2);
      } else if (cmd == "u3") {
        SerialBT.println(u3);
      } else if (cmd == "u4") {
        SerialBT.println(u4);
      } else if (cmd == "i0") {
        SerialBT.println(IRsense);
      } else {
        Serial.println("Invalid Command");
      
        mySerial.println(cmd);
        Serial.print("BT → Arduino: ");
        Serial.println(cmd);
      }
    }

    // Forward from Arduino → Bluetooth
    if (mySerial.available()) {
      String feedback = mySerial.readStringUntil('\n');
      //SerialBT.println(feedback);
      //Serial.print("Arduino → BT: ");
      //Serial.println(feedback);
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Prevent watchdog reset
  }
}

// void sendSensorData(void *pvParameters)
// {
//   for (;;)
//   {
//     if (SerialBT.available())
//     {
//       String cmd = SerialBT.readStringUntil('\n');
//       cmd.trim(); // remove newline or spaces

//       if (cmd == "u1") {
//         SerialBT.println(u1);
//       } else if (cmd == "u2") {
//         SerialBT.println(u2);
//       } else if (cmd == "u3") {
//         SerialBT.println(u3);
//       } else if (cmd == "u4") {
//         SerialBT.println(u4);
//       } else if (cmd == "i0") {
//         SerialBT.println(IRsense);
//       } else {
//         Serial.println("Invalid Command");
//       }
//     }

//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// -------------------- ULTRASONIC TIMER CALLBACK --------------------
void getUltrasonicData(TimerHandle_t xTimer) 
{
  // Read each sensor sequentially using the shared trigger pin
  u1 = sonar[0].ping_cm();
  u2 = sonar[1].ping_cm();
  u3 = sonar[2].ping_cm();
  u4 = sonar[3].ping_cm();

  // Replace 0 readings with MAX_DISTANCE (means “no object detected”)
  if (u1 == 0) u1 = MAX_DISTANCE;
  if (u2 == 0) u2 = MAX_DISTANCE;
  if (u3 == 0) u3 = MAX_DISTANCE;
  if (u4 == 0) u4 = MAX_DISTANCE;

  // Print for debugging
  // Serial.printf("U1: %.0f cm | U2: %.0f cm | U3: %.0f cm | U4: %.0f cm\n", u1, u2, u3, u4);
}

void getIRData(TimerHandle_t xTimer)
{
  IRsense = digitalRead(IR_SIGNAL_PIN);
}


void printData(TimerHandle_t xTimer)
{
  Serial.print(u1); Serial.print("\t");
  Serial.print(u2); Serial.print("\t");
  Serial.print(u3); Serial.print("\t");
  Serial.print(u4); Serial.print("\t");
  Serial.print(IRsense); Serial.print("\t");

  Serial.println();
}
