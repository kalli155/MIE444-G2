#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <Arduino.h>
#include <Adafruit_VL53L0X.h>


// ---------------------- I2C COMMUNICATION ---------------------
#define SCL_PIN 25
#define SDA_PIN 26
// sda should be 25
// -------------------- SERIAL COMMUNICATION --------------------
BluetoothSerial SerialBT;         // Communication with Laptop
HardwareSerial mySerial(2);       // Communication with Arduino
bool MANUAL_CONTROL = false;

// ------------------------ TOF SENSOR --------------------------
Adafruit_VL53L0X lox1;
Adafruit_VL53L0X lox2;
Adafruit_VL53L0X lox3;
Adafruit_VL53L0X lox4;
Adafruit_VL53L0X lox0;

#define ADDR1 0x30
#define ADDR2 0x31
#define ADDR3 0x32
#define ADDR4 0x33

#define ADDR0 0x34

#define XSHUT1 18
#define XSHUT2 19
#define XSHUT3 22
#define XSHUT4 21

#define XSHUT0  2

// ------------------------ IR SENSOR PINS -----------------------

#define IR_SIGNAL_PIN 32

// -------------------- SENSOR DATA --------------------
float u1 = 0;
float u2 = 0;
float u3 = 0;
float u4 = 0;

float u0 = 0;

bool IRsense;

#define TOO_CLOSE_CRITERIA 3.5

bool leftTooClose = false;
bool rightTooClose = false;

bool localized = false;

// Create sonar objects

// -------------------- LED --------------------

#define LED_PIN 15
bool led = false;

// -------------------- TASKS AND TIMERS --------------------
TaskHandle_t SerialTaskHandle;
TaskHandle_t SendSensorData;
TimerHandle_t xPrintData;
TimerHandle_t xGetIRData;
TimerHandle_t xGetTOFData;
TimerHandle_t xControlLED;
TimerHandle_t xCheckTooClose;

// Function prototypes
void serialTask(void *pvParameters);
void sendSensorData(void *pvParameters);
void printData(TimerHandle_t xTimer);
void getIRData(TimerHandle_t xTimer);
void getTOFData(TimerHandle_t xTimer);
void controlLED(TimerHandle_t xTimer);
void checkTooClose(TimerHandle_t xTimer);

void scanI2C();

// -------------------- SETUP --------------------

void setupTOFSensors() 
{
  Serial.println("Initializing VL53L0X sensors...");

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);
  pinMode(XSHUT4, OUTPUT);
  pinMode(XSHUT0, OUTPUT);

  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(10);

  // keep all sensors in reset
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  digitalWrite(XSHUT4, LOW);
  digitalWrite(XSHUT0, LOW);
  
  delay(10);


  // --- SENSOR 1 ---
  digitalWrite(XSHUT1, HIGH);
  delay(20);
  if (!lox1.begin()) {
    Serial.println("Sensor 1 FAILED");
  }
  else {
    lox1.setAddress(ADDR1); 
    Serial.println("Sensor 1 OK");
  };

  scanI2C();
  delay(1000);

  // --- SENSOR 2 ---
  digitalWrite(XSHUT2, HIGH);
  delay(20);
  if (!lox2.begin()) {
    Serial.println("Sensor 2 FAILED");
  }
  else {
    lox2.setAddress(ADDR2); 
    Serial.println("Sensor 2 OK");
  };

  scanI2C();
  delay(1000);

  // --- SENSOR 3 ---
  digitalWrite(XSHUT3, HIGH);
  delay(20);
  if (!lox3.begin()) {
    Serial.println("Sensor 3 FAILED");
  } else {
    lox3.setAddress(ADDR3);   // 0x32
    Serial.println("Sensor 3 OK");
  };

  scanI2C();
  delay(1000);

  // --- SENSOR 4 ---
  digitalWrite(XSHUT4, HIGH);
  delay(20);
  if (!lox4.begin()) {
    Serial.println("Sensor 4 FAILED");
  } else {
    lox4.setAddress(ADDR4);   // 0x33
    Serial.println("Sensor 4 OK");
  };

  scanI2C();
  delay(1000);

  // --- SENSOR 0 ---
  digitalWrite(XSHUT0, HIGH);
  delay(20);
  if (!lox0.begin()) {
    Serial.println("Sensor 0 FAILED");
  } else {
    lox0.setAddress(ADDR0);   // 0x34
    Serial.println("Sensor 0 OK");
  }
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("Starting ESP32...");



  mySerial.begin(9600, SERIAL_8N1, 16, 17);   // RX=16, TX=17
  SerialBT.begin("ESP32_MIE444_Group2");
  Serial.println("Bluetooth device active. Pair with 'ESP32_MIE444_Group2'");

  setupTOFSensors();


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

  xGetIRData = xTimerCreate(
    "GetIRData",
    pdMS_TO_TICKS(100),
    pdTRUE,
    (void *)0,
    getIRData
  );

  xControlLED = xTimerCreate(
    "Control LED",
    pdMS_TO_TICKS(500),
    pdTRUE,
    (void *)0,
    controlLED
  );

  xGetTOFData = xTimerCreate(
    "Get TOF Readings",
    pdMS_TO_TICKS(550),
    pdTRUE,
    (void *)0,
    getTOFData
  );

  xPrintData = xTimerCreate(
    "Print Data",
    pdMS_TO_TICKS(1000),
    pdTRUE,
    (void*)0,
    printData
  );

  xCheckTooClose = xTimerCreate(
    "Check if too close",
    pdMS_TO_TICKS(1000),
    pdTRUE,
    (void*)0,
    checkTooClose
  );

  pinMode(IR_SIGNAL_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  xTimerStart(xPrintData, 0);
  xTimerStart(xGetIRData, 0);
  xTimerStart(xControlLED, 0);
  xTimerStart(xGetTOFData, 0);
  xTimerStart(xCheckTooClose, 0);

  scanI2C();
}

void scanI2C() 
{
  Serial.println("Scanning I2C bus...");
  uint8_t count = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("  → Device found at 0x");
      Serial.println(address, HEX);
      count++;
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Total devices: ");
    Serial.println(count);
  }

  Serial.println();
}

void loop() {}

// -------------------- SERIAL RELAY TASK --------------------
void serialTask(void *pvParameters) 
{
  for (;;) {

    // Forward from Bluetooth → Arduino
    if (SerialBT.available()) 
    {
      String cmd = SerialBT.readStringUntil('\n');
      cmd.trim(); // remove newline or spaces
      //Serial.print("BLUETOOTH COMMAND: ");
      Serial.println(cmd);
      if (cmd == "u1") 
      {
        SerialBT.println(u1);
        Serial.print("u1 called");
      } 
      else if (cmd == "u2") 
      {
        SerialBT.println(u2);
        Serial.print("u1 called");
      } 
      else if (cmd == "u3") 
      {
        SerialBT.println(u3);
      } 
      else if (cmd == "u4") 
      {
        SerialBT.println(u4);
      } 
      else if (cmd == "i0") 
      {
        SerialBT.println(IRsense);
      } 
      else if (cmd == "light") 
      {
        led = !led;
      } 
      else if (cmd == "manualData") 
      {
        SerialBT.print("\t");
        SerialBT.print(u1);
        SerialBT.print("\t\t");
        SerialBT.println(u3);

        SerialBT.print(u2); 
        SerialBT.print("\t\t");
        SerialBT.print(IRsense);

        SerialBT.print("\t\t");
        SerialBT.println(u4); 
      } 
      else if (cmd == "getData") 
      {
        SerialBT.print("u1: ");
        SerialBT.print(u1);
        
        SerialBT.print("u2: ");
        SerialBT.print(u2);

        SerialBT.print("u3: ");
        SerialBT.print(u3);

        SerialBT.print("u4: ");
        SerialBT.print(u4);

        SerialBT.print("u0: ");
        SerialBT.print(u0);

        SerialBT.print("i0: ");
        SerialBT.print(IRsense);

        SerialBT.println();
      } 
      else if (cmd == "localized")
      {
        localized = true;
      }
      else if (cmd == "not localized")
      {
        localized = false;
      }
      else 
      {
        Serial.println("Sending Command to Arduino");
        Serial.println("Invalid Command");
        mySerial.println(cmd);
        Serial.print("BT → Arduino: ");
        Serial.println(cmd);
      }
    }

    
    // Forward from Arduino → Bluetooth
     if (mySerial.available()) 
     {
      String feedback = mySerial.readStringUntil('\n');
      if (feedback == "Good" && SerialBT.available())
      {
        SerialBT.println("stopped");
      }
      //SerialBT.println(feedback);
      //Serial.print("Arduino → ");
    //  Serial.println(feedback);
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Prevent watchdog reset
  }
}

void getIRData(TimerHandle_t xTimer)
{
  IRsense = digitalRead(IR_SIGNAL_PIN);
}

float readAveraged(Adafruit_VL53L0X &sensor, int samples = 3)
{
  VL53L0X_RangingMeasurementData_t m;
  uint32_t sum = 0;
  int count = 0;

  for (int i = 0; i < samples; i++) {
    sensor.rangingTest(&m, false);
    if (m.RangeStatus != 4) {
      sum += m.RangeMilliMeter;
      count++;
    }
    delay(3);
  }

  if (count == 0) return -1;
  return (sum / count) / 10.0;
}


void getTOFData(TimerHandle_t xTimer)
{
  u1 = readAveraged(lox1);
  u2 = readAveraged(lox2, 1);
  u3 = readAveraged(lox3);
  u4 = readAveraged(lox4, 1);
  u0 = readAveraged(lox0);
}



void controlLED(TimerHandle_t xTimer){
  if (led)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }
}

void printData(TimerHandle_t xTimer){
  Serial.print("\t");
  Serial.print(u1);
  Serial.print("\t\t");
  Serial.println(u3);

  Serial.print(u2); 
  Serial.print("\t\t");
  Serial.print(IRsense);
  Serial.print("\t\t");
  Serial.println(u4); 

  Serial.print("\t\t");
  Serial.println(u0);

  Serial.println();

  //scanI2C();
}


void checkTooClose(TimerHandle_t xTimer)
{
  if (u2 > 0 && u2 < TOO_CLOSE_CRITERIA && localized)
  {
    mySerial.println("x");
    mySerial.println("q");
    Serial.println("lefttooclose");
    delay(100);
  }
  else if (u4 > 0 && u4 < TOO_CLOSE_CRITERIA && localized)
  {
    mySerial.println("x");
    mySerial.println("e");
    Serial.println("righttooclose");
    delay(100);
  }
}