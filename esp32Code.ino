// ESP32 code

#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_L3GD20_U.h>

#define SDA_PIN 26
#define SCL_PIN 25

Adafruit_L3GD20 gyro;

BluetoothSerial SerialBT;      // Bluetooth Serial object

int runTime = 1000;

HardwareSerial mySerial(2); // Use UART2

void setup() 
{
  Serial.begin(9600);    // For monitoring on your computer
  mySerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("ESP32 ready");

  Serial.println("Enter command: w/s/a/d/x");
  // Start Bluetooth
  SerialBT.begin("ESP32_MIE444_Group2");  // Name shown in Bluetooth
  Serial.println("Bluetooth device active. Pair with 'ESP32_MIE444_Group2'");
}

void loop() 
{
  // Forward from Bluetooth → Arduino
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    mySerial.println(cmd);
    Serial.print("BT → Arduino: ");
    Serial.println(cmd);
  }

  // Forward from Arduino → Bluetooth
  if (mySerial.available()) {
    String feedback = mySerial.readStringUntil('\n');
    SerialBT.println(feedback);
    Serial.print("Arduino → BT: ");
    Serial.println(feedback);
  }
}
