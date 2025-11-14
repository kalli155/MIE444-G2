#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Create the sensor object
Adafruit_VL53L0X sensors[5];



const int xshut1 = 14; //
const int xshut2 = 27; 
const int xshut3 = 32; 
const int xshut4 = 33; 
const int xshut5 = 34; //block sensor

// Number of sensors
const int NUM_SENSORS = 5;



void setup() {
  Serial.begin(115200);
  Wire.begin(25, 26); // initializes sda/scl

  //initialize the xshunt
  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  pinMode(xshut3, OUTPUT);
  pinMode(xshut4, OUTPUT);
  pinMode(xshut5, OUTPUT);
  
  //ensure all sensors are off initailly
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);
  digitalWrite(xshut3, LOW);
  digitalWrite(xshut4, LOW);
  digitalWrite(xshut5, LOW);

  delay (10);
  //initialize each sensor
  digitalWrite(xshut1, HIGH);
  delay(10);
  sensors[0].begin(0x30, false, &Wire);

  digitalWrite(xshut2, HIGH);
  delay(10);
  sensors[1].begin(0x31, false, &Wire);

  digitalWrite(xshut3, HIGH);
  delay(10);
  sensors[2].begin(0x32, false, &Wire);

  digitalWrite(xshut4, HIGH);
  delay(10);
  sensors[3].begin(0x33, false, &Wire);

  digitalWrite(xshut5, HIGH);
  delay(10);
  sensors[4].begin(0x34, false, &Wire);
}

///function to get one measuremnt
uint16_t getMeasure( int senseNum) {
  
  int index = senseNum - 1;

  VL53L0X_RangingMeasurementData_t measure;
  sensors[index].rangingTest(&measure, false); 

  return (int16_t)measure.RangeMilliMeter;
}

//function for print all sensors
void printAll() {
  for (int i = 1; i <= NUM_SENSORS; i++) {
    int16_t d = getMeasure(i);

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");

    if (d >= 0) Serial.print(d);
    else Serial.print("Out of range");

    Serial.print(" mm   ");
  }
  Serial.println();
}

void loop() {
  
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();   // remove spaces/newlines

    // r = read ALL
    if (cmd == "r") {
      printAll();
    }

    else if (cmd.startsWith("r") && cmd.length() == 2) {
      int sensorNum = cmd.substring(1).toInt();  // convert "1" -> 1

      if (sensorNum >= 1 && sensorNum <= NUM_SENSORS) {
        int16_t d = getMeasure(sensorNum);

        Serial.print("Sensor ");
        Serial.print(sensorNum);
        Serial.print(" = ");

        if (d >= 0) Serial.print(d);
        else Serial.print("Out of range");

        Serial.println(" mm");
      } 
    }
 
  }
  
}
