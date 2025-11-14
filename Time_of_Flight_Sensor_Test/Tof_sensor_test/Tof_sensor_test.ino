#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Create the sensor object
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("Starting VL53L0X test on Arduino Mega...");


  Wire.begin();
  
}

void loop() {
  
  if (Serial.available()> 0) {
    char c = Serial.read();

    if (c == 'r') {
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest (&measure, false);

      Serial.println(measure.RangeMilliMeter);
    }

    
  }
  delay(300);
  
}
