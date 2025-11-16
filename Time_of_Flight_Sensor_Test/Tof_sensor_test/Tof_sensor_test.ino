#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Create the sensor object
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting VL53L0X test on Arduino Mega...");


  Wire.begin();
  // Initialize the sensor
  if (!lox.begin()) {
    Serial.println("Failed to detect VL53L0X! Check wiring.");
    while (1);  // Stop here forever
  }


}

void loop() {
  
  if (Serial.available()> 0) {
    char c = Serial.read();

    if (c == 'r') {
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest (&measure, false);
      
      Serial.print("Distance: ");
      Serial.print(measure.RangeMilliMeter);
      Serial.println(" mm");
    }
    // Clear the input buffer
    while (Serial.available()) Serial.read();
    
  }
  
  
}
