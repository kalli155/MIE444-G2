#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX | TX
void setup()
{
 Serial.begin(9600);
 Serial.println("Software Serial Mode:");
 // Initiate Software Serial
 mySerial.begin(9600);
}
void loop()
{
 // Read from Software Serial and send to Arduino Serial Monitor
 if (mySerial.available())
 {
 delay(10);
 Serial.write(mySerial.read());
 }
 // Keep reading from Arduino Serial Monitor and send to Software Serial
 if (Serial.available())
 mySerial.write(Serial.read());
}
