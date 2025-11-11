char val = 0; //holds ascii from serial line
void setup()
{
 Serial.begin(9600);
 Serial.println("Rover Uno is alive!");
}
void loop()
{
 if (Serial.available())
 {
 val = Serial.read();
 if (val == 'w')
 {
 // drive forward
 Serial.println("forward");
 }
 else if(val == 's')
 {
 // drive backward
 Serial.println("backward");
 
}
 else if(val == 'd')
 {
 // drive right
 Serial.println("right");
 }
 else if(val == 'a')
 {
 // drive left
 Serial.println("left");
 }
 }
}
