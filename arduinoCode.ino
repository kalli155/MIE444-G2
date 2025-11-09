// Arduino UNO code
int in1 = 12;
int in2 = 6;
int in3 = 7;
int in4 = 4;

int rightSpeedPin = 11;
int leftSpeedPin = 5;

volatile unsigned long encoder1Count = 0;
volatile unsigned long encoder2Count = 0;

void setup() {
  Serial.begin(9600); // Use the default hardware serial
  Serial.println("Arduino ready");
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(rightSpeedPin, OUTPUT);
  pinMode(leftSpeedPin, OUTPUT);

  analogWrite(rightSpeedPin, 128);
  analogWrite(leftSpeedPin, 128);

  analogWrite(rightSpeedPin, 128);
  analogWrite(leftSpeedPin, 128);


  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  // Attach interrupts on both pins
  attachInterrupt(digitalPinToInterrupt(2), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(3), encoder2ISR, RISING);
}

void motorA_forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void motorA_reverse() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void motorB_forward() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void motorB_reverse() {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void brake(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void moveForward() {
  motorA_forward();
  motorB_forward();
}

void moveReverse() {
  motorA_reverse();
  motorB_reverse();
}

void turnRight() {
  motorA_forward();
  motorB_reverse();
}

void turnLeft() {
  motorA_reverse();
  motorB_forward();
}

void loop() {
  // Read counts safely (with interrupts briefly disabled)
  noInterrupts();
  unsigned long count1 = encoder1Count;
  unsigned long count2 = encoder2Count;
  interrupts();

  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // read line
    input.trim(); // remove spaces/newlines

    if (input.length() == 0) return;

    // Split the input
    char cmd = input.charAt(0);
    int spaceIndex = input.indexOf(' ');
    int duration = 0;

    if (spaceIndex > 0) {
      duration = input.substring(spaceIndex + 1).toInt();
    }

    Serial.print("Command: ");
    Serial.print(cmd);
    Serial.print(" | Duration: ");
    Serial.println(duration);

    // Execute command
    switch (tolower(cmd)) {
      case 'w':
        moveForward();
        Serial.println("Forward");
        break;
      case 's':
        moveReverse();
        Serial.println("Backward");
        break;
      case 'a':
        turnLeft();
        Serial.println("Left");
        break;
      case 'd':
        turnRight();
        Serial.println("Right");
        break;
      case 'x':
        brake();
        Serial.println("Stop");
        break;
      default:
        Serial.println("Invalid command");
        return;
    }

    // Run for specified duration, then stop
    if (duration > 0) {
      delay(duration);
      brake();
      Serial.println("Stopped after delay");
    }
  }
}
