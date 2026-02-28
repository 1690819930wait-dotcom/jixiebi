#include <Servo.h>

Servo baseServo; Servo frontServo; Servo rearServo; Servo clawServo;   
const int basePin = 9; const int frontPin = 10; const int rearPin = 11; const int clawPin = 6;
const int potPin = A0; 
int currentMode = 1; 

void setup() {
  Serial.begin(115200);
  baseServo.attach(basePin); frontServo.attach(frontPin);
  rearServo.attach(rearPin); clawServo.attach(clawPin);
  
  // 初始角度：底座120
  baseServo.write(120); 
  frontServo.write(75); 
  rearServo.write(100); 
  clawServo.write(0); 
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data.startsWith("[") && data.endsWith("]")) {
      data = data.substring(1, data.length() - 1);
      if (data == "G,OPEN") {
        clawServo.write(0); 
      } 
      else if (data == "G,CLOSE") {
        clawServo.write(90); 
      } 
      else {
        int firstComma = data.indexOf(',');
        int secondComma = data.indexOf(',', firstComma + 1);
        int thirdComma = data.indexOf(',', secondComma + 1);
        if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
          int bA = data.substring(0, firstComma).toInt();
          int fA = data.substring(firstComma + 1, secondComma).toInt();
          int rA = data.substring(secondComma + 1, thirdComma).toInt();
          currentMode = data.substring(thirdComma + 1).toInt();
          
          baseServo.write(constrain(bA, 0, 180));
          frontServo.write(constrain(fA, 30, 120)); 
          rearServo.write(constrain(rA, 45, 155)); 
        }
      }
    }
  }

  if (currentMode == 1) {
    int potVal = analogRead(potPin);
    int clawAngle = map(potVal, 0, 1023, 0, 90); 
    clawServo.write(clawAngle);
  }
}