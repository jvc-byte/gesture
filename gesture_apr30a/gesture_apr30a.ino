#include <Servo.h>

Servo myServo;        // Create servo object to control a servo
int servoPin = 9;     // Servo is connected to pin 9
int lastAngle = -1;   // Initialize to an invalid value to ensure first command works
unsigned long lastAngleChange = 0;
const int debounceTime = 500;  // Minimum time between angle changes in milliseconds

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);  // Attaches the servo on pin 9
  myServo.write(0);          // Initialize to 0 degrees (door closed)
  delay(1000);               // Give servo time to reach starting position
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.read();  // Read the angle (expected to be 0 or 90)
    unsigned long currentTime = millis();
    
    // Ensure the angle is valid and different from last angle
    // Also make sure enough time has passed since the last change
    if ((angle == 0 || angle == 90) && 
        (angle != lastAngle) && 
        (currentTime - lastAngleChange > debounceTime)) {
      
      myServo.write(angle);    // Move servo to the received angle
      lastAngle = angle;
      lastAngleChange = currentTime;
      
      // Send confirmation back to Python
      Serial.print("Moved to ");
      Serial.print(angle);
      Serial.println(" degrees");
    }
  }
}