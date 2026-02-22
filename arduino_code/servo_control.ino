#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 9; // Change to your servo pin

void setup() {
  Serial.begin(115200); // Must match ROS node baud rate
  myServo.attach(SERVO_PIN);
  myServo.write(90); // Start at center position
  Serial.println("Arduino Servo Ready");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read until newline
    input.trim();                                // Remove whitespace/\r

    int angle = input.toInt();

    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
      Serial.print("Moved to: ");
      Serial.println(angle);
    } else {
      Serial.println("Error: Angle out of range (0-180)");
    }
  }
}