#include <Servo.h> 

Servo servo; 

void setup() {
  servo.attach(3); // Servo is attached to digital pin 3
}

void loop() {
  // Experiment with these values to achieve a wider range
  // servo.writeMicroseconds(0); // Corresponds to 0 degrees (try reducing this value)
  // delay(2000);
  // servo.writeMicroseconds(1500); // Corresponds to 90 degrees (middle position)
  // delay(2000);
  servo.writeMicroseconds(2500); // Corresponds to 180 degrees (try increasing this value)
  delay(2000);
}
