#include <Servo.h> 

Servo myservo;

void setup() 
{ 
  myservo.attach(13);
  delay(1000);
//  myservo.writeMicroseconds(1400);  // set servo to mid-point
//  delay(1000);
  myservo.writeMicroseconds(1600);  // set servo to mid-point
  delay(1000);
  myservo.writeMicroseconds(1700);
} 

void loop() {
  } 
