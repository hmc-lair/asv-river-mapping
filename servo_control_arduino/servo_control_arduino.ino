#include <Servo.h> 

#define SERVOPIN 13

Servo myservo;
//Servo range: 1195-1834
//184 is turning right
void setup() 
{ 
  myservo.attach(SERVOPIN);
  delay(1000);
  myservo.writeMicroseconds(1834);  // set servo to mid-point
} 

void loop() {
} 
