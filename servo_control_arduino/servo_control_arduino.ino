#include <Servo.h> 

#define SERVOPIN 11
#define SERVO_MIN 1195
#define SERVO_MAX 1834 // turning right

// character arrays for storing commands
char currentCommand[20];
int commandIndex = 0;

Servo myservo;
int servoVal = 1600; // default is straight

void setup() { 
  Serial.begin(9600);
  myservo.attach(SERVOPIN);
} 

void loop() {
  // If there is an incoming reading...
  if (Serial.available() > 0) {
    while(Serial.available() > 0) {
      char currentChar = Serial.read();
      
      if (currentChar == '$') {
        // if new command, empty the command buffer and set commandIndex to 0
        commandIndex = 0;
        memset(currentCommand, 0, 20);
        currentCommand[commandIndex] = currentChar;
      } 
      else if (currentChar == '@') {
         // Execute the command when @ is recieved 
         // Serial.println(currentCommand);
         servoVal = atoi(currentCommand);
         sendServoCommand(servoVal);
      }
      else {
         // populate the command until '$'
         currentCommand[commandIndex] = currentChar;
         commandIndex++;
      }
    }
  }
}

void sendServoCommand(int val) {
  if (val >= SERVO_MIN && val <= SERVO_MAX) { // check if valid servo position
//    Serial.print("Writing to servo...");
//    Serial.println(val);
    myservo.writeMicroseconds(val);
  }
  return;
} 
