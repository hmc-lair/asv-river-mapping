#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Servo.h> 

/*
 ******************************************************************************
 * Servo Setup
 ******************************************************************************
 */
#define SERVOPIN 11
#define SERVO_MIN 1195
#define SERVO_MAX 1834 // turning right

// character arrays for storing commands
char currentCommand[20];
int commandIndex = 0;

Servo myservo;
int servoVal = 1600; // default is straight
bool shouldFlushPort = true;

/*
 ******************************************************************************
 * IMU Setup
 ******************************************************************************
 */
#define ST_LSM303DLHC_L3GD20        (0)
#define ST_LSM9DS1                  (1)
#define NXP_FXOS8700_FXAS21002      (2)

#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -83.24F, -83.75F, 78.78F };
// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.989,  0.023,  0.015 },
                                    {  0.023,  1.007, 0.016 },
                                    {  0.015, 0.016,  1.005 } };
float mag_field_strength        = 62.93F;
// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

Mahony filter;

/*
 ******************************************************************************
 */
 
void setup() {
  Serial.begin(9600);
  myservo.attach(SERVOPIN);

  // Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
  while(!Serial);
//  Serial.println("testing");
//  Serial.println(F("Adafruit AHRS Fusion Example")); Serial.println("");

  // Initialize the sensors.
  if(!gyro.begin()) {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.write("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }

  if(!accelmag.begin(ACCEL_RANGE_4G)) {
    Serial.write("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }

  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(10);
}

void loop(void) {
  /*
   ******************************************************************************
   * Heading update
   ******************************************************************************
   */
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Get new data samples
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);
  
  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);
  
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();
  char angle_buffer[10];
  Serial.write("$MAG, ");
  dtostrf(heading, 4, 2, angle_buffer);
  Serial.write(angle_buffer);
  Serial.write(",");
  dtostrf(pitch, 4, 2, angle_buffer);
  Serial.write(angle_buffer);
  Serial.write(",");
  dtostrf(roll,4,2,angle_buffer);
  Serial.write(angle_buffer);
  Serial.write("\n");

  /*
   ******************************************************************************
   * Servo control
   ******************************************************************************
   */
   
   // If there is an incoming reading...
  if (Serial.available() > 0) {
    while(Serial.available() > 0) {
      char currentChar = Serial.read();
      
      if (currentChar == '$') {
        shouldFlushPort = false;
        // if new command, empty the command buffer and set commandIndex to 0
        commandIndex = 0;
        memset(currentCommand, 0, 20);
        currentCommand[commandIndex] = currentChar;
      } 
      else if (currentChar == '@') {
         shouldFlushPort = true;
         // Execute the command when @ is recieved 
         // Serial.println(currentCommand);
         servoVal = atoi(currentCommand);
         sendServoCommand(servoVal);
      }
//      else if (shouldFlushPort) {
//        flushSerialPort();
//      }
      else {
         // populate the command until '$'
         currentCommand[commandIndex] = currentChar;
         commandIndex++;
      }
    }
  }
}

void sendServoCommand(int val) {
  Serial.write("$SERVO, ");
  Serial.write(val);
  Serial.write("\n");
  if (val >= SERVO_MIN && val <= SERVO_MAX) { // check if valid servo position
//    Serial.print("Writing to servo...");
//    Serial.println(val);
    myservo.writeMicroseconds(val);
  }
  return;
} 

void flushSerialPort() {
  // Flushing crap
  while (Serial.read() >= 0) ;
  do 
  {
    Serial.read();
  } while(Serial.available());
  delay(10);
}

