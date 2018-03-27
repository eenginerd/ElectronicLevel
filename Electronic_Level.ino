/*
    The code for the filters used are based on Kristian Lauszus of TKJ Electronics'
    KalmanFilter library and the MPU6050 example sketch
*/
// Source: https://github.com/TKJElectronics/KalmanFilter


#include <MPU6050.h>
#include <Kalman.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4    // #define --> all values of OLED_RESET will be replaced with 4
Adafruit_SSD1306 display(OLED_RESET);
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
float alpha  = 0.93;
float beta = 0.07;
MPU6050 accelgyro;

// IMU Valus //
int16_t ax, ay, az;
int16_t gx, gy, gz;

//*** setting up filter variables and integration timing ***///

// creates kalman instances for x and y axes
Kalman kalmanX;
Kalman kalmanY;

//define the variables
float accX, accY, accZ, gyroX, gyroY, gyroZ;
float pitchAcc, rollAcc;
float gyroXangle, gyroYangle; // Angle calculate using the gyro only
float compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
float kalXmax = 0; // used to determine stability of x degree measurment
float kalXmin = 0; 
float kalXavglast = 0;
float kalXavg = 0;
volatile int count = 0;
uint32_t timer;
double dt = 0;
float dx = 0;
/// variables established to print to Serial or OLED
double gyroXanglePrint, gyroYanglePrint;
double compAngleXprint, compAngleYprint;
double kalAngleXprint, kalAngleYprint;
double pitchPrint, rollPrint;

//*** run setup routine ***//

void setup() {
  // initialize serial communication
  Serial.begin(115200);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  Wire.beginTransmission(0b1101000); //I2C address of chip
  Wire.write(0x1A); // access Register 1A
  Wire.write(0b00000000); // disable fsync and set accel and gyro filtering to 260 and 256
  Wire.endTransmission(); // end sending to that register

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //OLED init for 128x32 display size
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.display();

  // Set optimal offsets
  SetOffsets();

  // wait for sensor to stabillize
  // currently only method utilized to stabilize sensor is for it to run for a breif period of time before you start to read data
  Serial.println("Waiting for MPU 6050 to stabilize...");
  delay(200);
  Serial.println("Sucess");
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accX = ax;
  accY = ay;
  accZ = az;

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();

}

void loop() {

  processData();
  updateDisplay();
  delay(2); // 2ms delay so we dont burn through code

}

void processData() {

  /*
   *  processData() takes in IMU values and computes the angle off of each axis in deg
   *  The MPU-6050 is very sensitive to noise and/or drift over time, which is accounted for here
   *  a Kalman filter and a Complementary filter are employed seperatly to account for this
   *  The angles output by each filter can be compared to see which one is more accurate
   *  The Kalman filter output is what we want as it currently produces a more accurate output
   *  
  */

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accX = ax;
  accY = ay;
  accZ = az;
  gyroX = gx;
  gyroY = gy;
  gyroZ = gz;

  dt = (double)(micros() - timer) / 1000000; // Calculate dt for integration with units of sec
  timer = micros(); // record new timer value

  // roll and pitch are restricted to -pi and pi rads then converted to deg
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double gyroXrate = gyroX / 131.0; // Convert to deg/s -- > 131.0 is current resolution
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  // processes and handles when accel angle jumps between -180 and 180 deg
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } 
  else{
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;

  // Calculate the angle using a Complimentary filter
  compAngleX = alpha * (compAngleX + gyroXrate * dt) + beta * roll;
  compAngleY = alpha * (compAngleY + gyroYrate * dt) + beta * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  pitchPrint = pitch;
  rollPrint = roll;

  /* since this routine redefines some values at the beginning the values are passed to 
     new variables that will hold it for printing. 
     most of the variables used to calculate the angles are not defined globally  
  */
  compAngleXprint = compAngleX;
  compAngleYprint = compAngleY;
  gyroXanglePrint = gyroXangle;
  gyroYanglePrint = gyroYangle;
  kalAngleXprint = kalAngleX;
  kalAngleYprint = kalAngleY;

  /*
   * Checks to see if the kal X angle is stable --> dx < .25 deg 
   */
  if (kalAngleX > kalXmax)
  {kalXmax = kalAngleX;}
  if(kalAngleX < kalXmin)
  {kalXmin = kalAngleX;}
  count += 1;
  if (count == 50){
    kalXavglast = kalXmax - kalXmin;
    kalXmax = 0;
    kalXmin = 0;
  }
  if (count == 100){
    kalXavg = kalXmax - kalXmin;
    kalXmax = 0;
    kalXmin = 0;
    dx = kalXavglast - kalXavg;
    count = 0;
  }
  if (count > 105){
    count = 0;
  }
  
}

void CheckOffsets() {

  //*** Checks current offsets and prints them to Serial ***//

  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); //  prints current offsets
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); //
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); //
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); //
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); //
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); //
  Serial.print("\n");
}


void SetOffsets() {

  //***Reads current offsets and then updates them to values set by user ***//
  // currently have to hard code offsets by changing them below

  Serial.println("Updating internal sensor offsets...");
  Serial.println("Old offsets...");
  CheckOffsets();
  accelgyro.setXGyroOffset(36);
  accelgyro.setYGyroOffset(-26);
  accelgyro.setZGyroOffset(8);
  accelgyro.setXAccelOffset(-5);
  accelgyro.setYAccelOffset(-1859);
  accelgyro.setZAccelOffset(1767);
  Serial.println("Updated offsets...");
  // Print new offsets
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); 
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); 
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); 
  Serial.print(accelgyro.getXGyroOffset());  Serial.print("\t"); 
  Serial.print(accelgyro.getYGyroOffset());  Serial.print("\t"); 
  Serial.print(accelgyro.getZGyroOffset());  Serial.print("\t"); 
  Serial.print("\n");

}

void updateDisplay() {
  // updates OLED and Serial

  // OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

#if 1 // set to 1 if you want this printed to OLED
  display.print("X: ");
  display.println(kalAngleXprint);
  display.print("Y: ");
  display.println(kalAngleYprint);
  
    if(dx < .25){
      display.setCursor(0, 20);
      display.print("Stable"); 
      }
    if(dx >= .25){
      display.setCursor(0, 20);
      display.print("Unstable"); 
       }
#endif

#if 0 // set to 1 if you want this printed to OLED
  display.print("Pitch: ");
  display.println(pitchPrint);
  display.print("Roll: ");
  display.println(rollPrint);
#endif

  display.display();

  // SERIAL
#if 0 // set to 1 if you want pitch and roll values
  Serial.print("Pitch: ");
  Serial.print(pitchPrint); Serial.print("\t");
  Serial.print("  Roll: ");
  Serial.print(rollPrint); Serial.print("\t");

#endif

#if 1   // set to 0 to stop printing to serial 
  // pitch and roll are unfiltered values
  
  Serial.print("gyroX: ");
  Serial.print(gyroXanglePrint); Serial.print("\t");
  Serial.print("  gyroY: ");
  Serial.print(gyroYanglePrint); Serial.print("\t");
  Serial.print("  compX: ");
  Serial.print(compAngleXprint); Serial.print("\t");
  Serial.print("  compY: ");
  Serial.print(compAngleYprint); Serial.print("\t");
  Serial.print("  kalX: ");
  Serial.print(kalAngleXprint); Serial.print("\t");
  Serial.print("  kalY: ");
  Serial.print(kalAngleYprint); Serial.print("\t");
#endif

#if 1
  Serial.print(" LA: ");
  Serial.print(kalXavglast); Serial.print("\t");
  Serial.print(" A: ");
  Serial.print(kalXavg); Serial.print("\t");
  Serial.print(" dx: ");
  Serial.print(dx); Serial.print("\t");
  Serial.print(" LC: ");
  Serial.print(count); Serial.print("\t");  
  
#endif

  Serial.print("\r\n");

}



