/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the i/j/k/real parts of the rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 9600 baud to serial monitor.
*/

#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080

#include <math.h>

BNO080 IMU1;
BNO080 IMU2;

struct IMUData{
  float i, j, k, w;
  float pitch, roll, yaw;
  float ax, ay, az;
  float gx, gy, gz;
};

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  //Find IMU1
  if (IMU1.begin(0x4A) == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  delay(100);

  //Find IMU2
  if (IMU2.begin(0x4B) == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  //Set Reports
  setReports(IMU1);
  setReports(IMU2);

  Serial.println(("Initialized"));
}


IMUData data1;
IMUData data2;

void loop()
{
  unsigned long timestamp = millis();
  if(getReports(IMU1, data1)){
    printReports(data1, timestamp, 1);
  }
  if (getReports(IMU2, data2)){
    printReports(data2, timestamp, 2);
  }
}

bool getReports(BNO080 &imu, IMUData &data){
    //Look for reports from the IMU
  if (imu.dataAvailable() == true)
  {
    float quatRadianAcc = 0;
    byte quatAcc = 0;
    imu.getQuat(data.i, data.j, data.k, data.w, quatRadianAcc, quatAcc);
    
    quaternionToEuler(data.i, data.j, data.k, data.w, data.roll, data.pitch, data.yaw);

    byte accAcc = 0;
    imu.getAccel(data.ax, data.ay, data.az, accAcc);

    byte gyroAcc = 0;
    imu.getGyro(data.gx, data.gy, data.gz, gyroAcc);
    return true;
  }else{
    return false;
  }
}

void printReports(IMUData &data, unsigned long &timestamp, int device){
  //Serial.print("IMU:"); 
  Serial.print(device);Serial.print(", ");
  //Serial.print("Time:"); 
  Serial.print(timestamp);Serial.print(", ");
  
  //print quaternion
//  Serial.print("I:"); Serial.print(data.i);Serial.print(", ");
//  Serial.print("j:"); Serial.print(data.j);Serial.print(", ");
//  Serial.print("k:"); Serial.print(data.k);Serial.print(", ");
//  Serial.print("w:"); Serial.print(data.w);Serial.print(", ");

  //Print Euler
  //Serial.print("Yaw:"); 
  Serial.print(data.yaw);Serial.print(", ");
  //Serial.print("Pitch:"); 
  Serial.print(data.pitch);Serial.print(", ");
  //Serial.print("Roll:"); 
  Serial.print(data.roll);Serial.print(", ");

  //Print Raw Accel
  //Serial.print("ax:"); 
  Serial.print(data.ax);Serial.print(", ");
  //Serial.print("ay:"); 
  Serial.print(data.ay);Serial.print(", ");
  //Serial.print("az:"); 
  Serial.print(data.az);Serial.print(", ");
  
  //Print Raw Gyro
  //Serial.print("gx:"); 
  Serial.print(data.gx);Serial.print(", ");
  //Serial.print("gy:"); 
  Serial.print(data.gy);Serial.print(", ");
  //Serial.print("gz:"); 
  Serial.print(data.gz);//Serial.print(", ");
  
  Serial.println();
}

void quaternionToEuler(float i, float j, float k, float r, float& roll, float& pitch, float& yaw) {
  // Normalize the quaternion (optional but good practice)
  float norm = sqrt(i*i + j*j + k*k + r*r);
  i /= norm;
  j /= norm;
  k /= norm;
  r /= norm;

  // Roll (x-axis rotation)
  float sinr_cosp = 2 * (r * i + j * k);
  float cosr_cosp = 1 - 2 * (i * i + j * j);
  roll = atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  float sinp = 2 * (r * j - k * i);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // Yaw (z-axis rotation)
  float siny_cosp = 2 * (r * k + i * j);
  float cosy_cosp = 1 - 2 * (j * j + k * k);
  yaw = atan2(siny_cosp, cosy_cosp);

  // Convert from radians to degrees
  roll *= 180.0 / M_PI;
  pitch *= 180.0 / M_PI;
  yaw *= 180.0 / M_PI;
}

void setReports(BNO080 &imu){
  Serial.println("Enabling Reports");
  imu.enableRotationVector(20);
  imu.enableAccelerometer(30);
  imu.enableGyro(30);
}
