#include <Adafruit_BNO08x.h>
#include <math.h>

#define BNO08X_RESET -1

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;



void setup() {
  Serial.begin(115200);      // USB serial monitor
  while (!Serial) delay(10);
  Serial.println("Uart IMU");
  Serial2.begin(115200);     // UART for BNO085
  while (!Serial2) delay(10);
  Serial.println("After serial 1");
  //while (!bno08x.begin_UART(&Serial2)) {  // Requires a device with > 300 byte UART buffer!
  if(!bno08x.begin_UART(&Serial1)){
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    //while (1) { delay(10); }
    delay(10);
  }

  Serial.println("BNO08x connected over UART");
  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }
  setReports();
  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("Could not enable game vector");
  }
  if (! bno08x.enableReport(SH2_ACCELEROMETER, 10000)){
    Serial.println("Could not enable accelerometer report");
  }
  if (! bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)){
    Serial.println("Could not enable gyroscope report");
  }
}

void loop() {
  delay(10);
  Serial.println("loop");
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  while (!bno08x.getSensorEvent(&sensorValue)) {
    Serial.println("No new sensor event");
    return;
  }
  Serial.print("Sensor ID: ");
  Serial.println(sensorValue.sensorId);
  static float roll, pitch, yaw;
  static float ax, ay, az;
  static float gx, gy, gz;
  unsigned long timestamp = millis();
  switch (sensorValue.sensorId) {
    
    case SH2_ROTATION_VECTOR: {
      float i = sensorValue.un.rotationVector.i;
      float j = sensorValue.un.rotationVector.j;
      float k = sensorValue.un.rotationVector.k;
      float real = sensorValue.un.rotationVector.real;
    
//        quaternionToEuler(i, j, k, real, roll, pitch, yaw);
      Serial.print("Time:");Serial.print(timestamp); Serial.print(", ");
      Serial.print("Roll:"); Serial.print(roll); Serial.print(", ");
      Serial.print("Pitch:"); Serial.print(pitch); Serial.print(", ");
      Serial.print("Yaw:"); Serial.println(yaw);
      break;
    }
    case SH2_GYROSCOPE_CALIBRATED: {
      gx = sensorValue.un.gyroscope.x;
      gy = sensorValue.un.gyroscope.y;
      gz = sensorValue.un.gyroscope.z;
      Serial.print("Time:");Serial.print(timestamp); Serial.print(", ");
      Serial.print("Gx:"); Serial.print(gx); Serial.print(", ");
      Serial.print("Gy:"); Serial.print(gy); Serial.print(", ");
      Serial.print("Gz:"); Serial.println(gz);
      break;
    }
    case SH2_ACCELEROMETER: {
      ax = sensorValue.un.accelerometer.x;
      ay = sensorValue.un.accelerometer.y;
      az = sensorValue.un.accelerometer.z;
      Serial.print("Time:");Serial.print(timestamp); Serial.print(", ");
      Serial.print("Ax:"); Serial.print(ax); Serial.print(", ");
      Serial.print("Ay:"); Serial.print(ay); Serial.print(", ");
      Serial.print("Az:"); Serial.println(az);
      break;
    }
    
  }

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
