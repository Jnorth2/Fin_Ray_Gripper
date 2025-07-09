// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>
#include <math.h>
// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

Adafruit_BNO08x  bno1(BNO08X_RESET);
Adafruit_BNO08x bno2(BNO08X_RESET); 
sh2_SensorValue_t sensorValue;

int change_addr = 51;
bool is_bno1 = false;
bool is_bno2 = false;


void setup(void) {
  pinMode(change_addr, OUTPUT);
  digitalWrite(change_addr, HIGH);
  delay(100);
  
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno1.begin_I2C(0x4A, &Wire, 1)) {
    Serial.println("Failed to find BNO08x chip at 0x4A");
    //while (1) { delay(10); }
    delay(10);
  }
  else{
    Serial.println("Found BNO08x at 0x4A");
    is_bno1 = true;
    setReports(bno1);
  }

delay(1000);
  if(!bno2.begin_I2C(0x4B, &Wire, 2)){
    Serial.println("Failed to find BNO08x at 0x4B");
    //while(1) {delay(10);}
    delay(10);
  }
  else{
    Serial.println("Found BNO08x at 0x4B");
    is_bno2 = true;
    setReports(bno2);
  }
  
  //Serial.println("BNO08x Found!");

  if(is_bno1){
    for (int n = 0; n < bno1.prodIds.numEntries; n++) {
      Serial.print("Part ");
      Serial.print(bno1.prodIds.entry[n].swPartNumber);
      Serial.print(": Version :");
      Serial.print(bno1.prodIds.entry[n].swVersionMajor);
      Serial.print(".");
      Serial.print(bno1.prodIds.entry[n].swVersionMinor);
      Serial.print(".");
      Serial.print(bno1.prodIds.entry[n].swVersionPatch);
      Serial.print(" Build ");
      Serial.println(bno1.prodIds.entry[n].swBuildNumber);
    }
  }

  if(is_bno2){
    for (int n = 0; n < bno2.prodIds.numEntries; n++) {
      Serial.print("Part ");
      Serial.print(bno2.prodIds.entry[n].swPartNumber);
      Serial.print(": Version :");
      Serial.print(bno2.prodIds.entry[n].swVersionMajor);
      Serial.print(".");
      Serial.print(bno2.prodIds.entry[n].swVersionMinor);
      Serial.print(".");
      Serial.print(bno2.prodIds.entry[n].swVersionPatch);
      Serial.print(" Build ");
      Serial.println(bno2.prodIds.entry[n].swBuildNumber);
    }
  }

// setReports(bno1);

  Serial.println("Reading events");
  delay(1000);
}

// Here is where you define the sensor outputs you want to receive
void setReports(Adafruit_BNO08x bno) {
  Serial.println("Setting desired reports");
  if (! bno.enableReport(SH2_ROTATION_VECTOR, 20000)) {
     Serial.println("Could not enable game vector");
   }
  if (! bno.enableReport(SH2_ACCELEROMETER, 20000)){
    Serial.println("Could not enable accelerometer report");
  }
  if (! bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000)){
    Serial.println("Could not enable gyroscope report");
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



void loop() {
  delay(1);

  if (bno1.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(bno1);
  }
  if (! bno1.getSensorEvent(&sensorValue)) {
    return;
  }
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
    
      quaternionToEuler(i, j, k, real, roll, pitch, yaw);
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
