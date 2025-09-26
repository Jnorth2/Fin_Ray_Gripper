#include <Wire.h>
void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(1000);
  Serial.println("Simple I2C scan:");
//  for (byte addr = 1; addr < 127; addr++) {
//    Wire.beginTransmission(addr);
//    if (Wire.endTransmission() == 0) {
//      Serial.print("Device found at 0x");
//      Serial.println(addr, HEX);
//    }
//  }
  Wire.beginTransmission(0x70);
  if (Wire.endTransmission() == 0) {
    Serial.println("PCA9546 found!");
  } else {
    Serial.println("PCA9546 NOT found!");
  }

}
void loop() {}
