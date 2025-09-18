#include <Wire.h>

// Change this to your multiplexer’s I2C address.
// Commonly: 0x70 (TCA9548A), 0x72 (TCA9546A), 0x70–0x73 for PCA9544A.
#define MUX_ADDR 0x70  

void selectMuxChannel(uint8_t channel) {
  if (channel > 3) return; // only 0–3 valid
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);  // one-hot channel select
  Wire.endTransmission();
}

void scanI2C() {
  for (uint8_t channel = 0; channel < 4; channel++) {
    Serial.print("\n--- Scanning multiplexer channel ");
    Serial.print(channel);
    Serial.println(" ---");

    selectMuxChannel(channel);

    for (uint8_t address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      uint8_t error = Wire.endTransmission();

      if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address < 16) Serial.print("0");
        Serial.print(address, HEX);
        Serial.println(" !");
      } else if (error == 4) {
        Serial.print("Unknown error at address 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
      }
    }
  }
}

void scan_single_channel(int channel){
  Serial.print("\n--- Scanning multiplexer channel ");
  Serial.print(channel);
  Serial.println(" ---");

  selectMuxChannel(channel);

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA, SCL for Mega
  // SDA = 21, SCL = 22 on ESP32 by default

  delay(2000);
  Serial.println("Starting I2C multiplexer scan...");

  scanI2C();
  Serial.println("Scan complete.");
}

void loop() {
  // Nothing to do here
  delay(10000);
  //scan_single_channel(3);
  scanI2C();
}
