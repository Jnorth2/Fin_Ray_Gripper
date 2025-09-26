#include <Wire.h>
#include "Adafruit_MPRLS.h"

// ----- PCA9546A multiplexer I2C address -----
#define PCAADDR 0x70   // Default address

// ----- MPRLS sensor setup -----
#define RESET_PIN  -1  // Not connected
#define EOC_PIN    -1  // Not connected
Adafruit_MPRLS mprls(RESET_PIN, EOC_PIN);

// ----- Select channel on the mux -----
void pcaSelect(uint8_t channel) {
  if (channel > 3) return; // only 0–3 valid
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << channel);  
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Created Serial line");
  Wire.begin();  // ESP32 default SDA=21, SCL=22

  delay(500);
  Serial.println("MPRLS + PCA9546A test");

  // Select channel 0 on mux
  pcaSelect(3);

  // Initialize the MPRLS
  if (!mprls.begin()) {
    Serial.println("Could not find MPRLS sensor, check wiring!");
    while (1) { delay(10); }
  }
  Serial.println("MPRLS found.");
}

void loop() {
  // Make sure we are still on channel 0
  pcaSelect(3);

  // Read pressure (returns hPa)
  float pressure_hPa = mprls.readPressure();
  if (isnan(pressure_hPa)) {
    Serial.println("Pressure read failed (NaN)");
  } else {
    Serial.print("Pressure: ");
    Serial.print(pressure_hPa);
    Serial.println(" hPa");
  }

  delay(1000);
}
