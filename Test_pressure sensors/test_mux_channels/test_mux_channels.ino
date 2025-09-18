#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define MUX_MIN 0x70
#define MUX_MAX 0x73
#define MPRLS_ADDR 0x18

void i2cBusRecover(int sclPin, int sdaPin) {
  pinMode(sclPin, INPUT_PULLUP);
  pinMode(sdaPin, INPUT_PULLUP);
  delay(5);
  if (digitalRead(sdaPin) == LOW) {
    pinMode(sclPin, OUTPUT);
    for (int i = 0; i < 9 && digitalRead(sdaPin) == LOW; i++) {
      digitalWrite(sclPin, HIGH); delayMicroseconds(5);
      digitalWrite(sclPin, LOW);  delayMicroseconds(5);
    }
    pinMode(sclPin, INPUT_PULLUP);
  }
}

bool scanAddr(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool muxWrite(uint8_t muxAddr, uint8_t channel) {
  Wire.beginTransmission(muxAddr);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

void scanBus() {
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found 0x"); Serial.println(a, HEX);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nI2C diag start");

  i2cBusRecover(SCL_PIN, SDA_PIN);

  Wire.setTimeOut(50);             // avoid long hangs
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);           // 100 kHz

  Serial.println("Scanning for PCA9546 at 0x70..0x73");
  uint8_t foundMux = 0;
  for (uint8_t a = MUX_MIN; a <= MUX_MAX; a++) {
    Serial.print("Try addr 0x"); Serial.print(a, HEX); Serial.print(" -> ");
    if (scanAddr(a)) { Serial.println("OK"); foundMux = a; break; }
    else Serial.println("no");
  }
  if (!foundMux) {
    Serial.println("No PCA9546 at 0x70..0x73. Full bus scan:");
    scanBus();
    return;
  }

  Serial.print("PCA9546 found at 0x"); Serial.println(foundMux, HEX);
  Serial.println("Select channel 0 and scan...");
  if (!muxWrite(foundMux, 0)) {
    Serial.println("mux write failed (NACK)");
    return;
  }
  delay(5);
  scanBus();
}

void loop() {}
