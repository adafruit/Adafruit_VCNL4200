/*  This is a simple test example for ALS (light sensor) part of the VCNL4200.
 *   we don't use the proximity sensor in this test, its good for verifying the interrupt support
 *   and the time integration settings.
 */


#include "Adafruit_VCNL4200.h"

Adafruit_VCNL4200 vcnl4200;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for native USB
  }

  Serial.println("Adafruit VCNL4200 ALS simple test");

  if (!vcnl4200.begin()) {
    Serial.println("Could not find a valid VCNL4200 sensor, check wiring!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("VCNL4200 found!");

  vcnl4200.setALSshutdown(false);
  vcnl4200.setALSIntegrationTime(VCNL4200_ALS_IT_100MS);
  vcnl4200.setALSPersistence(VCNL4200_ALS_PERS_2);
  vcnl4200.setALSthresholdLow(100);
  vcnl4200.setALSthresholdHigh(20000);
  vcnl4200.setInterrupt(true, false); // activate int on ALS chan
}

void loop() {
  // Read the ambient light sensor (ALS) data
  uint16_t alsData = vcnl4200.readALSdata();
  Serial.print("ALS Data: ");
  Serial.print(alsData);
  uint16_t whiteData = vcnl4200.readWhiteData();
  Serial.print(", White Data: ");
  Serial.println(whiteData);

  uint8_t flags = vcnl4200.getInterruptFlags();
  if (flags != 0 && flags != 0xFF) {
    Serial.print("Interrupt flags: 0x");
    Serial.println(flags, HEX);

    if (flags & VCNL4200_INTFLAG_ALS_HIGH) {
      Serial.println("ALS high threshold interrupt triggered.");
    }
    if (flags & VCNL4200_INTFLAG_ALS_LOW) {
      Serial.println("ALS low threshold interrupt triggered.");
    }
  }

  delay(100);
}
