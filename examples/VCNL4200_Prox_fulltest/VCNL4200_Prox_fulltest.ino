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

  Serial.println("Adafruit VCNL4200 proximity simple test");

  if (!vcnl4200.begin()) {
    Serial.println("Could not find a valid VCNL4200 sensor, check wiring!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("VCNL4200 found!");

  vcnl4200.setALSshutdown(true);
  vcnl4200.setProxShutdown(false);
  vcnl4200.setProxHD(false);
  vcnl4200.setProxLEDCurrent(VCNL4200_LED_I_200MA);
  vcnl4200.setProxIntegrationTime(VCNL4200_PS_IT_8T);
}

void loop() {
  uint16_t proxData = vcnl4200.readProxData();
  Serial.print("Prox Data: ");
  Serial.println(proxData);

  delay(100);
}
