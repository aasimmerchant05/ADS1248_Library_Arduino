#include "ADS1248_DEVCUBE.h"

void setup() {
  pinMode(6, OUTPUT);
  Serial.begin(115200);
  if(ADS1248_DEV.begin() == true)
  {
    Serial.print("ADS1248 successfully configured!\n");
  }
  else
  {
    Serial.print("Problem configuring ADS1248!\n");
  }
}

void loop() {
  Serial.print((ADS1248_DEV.sample_raw(6,7,ADC_PGA_128,ADC_5_SPS,ADC_INTREF)));
  Serial.print("\r\n");
  delay(1000);
}
