/*!
 * @file ADS1248_Library_example.ino
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for sample program for
 * Texas Intruments ADS1248 delta sigma 24-bit ADC
 *
 * @section author Author
 *
 * Written by Mohammed Asim Merchant
 *
 * @section license License
 *
 * MIT License.
 * Copyright (c) 2019 Mohammed Asim Merchant

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
 *
 */

 /*
 Pin Connections

 ADS1248 RESET --- Digital Pin 2
 ADS1248 START --- Digital Pin 3
 ADS1248 DRDY  --- Digital Pin 4
 ADS1248 DIN   --- Digital Pin 11 MOSI
 ADS1248 DOUT  --- Digital Pin 12 MISO
 ADS1248 SCK   --- Digital Pin 13 SCK
 ADS1248 CS    --- Digital Pin 10 SS
 */

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
