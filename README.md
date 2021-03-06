# ADS1248_Library_Arduino
ADS1248 interfacing example for arduino

Based on JONNYDYER DAQSHIELD Library
https://github.com/jonnydyer/daqshield

# Example Program
The example program sets channel 6 as +ve input and channel 7 as -ve input
The Sample per second is set to 5
The ADC gain is set 128 hence voltage greater than +-16mV will be out of range
The ADC internal reference 2.048V is used for readings

# Pin Connections
ADS1248 RESET --- Digital Pin 2<br>
ADS1248 START --- Digital Pin 3<br>
ADS1248 DRDY  --- Digital Pin 4<br>
ADS1248 DIN   --- Digital Pin 11 MOSI<br>
ADS1248 DOUT  --- Digital Pin 12 MISO<br>
ADS1248 SCK   --- Digital Pin 13 SCK<br>
ADS1248 CS    --- Digital Pin 10 SS<br>

# License

MIT License

Copyright (c) 2019 Mohammed Asim Merchant

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
