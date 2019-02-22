/*
 * \brief Library for testing ADS1248 using Arduino
 
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
 */

#include "pins_arduino.h"
#include "ADS1248_DEVCUBE.h"

#define SPI_MODE_MASK 0x0C  	// CPOL = bit 3, CPHA = bit 2 on SPCR

static inline uint8_t spiTransferByte(uint8_t data);
static inline void setSPIMode(uint8_t mode);

ADS1248_DEVCUBE ADS1248_DEV;

bool ADS1248_DEVCUBE::begin(void)
{
	uint8_t temp;

  // Set pin directions
	pinMode(SCK, OUTPUT);
	pinMode(MOSI, OUTPUT);
	pinMode(SS, OUTPUT);
	pinMode(MISO,INPUT);
	pinMode(ADS1248_RESET,OUTPUT);
	pinMode(ADS1248_START,OUTPUT);
	pinMode(ADS1248_DRDY,INPUT);

	// Set pin idle states
	digitalWrite(SCK, LOW);
	digitalWrite(MOSI, LOW);
	digitalWrite(SS, HIGH);
	digitalWrite(MISO, LOW);
	digitalWrite(ADS1248_RESET, HIGH);
	digitalWrite(ADS1248_START, LOW);

  //SPI speed is set to 2MHz
	// Init SPI - f_cpu/128
	SPCR = (1<<MSTR) | (1<<SPE) | (0<<SPR1) | (1<<SPR0);
	SPSR |= (1<<SPI2X);				//2x speed
	// clear status by doing a read
	SPSR;SPDR;

	reset();

	temp = readReg(ADS1248_TEST_REG);
	writeReg(ADS1248_TEST_REG, 0x30);
	if(readReg(ADS1248_TEST_REG) == 0x30)
	{
		writeReg(ADS1248_TEST_REG,temp); 		//Return to reset state
		// Perform self offset calibration
		//selfOffsetCal();
		return true;
	}
	return false;

}

void ADS1248_DEVCUBE::end(void)
{
	SPCR &= ~(1<<SPE);
}

void ADS1248_DEVCUBE::enableIntRef(void)
{
	writeReg(ADS1248_MUX1,readReg(ADS1248_MUX1) | ADC_VREF_ON);
}

void ADS1248_DEVCUBE::selectRef(uint8_t ref_mux)
{
	writeReg(ADS1248_MUX1,(readReg(ADS1248_MUX1) & ~ADS1248_REFSEL_MASK) | (ref_mux & ADS1248_REFSEL_MASK));
}

void ADS1248_DEVCUBE::selectMuxCal(uint8_t muxcal)
{
	writeReg(ADS1248_MUX1,(readReg(ADS1248_MUX1) & ~ADS1248_MUXCAL_MASK) | (muxcal & ADS1248_MUXCAL_MASK));
}

void ADS1248_DEVCUBE::reset(void)
{
	digitalWrite(ADS1248_RESET,LOW);
	delay(1);			// Min 4*t_osc
	digitalWrite(ADS1248_RESET,HIGH);
	delay(1);			// min .6ms
}

float ADS1248_DEVCUBE::sample_raw(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t sampleRate, uint8_t ref)
{
	int32_t res;
	// Enable START to allow writing to registers
	digitalWrite(ADS1248_START, HIGH);
	// Check if reference on.  If not, turn on and wait...
	if(readReg(ADS1248_MUX1) & ADC_VREF_ON)
		writeReg(ADS1248_MUX1, ADC_VREF_ON | (ADS1248_REFSEL_MASK & ref));
	else
	{
		writeReg(ADS1248_MUX1, ADC_VREF_ON | (ADS1248_REFSEL_MASK & ref));
		// Delay 3ms to allow reference to settle
		delay(3);
	}
	// Setup gain and sample rate
	writeReg(ADS1248_SYS0, (ADS1248_PGA_MASK & gain) | (ADS1248_SPS_MASK & sampleRate));
	// Choose channels
	writeReg(ADS1248_MUX0, (ADS1248_CH_MASK & ch_neg) | ((ADS1248_CH_MASK & ch_pos) << 3));

  writeReg(ADS1248_VBIAS, 0x80);
	// Disable START to reset sample
	digitalWrite(ADS1248_START, LOW);
	delayMicroseconds(1);
	// Start conversion
	startSingle();
	// Wait for completion - TODO: put timeout on DRDY in case the ADS1248 doesn't return
	while(digitalRead(ADS1248_DRDY) == HIGH){};
	// Retrieve result
	res = readData();
	return res;
}

void ADS1248_DEVCUBE::startSingle(void)
{
	digitalWrite(ADS1248_START, HIGH);
	delayMicroseconds(2);				// Datasheet specifies minimum 3*t_osc which would really be < 1us
	digitalWrite(ADS1248_START, LOW);
}

void ADS1248_DEVCUBE::setPGA(uint8_t gain)
{
	writeReg(ADS1248_SYS0,(readReg(ADS1248_SYS0) & ADS1248_SPS_MASK) | (gain & ADS1248_PGA_MASK));
}

void ADS1248_DEVCUBE::setSampleRate(uint8_t rate)
{
	writeReg(ADS1248_SYS0,(readReg(ADS1248_SYS0) & ADS1248_PGA_MASK) | (rate & ADS1248_SPS_MASK));
}

// len can be no more than 16 bytes
uint8_t ADS1248_DEVCUBE::readReg(uint8_t addr)
{
	uint8_t ret;
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte((addr & 0x0F) | ADS1248_CMD_RREG);
	spiTransferByte(0x00);
	ret = spiTransferByte(ADS1248_CMD_NOP);
	digitalWrite(SS, HIGH);  // Pull CS high
	return ret;
}

int32_t ADS1248_DEVCUBE::readData(void)
{
	uint32_t ret;
	uint8_t byte;
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	byte = spiTransferByte(ADS1248_CMD_NOP);
	ret = (((uint32_t)byte) << 16) | ((byte & 0x80) ? 0xFF000000:0x00000000);
	ret |= (((uint32_t)spiTransferByte(ADS1248_CMD_NOP)) << 8);
	ret |= (uint32_t)spiTransferByte(ADS1248_CMD_NOP);
	digitalWrite(SS, HIGH);  // Pull CS high
	return ret;
}

void ADS1248_DEVCUBE::stopContinuous(void)
{
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS1248_CMD_SDATAC);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void ADS1248_DEVCUBE::writeReg(uint8_t addr, uint8_t data)
{
	digitalWrite(SS, HIGH);  // Just to be sure
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte((addr & 0x0F) | ADS1248_CMD_WREG);
	spiTransferByte(0x00);
	spiTransferByte(data);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void ADS1248_DEVCUBE::wake(void)
{
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS1248_CMD_WAKE);
	digitalWrite(SS, HIGH);  // Pull CS high
}


void ADS1248_DEVCUBE::sleep(void)
{
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS1248_CMD_SELFOCAL);
	digitalWrite(SS, HIGH);  // Pull CS high
}

void ADS1248_DEVCUBE::selfOffsetCal(void)
{
	uint8_t MUX1_old = readReg(ADS1248_MUX1);
	digitalWrite(ADS1248_START, HIGH);
	enableIntRef();
	selectRef(ADC_INTREF);
	digitalWrite(ADS1248_START, LOW);
	delay(1);			// Wait for ref to settle
	setSPIMode(ADS1248_SPI_MODE);
	digitalWrite(SS, LOW);  // Pull CS low
	spiTransferByte(ADS1248_CMD_SLEEP);
	digitalWrite(SS, HIGH);  // Pull CS high
	// TODO add timeout here
	while(digitalRead(ADS1248_DRDY) == HIGH){};
	digitalWrite(ADS1248_START, HIGH);
	writeReg(ADS1248_MUX1,MUX1_old);
	digitalWrite(ADS1248_START, LOW);
}

void ADS1248_DEVCUBE::dumpRegs(void)
{
	digitalWrite(ADS1248_START, HIGH);
	for(uint8_t i = 0;i<0x0F;i++)
	{
		Serial.print("Reg 0x");
		Serial.print(i,16);
		Serial.print(" : 0x");
		Serial.print(readReg(i),16);
		Serial.print("\n");
	}
	digitalWrite(ADS1248_START, LOW);
}

// Remember that this function does NOT control the chip select line
uint8_t spiTransferByte(uint8_t data)
{
	SPDR = data;
	// wait for transfer to complete
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}
//SPI Mode is 1 for ADS1248
void setSPIMode(uint8_t mode)
{
	SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}
