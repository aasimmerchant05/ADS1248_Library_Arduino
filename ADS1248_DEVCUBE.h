/*
 * \brief Definitions for the ADS1248 part
 *
 * This library has all necessary functions for using the ADS1248
 * Copyright (c) 2012 by Mohammed Asim Merchant <mohammedasimmerchant@gmail.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _ADS1248_DEVCUBE_H_INCLUDED
#define _ADS1248_DEVCUBE_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "pins_arduino.h"

// Commands
#define ADS1248_RESET         2
#define ADS1248_START         3
#define ADS1248_DRDY          4
#define ADS1248_SPI_MODE      ((0<<CPOL) | (1<<CPHA))
#define ADS1248_CMD_WAKE      0x00
#define ADS1248_CMD_SLEEP     0x02
#define ADS1248_CMD_SYNC      0x04
#define ADS1248_CMD_RESET     0x06
#define ADS1248_CMD_RDATA     0x12
#define ADS1248_CMD_RDATAC    0x14
#define ADS1248_CMD_SDATAC    0x16
#define ADS1248_CMD_RREG      0x20
#define ADS1248_CMD_WREG      0x40
#define ADS1248_CMD_SYSOCAL   0x60
#define ADS1248_CMD_SYSGCAL   0x61
#define ADS1248_CMD_SELFOCAL  0x62
#define ADS1248_CMD_NOP	      0xFF

// Registers
#define ADS1248_MUX0          0x00
#define ADS1248_VBIAS         0x01
#define ADS1248_MUX1          0x02
#define ADS1248_SYS0          0x03
#define ADS1248_OFC0          0x04
#define ADS1248_OFC1          0x05
#define ADS1248_OFC2          0x06
#define ADS1248_FSC0          0x07
#define ADS1248_FSC1          0x08
#define ADS1248_FSC2          0x09
#define ADS1248_IDAC0         0x0A
#define ADS1248_IDAC1         0x0B
#define ADS1248_GPIOCFG       0x0C
#define ADS1248_GPIODIR       0x0D
#define ADS1248_GPIODAT       0x0E

// Register Bit Masks
#define ADS1248_CH_MASK       0x07			// Low 3 bits used for MUX in MUX0, IDAC1 and IDAC0
#define ADS1248_PGA_MASK      0x70
#define ADS1248_SPS_MASK      0x0F
#define ADS1248_REFSEL_MASK   (0x03 << 3)
#define ADS1248_VREF_MASK     (0x03 << 5)
#define ADS1248_MUXCAL_MASK   0x07

#define ADS1248_MAX_VAL       0x7FFFFF
#define ADS1248_MIN_VAL       0x800000
#define ADS1248_RANGE         0xFFFFFF
#define ADS1248_INT_REF_MV    2048

#define ADS1248_TEST_REG      ADS1248_MUX1

#define ADS1248_DELAY         delayMicroseconds(10)

// Setting defines for ADS1248
#define ADC_5_SPS             0x00
#define ADC_10_SPS            0x01
#define ADC_20_SPS            0x02
#define ADC_40_SPS            0x03
#define ADC_80_SPS            0x04
#define ADC_160_SPS           0x05
#define ADC_320_SPS           0x06
#define ADC_640_SPS           0x07
#define ADC_1000_SPS          0x08
#define ADC_2000_SPS          0x09
#define ADC_PGA_1             (0x00 << 4)
#define ADC_PGA_2             (0x01 << 4)
#define ADC_PGA_4             (0x02 << 4)
#define ADC_PGA_8             (0x03 << 4)
#define ADC_PGA_16            (0x04 << 4)
#define ADC_PGA_32            (0x05 << 4)
#define ADC_PGA_64            (0x06 << 4)
#define ADC_PGA_128           (0x07 << 4)
#define ADC_IDAC_0uA          0x00
#define ADC_IDAC_50uA         0x01
#define ADC_IDAC_100uA        0x02
#define ADC_IDAC_250uA        0x03
#define ADC_IDAC_500uA        0x04
#define ADC_IDAC_750uA        0x05
#define ADC_IDAC_1000uA       0x06
#define ADC_IDAC_1500uA       0x07
#define ADC_IDAC1_IEXT1       (0x80 << 4)
#define ADC_IDAC1_IEXT2       (0x81 << 4)
#define ADC_IDAC1_DISCONNECTED (0xC0 << 4)
#define ADC_IDAC2_IEXT1       0x80
#define ADC_IDAC2_IEXT2       0x81
#define ADC_IDAC2_DISCONNECTED 0xC0
#define ADC_REF0              (0x00 << 3)
#define ADC_REF1              (0x01 << 3)
#define ADC_INTREF            (0x02 << 3)
#define ADC_INTREF_REF0       (0x03 << 3)
#define ADC_VREF_OFF          (0x00 << 5)
#define ADC_VREF_ON           (0x01 << 5)
#define ADC_VREF_ON_LP        (0x02 << 5)
#define ADC_MUXCAL_NORM       0x00
#define ADC_MUXCAL_OFFSET     0x01
#define ADC_MUXCAL_GAIN       0x02
#define ADC_MUXCAL_TEMP       0x03
#define ADC_MUXCAL_REF1       0x04
#define ADC_MUXCAL_REF0       0x05
#define ADC_MUXCAL_AVDD       0x06
#define ADC_MUXCAL_DVDD       0x07

//ADS1248 Class
class ADS1248_DEVCUBE {
public:
	bool begin(void);
	void end(void);

	void reset(void);

	void wake(void);
	void sleep(void);
	void stopContinuous(void);

	void startSingle(void);
	float sample_raw(uint8_t ch_pos, uint8_t ch_neg, uint8_t gain, uint8_t sampleRate, uint8_t ref);

	void setPGA(uint8_t gain);
	void setSampleRate(uint8_t rate);
	void enableIntRef(void);
	void selectRef(uint8_t ref_mux);
	void selectMuxCal(uint8_t muxcal);
	void selfOffsetCal(void);
	void dumpRegs(void);

	uint8_t readReg(uint8_t addr);
	int32_t readData(void);
	void writeReg(uint8_t addr, uint8_t data);
private:
	void initSPI(void);
};

extern ADS1248_DEVCUBE ADS1248_DEV;

#endif
