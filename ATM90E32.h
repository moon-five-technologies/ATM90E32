/* ATM90E32 Energy Monitor Functions

The MIT License (MIT)

  Copyright (c) 2016 whatnick,Ryzee and Arun

  Modified to use with the CircuitSetup.us Split Phase Energy Meter by jdeglavina

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef ATM90E32_h
#define ATM90E32_h

#include <stdio.h>
#include <stdint.h>

#include "ATM90E32_COMMS.h"

/* STATUS REGISTERS */
#define MeterEn 0x00	   // Metering Enable
#define ChannelMapI 0x01   // Current Channel Mapping Configuration
#define ChannelMapU 0x02   // Voltage Channel Mapping Configuration
#define SagPeakDetCfg 0x05 // Sag and Peak Detector Period Configuration
#define OVth 0x06		   // Over Voltage Threshold
#define ZXConfig 0x07	   // Zero-Crossing Config
#define SagTh 0x08		   // Voltage Sag Th
#define PhaseLossTh 0x09   // Voltage Phase Losing Th
#define INWarnTh 0x0A	   // Neutral Current (Calculated) Warning Threshold
#define OIth 0x0B		   // Over Current Threshold
#define FreqLoTh 0x0C	   // Low Threshold for Frequency Detection
#define FreqHiTh 0x0D	   // High Threshold for Frequency Detection
#define PMPwrCtrl 0x0E	   // Partial Measurement Mode Power Control
#define IRQ0MergeCfg 0x0F  // IRQ0 Merge Configuration

/* EMM STATUS REGISTERS */
#define SoftReset 0x70	  // Software Reset
#define EMMState0 0x71	  // EMM State 0
#define EMMState1 0x72	  // EMM State 1
#define EMMIntState0 0x73 // EMM Interrupt Status 0
#define EMMIntState1 0x74 // EMM Interrupt Status 1
#define EMMIntEn0 0x75	  // EMM Interrupt Enable 0
#define EMMIntEn1 0x76	  // EMM Interrupt Enable 1
#define LastSPIData 0x78  // Last Read/Write SPI Value
#define CRCErrStatus 0x79 // CRC Error Status
#define CRCDigest 0x7A	  // CRC Digest
#define CfgRegAccEn 0x7F  // Configure Register Access Enable

/* LOW POWER MODE REGISTERS - NOT USED */
#define DetectCtrl 0x10
#define DetectTh1 0x11
#define DetectTh2 0x12
#define DetectTh3 0x13
#define PMOffsetA 0x14
#define PMOffsetB 0x15
#define PMOffsetC 0x16
#define PMPGA 0x17
#define PMIrmsA 0x18
#define PMIrmsB 0x19
#define PMIrmsC 0x1A
#define PMConfig 0x10B
#define PMAvgSamples 0x1C
#define PMIrmsLSB 0x1D

/* CONFIGURATION REGISTERS */
#define PLconstH 0x31 // High Word of PL_Constant
#define PLconstL 0x32 // Low Word of PL_Constant
#define MMode0 0x33	  // Metering Mode Config
#define MMode1 0x34	  // PGA Gain Configuration for Current Channels
#define PStartTh 0x35 // Startup Power Th (P)
#define QStartTh 0x36 // Startup Power Th (Q)
#define SStartTh 0x37 // Startup Power Th (S)
#define PPhaseTh 0x38 // Startup Power Accum Th (P)
#define QPhaseTh 0x39 // Startup Power Accum Th (Q)
#define SPhaseTh 0x3A // Startup Power Accum Th (S)

/* CALIBRATION REGISTERS */
#define PoffsetA 0x41 // A Line Power Offset (P)
#define QoffsetA 0x42 // A Line Power Offset (Q)
#define PoffsetB 0x43 // B Line Power Offset (P)
#define QoffsetB 0x44 // B Line Power Offset (Q)
#define PoffsetC 0x45 // C Line Power Offset (P)
#define QoffsetC 0x46 // C Line Power Offset (Q)
#define PQGainA 0x47  // A Line Calibration Gain
#define PhiA 0x48	  // A Line Calibration Angle
#define PQGainB 0x49  // B Line Calibration Gain
#define PhiB 0x4A	  // B Line Calibration Angle
#define PQGainC 0x4B  // C Line Calibration Gain
#define PhiC 0x4C	  // C Line Calibration Angle

/* FUNDAMENTAL/HARMONIC ENERGY CALIBRATION REGISTERS */
#define POffsetAF 0x51 // A Fund Power Offset (P)
#define POffsetBF 0x52 // B Fund Power Offset (P)
#define POffsetCF 0x53 // C Fund Power Offset (P)
#define PGainAF 0x54   // A Fund Power Gain (P)
#define PGainBF 0x55   // B Fund Power Gain (P)
#define PGainCF 0x56   // C Fund Power Gain (P)

/* MEASUREMENT CALIBRATION REGISTERS */
#define UgainA 0x61	  // A Voltage RMS Gain
#define IgainA 0x62	  // A Current RMS Gain
#define UoffsetA 0x63 // A Voltage Offset
#define IoffsetA 0x64 // A Current Offset
#define UgainB 0x65	  // B Voltage RMS Gain
#define IgainB 0x66	  // B Current RMS Gain
#define UoffsetB 0x67 // B Voltage Offset
#define IoffsetB 0x68 // B Current Offset
#define UgainC 0x69	  // C Voltage RMS Gain
#define IgainC 0x6A	  // C Current RMS Gain
#define UoffsetC 0x6B // C Voltage Offset
#define IoffsetC 0x6C // C Current Offset
#define IoffsetN 0x6E // N Current Offset

/* ENERGY REGISTERS */
#define APenergyT 0x80 // Total Forward Active
#define APenergyA 0x81 // A Forward Active
#define APenergyB 0x82 // B Forward Active
#define APenergyC 0x83 // C Forward Active
#define ANenergyT 0x84 // Total Reverse Active
#define ANenergyA 0x85 // A Reverse Active
#define ANenergyB 0x86 // B Reverse Active
#define ANenergyC 0x87 // C Reverse Active
#define RPenergyT 0x88 // Total Forward Reactive
#define RPenergyA 0x89 // A Forward Reactive
#define RPenergyB 0x8A // B Forward Reactive
#define RPenergyC 0x8B // C Forward Reactive
#define RNenergyT 0x8C // Total Reverse Reactive
#define RNenergyA 0x8D // A Reverse Reactive
#define RNenergyB 0x8E // B Reverse Reactive
#define RNenergyC 0x8F // C Reverse Reactive

#define SAenergyT 0x90 // Total Apparent Energy
#define SenergyA 0x91  // A Apparent Energy
#define SenergyB 0x92  // B Apparent Energy
#define SenergyC 0x93  // C Apparent Energy

/* FUNDAMENTAL / HARMONIC ENERGY REGISTERS */
#define APenergyTF 0xA0 // Total Forward Fund. Energy
#define APenergyAF 0xA1 // A Forward Fund. Energy
#define APenergyBF 0xA2 // B Forward Fund. Energy
#define APenergyCF 0xA3 // C Forward Fund. Energy
#define ANenergyTF 0xA4 // Total Reverse Fund Energy
#define ANenergyAF 0xA5 // A Reverse Fund. Energy
#define ANenergyBF 0xA6 // B Reverse Fund. Energy
#define ANenergyCF 0xA7 // C Reverse Fund. Energy
#define APenergyTH 0xA8 // Total Forward Harm. Energy
#define APenergyAH 0xA9 // A Forward Harm. Energy
#define APenergyBH 0xAA // B Forward Harm. Energy
#define APenergyCH 0xAB // C Forward Harm. Energy
#define ANenergyTH 0xAC // Total Reverse Harm. Energy
#define ANenergyAH 0xAD // A Reverse Harm. Energy
#define ANenergyBH 0xAE // B Reverse Harm. Energy
#define ANenergyCH 0xAF // C Reverse Harm. Energy

/* POWER & P.F. REGISTERS */
#define PmeanT 0xB0	 // Total Mean Power (P)
#define PmeanA 0xB1	 // A Mean Power (P)
#define PmeanB 0xB2	 // B Mean Power (P)
#define PmeanC 0xB3	 // C Mean Power (P)
#define QmeanT 0xB4	 // Total Mean Power (Q)
#define QmeanA 0xB5	 // A Mean Power (Q)
#define QmeanB 0xB6	 // B Mean Power (Q)
#define QmeanC 0xB7	 // C Mean Power (Q)
#define SmeanT 0xB8	 // Total Mean Power (S)
#define SmeanA 0xB9	 // A Mean Power (S)
#define SmeanB 0xBA	 // B Mean Power (S)
#define SmeanC 0xBB	 // C Mean Power (S)
#define PFmeanT 0xBC // Mean Power Factor
#define PFmeanA 0xBD // A Power Factor
#define PFmeanB 0xBE // B Power Factor
#define PFmeanC 0xBF // C Power Factor

#define PmeanTLSB 0xC0	// Lower Word (Tot. Act. Power)
#define PmeanALSB 0xC1	// Lower Word (A Act. Power)
#define PmeanBLSB 0xC2	// Lower Word (B Act. Power)
#define PmeanCLSB 0xC3	// Lower Word (C Act. Power)
#define QmeanTLSB 0xC4	// Lower Word (Tot. React. Power)
#define QmeanALSB 0xC5	// Lower Word (A React. Power)
#define QmeanBLSB 0xC6	// Lower Word (B React. Power)
#define QmeanCLSB 0xC7	// Lower Word (C React. Power)
#define SAmeanTLSB 0xC8 // Lower Word (Tot. App. Power)
#define SmeanALSB 0xC9	// Lower Word (A App. Power)
#define SmeanBLSB 0xCA	// Lower Word (B App. Power)
#define SmeanCLSB 0xCB	// Lower Word (C App. Power)

/* FUND/HARM POWER & V/I RMS REGISTERS */
#define PmeanTF 0xD0 // Total Active Fund. Power
#define PmeanAF 0xD1 // A Active Fund. Power
#define PmeanBF 0xD2 // B Active Fund. Power
#define PmeanCF 0xD3 // C Active Fund. Power
#define PmeanTH 0xD4 // Total Active Harm. Power
#define PmeanAH 0xD5 // A Active Harm. Power
#define PmeanBH 0xD6 // B Active Harm. Power
#define PmeanCH 0xD7 // C Active Harm. Power
#define UrmsA 0xD9	 // A RMS Voltage
#define UrmsB 0xDA	 // B RMS Voltage
#define UrmsC 0xDB	 // C RMS Voltage
#define IrmsN 0xDC	 // Calculated N RMS Current
#define IrmsA 0xDD	 // A RMS Current
#define IrmsB 0xDE	 // B RMS Current
#define IrmsC 0xDF	 // C RMS Current

#define PmeanTFLSB 0xE0 // Lower Word (Tot. Act. Fund. Power)
#define PmeanAFLSB 0xE1 // Lower Word (A Act. Fund. Power)
#define PmeanBFLSB 0xE2 // Lower Word (B Act. Fund. Power)
#define PmeanCFLSB 0xE3 // Lower Word (C Act. Fund. Power)
#define PmeanTHLSB 0xE4 // Lower Word (Tot. Act. Harm. Power)
#define PmeanAHLSB 0xE5 // Lower Word (A Act. Harm. Power)
#define PmeanBHLSB 0xE6 // Lower Word (B Act. Harm. Power)
#define PmeanCHLSB 0xE7 // Lower Word (C Act. Harm. Power)
///////////////// 0xE8	    // Reserved Register
#define UrmsALSB 0xE9 // Lower Word (A RMS Voltage)
#define UrmsBLSB 0xEA // Lower Word (B RMS Voltage)
#define UrmsCLSB 0xEB // Lower Word (C RMS Voltage)
///////////////// 0xEC	    // Reserved Register
#define IrmsALSB 0xED // Lower Word (A RMS Current)
#define IrmsBLSB 0xEE // Lower Word (B RMS Current)
#define IrmsCLSB 0xEF // Lower Word (C RMS Current)

/* PEAK, FREQUENCY, ANGLE & TEMP REGISTERS*/
#define UPeakA 0xF1 // A Voltage Peak - THD+N on ATM90E36
#define UPeakB 0xF2 // B Voltage Peak
#define UPeakC 0xF3 // C Voltage Peak
///////////////// 0xF4	    // Reserved Register
#define IPeakA 0xF5	 // A Current Peak
#define IPeakB 0xF6	 // B Current Peak
#define IPeakC 0xF7	 // C Current Peak
#define Freq 0xF8	 // Frequency
#define PAngleA 0xF9 // A Mean Phase Angle
#define PAngleB 0xFA // B Mean Phase Angle
#define PAngleC 0xFB // C Mean Phase Angle
#define Temp 0xFC	 // Measured Temperature
#define UangleA 0xFD // A Voltage Phase Angle
#define UangleB 0xFE // B Voltage Phase Angle
#define UangleC 0xFF // C Voltage Phase Angle

// MMode0 REGISTER
#define MMode0_EnPC 1
#define MMode0_EnPB 1 << 1
#define MMode0_EnPA 1 << 2
#define MMode0_ABSEnP 1 << 3
#define MMode0_ABSEnQ 1 << 4
#define MMode0_CF2varh 1 << 7
#define MMode0_3P3W 1 << 8
#define MMode0_didtEn 1 << 10 // Set for Rogowski coil, unset for CT
#define MMode0_HPFOff 1 << 11
#define MMode0_Freq60Hz 1 << 12 // Set for 60Hz, unset for 50Hz

// 60 Hz, reactive energy, all phase count towards sum, CT sensors
#define MMode0_default_50Hz MMode0_EnPC | MMode0_EnPB | MMode0_EnPA | MMode0_CF2varh
#define MMode0_default_60Hz MMode0_default_50Hz | MMode0_Freq60Hz

// Gain=1 for A, B, C
#define MMode1_default 0x0000

#define PGA_GAIN_1 0
#define PGA_GAIN_2 1
#define PGA_GAIN_4 2

#define UGAIN_DEFAULT 0x8000
#define IGAIN_DEFAULT 0x8000

enum atm90_chan
{
	ATM90_CHAN_A,
	ATM90_CHAN_B,
	ATM90_CHAN_C,
	ATM90_CHAN_N, // Neutral channel
};
struct atm90e32_calib_meas_gain
{
	unsigned short ugain;
	unsigned short igain;
};
struct atm90e32_calib_meas_offset
{
	unsigned short uoffset;
	unsigned short ioffset;
};
struct atm90e32_calib_energy
{
	unsigned short poffset;
	unsigned short qoffset;
	unsigned short pqgain;
	unsigned short phi;
};
struct atm90e32_calibration
{
	unsigned short mmode0;
	unsigned short mmode1;
	unsigned short pga_gain;
	atm90e32_calib_meas_gain meas_gain_a;
	atm90e32_calib_meas_gain meas_gain_b;
	atm90e32_calib_meas_gain meas_gain_c;
	atm90e32_calib_meas_offset meas_offset_a;
	atm90e32_calib_meas_offset meas_offset_b;
	atm90e32_calib_meas_offset meas_offset_c;
};

const atm90e32_calibration default_cal = {
	// DEFAULT CALIBRATION
	.mmode0 = MMode0_default_60Hz,
	.mmode1 = MMode1_default,
	.pga_gain = PGA_GAIN_1,
	.meas_gain_a = {
		.ugain = UGAIN_DEFAULT,
		.igain = IGAIN_DEFAULT,
	},
	.meas_gain_b = {
		.ugain = UGAIN_DEFAULT,
		.igain = IGAIN_DEFAULT,
	},
	.meas_gain_c = {
		.ugain = UGAIN_DEFAULT,
		.igain = IGAIN_DEFAULT,
	},
	.meas_offset_a = {
		.uoffset = 0,
		.ioffset = 0,
	},
	.meas_offset_b = {
		.uoffset = 0,
		.ioffset = 0,
	},
	.meas_offset_c = {
		.uoffset = 0,
		.ioffset = 0,
	},
};

class ATM90E32
{
private:
	atm90e32_calibration cal = default_cal;
	Comms &_comms;

	int Read32Register(const unsigned short regh_addr, const unsigned short regl_addr);
	void applyCalibration();
	void readCalibration(atm90e32_calibration *cal);
	uint16_t ATM90E32::CalculateGain(atm90_chan chan, double actualVal,
									 double (ATM90E32::*measureFunc)(atm90_chan),
									 const unsigned short *gainMap,
									 size_t gainMapSize, unsigned short lsb_val);

public:
	/* Construct */
	ATM90E32(Comms &comms) : _comms(comms) {};
	/* Destruct */
	~ATM90E32(void);

	/* Initialization Functions */
	void begin();

	uint16_t CalculateVIOffset(const unsigned short regh_addr, const unsigned short regl_addr);
	uint16_t CalculatePowerOffset(const unsigned short regh_addr, const unsigned short regl_addr);
	uint16_t CalculateVIGain(atm90_chan chan, double actualVal, bool isVoltage);
	uint16_t CalculateVGain(atm90_chan chan, double actualVal);
	uint16_t CalculateUGain(atm90_chan chan, double actualVal);
	void setCalibration(atm90e32_calibration &cal);
	void getCalibration(atm90e32_calibration *cal);

	/* Main Electrical Parameters (GET)*/
	double GetLineVoltage(atm90_chan chan);

	double GetLineCurrent(atm90_chan chan);

	double GetLineVoltageA() { return GetLineVoltage(ATM90_CHAN_A); }
	double GetLineVoltageB() { return GetLineVoltage(ATM90_CHAN_B); }
	double GetLineVoltageC() { return GetLineVoltage(ATM90_CHAN_C); }

	double GetLineCurrentA() { return GetLineCurrent(ATM90_CHAN_A); }
	double GetLineCurrentB() { return GetLineCurrent(ATM90_CHAN_B); }
	double GetLineCurrentC() { return GetLineCurrent(ATM90_CHAN_C); }
	double GetLineCurrentN() { return GetLineCurrent(ATM90_CHAN_N); }

	double GetActivePowerA();
	double GetActivePowerB();
	double GetActivePowerC();
	double GetTotalActivePower();

	double GetTotalActiveFundPower();
	double GetTotalActiveHarPower();

	double GetReactivePowerA();
	double GetReactivePowerB();
	double GetReactivePowerC();
	double GetTotalReactivePower();

	double GetApparentPowerA();
	double GetApparentPowerB();
	double GetApparentPowerC();
	double GetTotalApparentPower();

	double GetFrequency();

	double GetPowerFactorA();
	double GetPowerFactorB();
	double GetPowerFactorC();
	double GetTotalPowerFactor();

	double GetPhaseA();
	double GetPhaseB();
	double GetPhaseC();

	double GetTemperature();

	/* Gain Parameters (GET)*/
	unsigned short GetValueRegister(unsigned short registerRead);

	/* Energy Consumption */
	double GetImportEnergy();
	double GetImportReactiveEnergy();
	double GetImportApparentEnergy();
	double GetExportEnergy();
	double GetExportReactiveEnergy();

	/* System Status */
	unsigned short GetSysStatus0();
	unsigned short GetSysStatus1();
	unsigned short GetMeterStatus0();
	unsigned short GetMeterStatus1();
};
#endif