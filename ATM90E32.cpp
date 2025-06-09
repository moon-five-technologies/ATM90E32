/* ATM90E32 Energy Monitor Functions

  The MIT License (MIT)

  Copyright (c) 2016 whatnick,Ryzee and Arun

  Modified to use with the CircuitSetup.us Split Phase Energy Meter by jdeglavina

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ATM90E32.h"
#include <stdint.h>

// Replace previous mapping structures with simple constant arrays:
static const unsigned short voltageGainMap[] = { UgainA, UgainB, UgainC };
static const unsigned short currentGainMap[] = { IgainA, IgainB, IgainC };

ATM90E32::~ATM90E32()
{
  // end
}

int ATM90E32::Read32Register(const unsigned short regh_addr, const unsigned short regl_addr)
{
  int val, val_h, val_l;
  val_h = _comms.transact(_comms.SPI_TRANS::READ, regh_addr);
  val_l = _comms.transact(_comms.SPI_TRANS::READ, regl_addr);

  // concatenate the 2 registers to make 1 32 bit number
  val = val_l | val_h << 16;

  return (val);
}

uint16_t ATM90E32::CalculateVIOffset(const unsigned short regh_addr, const unsigned short regl_addr)
{
  uint32_t raw_val = 0;
  for (int i = 0; i < 4; i++)
  {
    raw_val += Read32Register(regh_addr, regl_addr);
    if (i < 3) delay(320);
  }
  raw_val /= 4;
  raw_val = raw_val >> 7;    // right shift 7 bits
  raw_val = (~raw_val) + 1;  // 2s compliment
  uint16_t offset = raw_val; // keep lower 16 bits
  return uint16_t(offset);
}

// for getting the lower registers of energy and calculating the offset
// should only be run when CT sensors are connected to the meter,
// but not connected around wires
uint16_t ATM90E32::CalculatePowerOffset(const unsigned short regh_addr, const unsigned short regl_addr)
{
  uint32_t raw_val = 0;
  for (int i = 0; i < 4; i++)
  {
    raw_val += Read32Register(regh_addr, regl_addr);
    if (i < 3) delay(320);
  }
  raw_val /= 4;
  raw_val = (~raw_val) + 1;  // 2s compliment
  uint16_t offset = raw_val; // keep lower 16 bits
  return uint16_t(offset);
}

// Helper function to calculate gain for voltage or current
uint16_t ATM90E32::CalculateGain(atm90_chan chan, double actualVal, 
                                 double (ATM90E32::*measureFunc)(atm90_chan), 
                                 const unsigned short* gainMap, 
                                 size_t gainMapSize)
{
  unsigned short gain;
  double measuredVal = 0.0;

  // Average 4 readings using the provided measurement function
  for (int i = 0; i < 4; i++)
  {
    measuredVal += (this->*measureFunc)(chan);
    if (i < 3) delay(320);
  }
  measuredVal /= 4.0;

  // Check channel index and get gain register directly.
  if ((unsigned int)chan >= gainMapSize)
  {
      Serial.println("Error: Invalid channel for CalculateGain.");
      return 0;
  }
  unsigned short gainReg = gainMap[chan];

  gain = _comms.transact(_comms.SPI_TRANS::READ, gainReg);
  if (measuredVal == 0)
  {
    Serial.println("Error: Gain calculation failed, division by zero.");
    return 0;
  }

  // Calculate new gain
  gain = static_cast<uint16_t>((actualVal * gain) / measuredVal);
  return gain;
}

// input the channel and the actual voltage value that it should be
uint16_t ATM90E32::CalculateVGain(atm90_chan chan, double actualVal)
{
  return CalculateGain(chan, actualVal, 
                       &ATM90E32::GetLineVoltage,
                       voltageGainMap, sizeof(voltageGainMap)/sizeof(voltageGainMap[0]));
}

// input the channel and the actual current value that it should be
uint16_t ATM90E32::CalculateUGain(atm90_chan chan, double actualVal)
{
  return CalculateGain(chan, actualVal, 
                       &ATM90E32::GetLineCurrent,
                       currentGainMap, sizeof(currentGainMap)/sizeof(currentGainMap[0]));
}

/* Parameters Functions*/
/*
  - Gets main electrical parameters,
  such as: Voltage, Current, Power, Energy,
  and Frequency, and Temperature

*/
// VOLTAGE

double ATM90E32::GetLineVoltage(atm90_chan chan)
{
    unsigned short reg = 0, reg_lsb = 0;
    switch (chan) {
        case ATM90_CHAN_A:
            reg = UrmsA; reg_lsb = UrmsALSB; break;
        case ATM90_CHAN_B:
            reg = UrmsB; reg_lsb = UrmsBLSB; break;
        case ATM90_CHAN_C:
            reg = UrmsC; reg_lsb = UrmsCLSB; break;
        default:
            return 0.0;
    }
    // LSB = .01V
    unsigned short voltage = GetValueRegister(reg);
    // The upper 8-bits of the LSB register are used to store the next 8 bits 
    // of the voltage value.
    unsigned short voltage_lsb = GetValueRegister(reg_lsb);
    // Combine the upper 16 bits of the voltage and the lower 8 bits from the
    // LSB register to form a 24-bit value.
    uint32_t combined = (uint32_t(voltage) << 8) | (voltage_lsb >> 8);
    return (double)combined / 25600.0; // 100 * 256
}

double ATM90E32::GetLineCurrent(atm90_chan chan)
{
    unsigned short reg = 0;
    switch (chan) {
        case ATM90_CHAN_A:
            reg = IrmsA; break;
        case ATM90_CHAN_B:
            reg = IrmsB; break;
        case ATM90_CHAN_C:
            reg = IrmsC; break;
        case ATM90_CHAN_N:
            reg = IrmsN; break;
        default:
            return 0.0;
    }
    unsigned short current = GetValueRegister(reg);
    return (double)current / 1000;
}

// ACTIVE POWER
double ATM90E32::GetActivePowerA()
{
  int val = Read32Register(PmeanA, PmeanALSB);
  return (double)val * 0.00032;
}
double ATM90E32::GetActivePowerB()
{
  int val = Read32Register(PmeanB, PmeanBLSB);
  return (double)val * 0.00032;
}
double ATM90E32::GetActivePowerC()
{
  int val = Read32Register(PmeanC, PmeanCLSB);
  return (double)val * 0.00032;
}
double ATM90E32::GetTotalActivePower()
{
  int val = Read32Register(PmeanT, PmeanTLSB);
  return (double)val * 0.00032;
}

// Active Fundamental Power
double ATM90E32::GetTotalActiveFundPower()
{
  int val = Read32Register(PmeanTF, PmeanTFLSB);
  return (double)val * 0.00032;
}

// Active Harmonic Power
double ATM90E32::GetTotalActiveHarPower()
{
  int val = Read32Register(PmeanTH, PmeanTHLSB);
  return (double)val * 0.00032;
}

// REACTIVE POWER
double ATM90E32::GetReactivePowerA()
{
  int val = Read32Register(QmeanA, QmeanALSB);
  return (double)val * 0.00032;
}
double ATM90E32::GetReactivePowerB()
{
  int val = Read32Register(QmeanB, QmeanBLSB);
  return (double)val * 0.00032;
}
double ATM90E32::GetReactivePowerC()
{
  int val = Read32Register(QmeanC, QmeanCLSB);
  return (double)val * 0.00032;
}
double ATM90E32::GetTotalReactivePower()
{
  int val = Read32Register(QmeanT, QmeanTLSB);
  return (double)val * 0.00032;
}

// APPARENT POWER
double ATM90E32::GetApparentPowerA()
{
  int val = Read32Register(SmeanA, SmeanALSB);
  return (double)val * 0.00032;
}
double ATM90E32::GetApparentPowerB()
{
  int val = Read32Register(SmeanB, SmeanBLSB);
  return (double)val * 0.00032;
}
double ATM90E32::GetApparentPowerC()
{
  int val = Read32Register(SmeanC, SmeanCLSB);
  return (double)val * 0.00032;
}
double ATM90E32::GetTotalApparentPower()
{
  int val = Read32Register(SmeanT, SAmeanTLSB);
  return (double)val * 0.00032;
}

// FREQUENCY
double ATM90E32::GetFrequency()
{
  unsigned short freq = GetValueRegister(Freq);
  return (double)freq / 100;
}

// POWER FACTOR
double ATM90E32::GetPowerFactorA()
{
  signed short pf = (signed short)GetValueRegister(PFmeanA);
  return (double)pf / 1000;
}
double ATM90E32::GetPowerFactorB()
{
  signed short pf = (signed short)GetValueRegister(PFmeanB);
  return (double)pf / 1000;
}
double ATM90E32::GetPowerFactorC()
{
  signed short pf = (signed short)GetValueRegister(PFmeanC);
  return (double)pf / 1000;
}
double ATM90E32::GetTotalPowerFactor()
{
  signed short pf = (signed short)GetValueRegister(PFmeanT);
  return (double)pf / 1000;
}

// MEAN PHASE ANGLE
double ATM90E32::GetPhaseA()
{
  unsigned short angleA = (unsigned short)_comms.transact(_comms.SPI_TRANS::READ, PAngleA);
  return (double)angleA / 10;
}
double ATM90E32::GetPhaseB()
{
  unsigned short angleB = (unsigned short)_comms.transact(_comms.SPI_TRANS::READ, PAngleB);
  return (double)angleB / 10;
}
double ATM90E32::GetPhaseC()
{
  unsigned short angleC = (unsigned short)_comms.transact(_comms.SPI_TRANS::READ, PAngleC);
  return (double)angleC / 10;
}

// TEMPERATURE
double ATM90E32::GetTemperature()
{
  short int atemp = (short int)_comms.transact(_comms.SPI_TRANS::READ, Temp);
  return (double)atemp;
}

/* Gets the Register Value if Desired */
// REGISTER
unsigned short ATM90E32::GetValueRegister(unsigned short registerRead)
{
  return _comms.transact(_comms.SPI_TRANS::READ, registerRead);
}

// REGULAR ENERGY MEASUREMENT

// FORWARD ACTIVE ENERGY
// these registers accumulate energy and are cleared after being read
double ATM90E32::GetImportEnergy()
{
  unsigned short ienergyT = GetValueRegister(APenergyT);
  return (double)ienergyT / 100 / 3200; // returns kWh
}
// unsigned short ienergyA = _comms.transact(_comms.SPI_TRANS::READ, APenergyA);
// unsigned short ienergyB = _comms.transact(_comms.SPI_TRANS::READ, APenergyB);
// unsigned short ienergyC = _comms.transact(_comms.SPI_TRANS::READ, APenergyC);

// FORWARD REACTIVE ENERGY
double ATM90E32::GetImportReactiveEnergy()
{
  unsigned short renergyT = GetValueRegister(RPenergyT);
  return (double)renergyT / 100 / 3200; // returns kWh
}
// unsigned short renergyA = _comms.transact(_comms.SPI_TRANS::READ, RPenergyA);
// unsigned short renergyB = _comms.transact(_comms.SPI_TRANS::READ, RPenergyB);
// unsigned short renergyC = _comms.transact(_comms.SPI_TRANS::READ, RPenergyC);

// APPARENT ENERGY
double ATM90E32::GetImportApparentEnergy()
{
  unsigned short senergyT = GetValueRegister(SAenergyT);
  return (double)senergyT / 100 / 3200; // returns kWh
}
// unsigned short senergyA = _comms.transact(_comms.SPI_TRANS::READ, SenergyA);
// unsigned short senergyB = _comms.transact(_comms.SPI_TRANS::READ, SenergyB);
// unsigned short senergyC = _comms.transact(_comms.SPI_TRANS::READ, SenergyC);

// REVERSE ACTIVE ENERGY
double ATM90E32::GetExportEnergy()
{
  unsigned short eenergyT = GetValueRegister(ANenergyT);
  return (double)eenergyT / 100 / 3200; // returns kWh
}
// unsigned short eenergyA = _comms.transact(_comms.SPI_TRANS::READ, ANenergyA);
// unsigned short eenergyB = _comms.transact(_comms.SPI_TRANS::READ, ANenergyB);
// unsigned short eenergyC = _comms.transact(_comms.SPI_TRANS::READ, ANenergyC);

// REVERSE REACTIVE ENERGY
double ATM90E32::GetExportReactiveEnergy()
{
  unsigned short reenergyT = GetValueRegister(RNenergyT);
  return (double)reenergyT / 100 / 3200; // returns kWh
}
// unsigned short reenergyA = _comms.transact(_comms.SPI_TRANS::READ, RNenergyA);
// unsigned short reenergyB = _comms.transact(_comms.SPI_TRANS::READ, RNenergyB);
// unsigned short reenergyC = _comms.transact(_comms.SPI_TRANS::READ, RNenergyC);

/* System Status Registers */
unsigned short ATM90E32::GetSysStatus0()
{
  return _comms.transact(_comms.SPI_TRANS::READ, EMMIntState0);
}
unsigned short ATM90E32::GetSysStatus1()
{
  return _comms.transact(_comms.SPI_TRANS::READ, EMMIntState1);
}
unsigned short ATM90E32::GetMeterStatus0()
{
  return _comms.transact(_comms.SPI_TRANS::READ, EMMState0);
}
unsigned short ATM90E32::GetMeterStatus1()
{
  return _comms.transact(_comms.SPI_TRANS::READ, EMMState1);
}

void ATM90E32::begin()
{
  _comms.begin();

  printf("Connecting to ATM90E32\n");
#if defined(ENERGIA)
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
#endif
}

void ATM90E32::applyCalibration()
{
  // calculation for voltage sag threshold - assumes we do not want to go under 90v for split phase and 190v otherwise
  // determine proper low and high frequency threshold
  unsigned short vSagTh;
  unsigned short sagV;
  unsigned short FreqHiThresh;
  unsigned short FreqLoThresh;
  if (cal.mmode0 | MMode0_Freq60Hz)
  {
    sagV = 90;
    FreqHiThresh = 61 * 100;
    FreqLoThresh = 59 * 100;
  }
  else
  {
    sagV = 190;
    FreqHiThresh = 51 * 100;
    FreqLoThresh = 49 * 100;
  }

  // sqrt(2) = 1.4142135
  vSagTh = (sagV * 100 * 1.4142135) / (2 * cal.meas_gain_a.ugain / 32768);

  // Initialize registers
  _comms.transact(_comms.SPI_TRANS::WRITE, SoftReset, 0x789A);   // 70 Perform soft reset
  _comms.transact(_comms.SPI_TRANS::WRITE, CfgRegAccEn, 0x55AA); // 7F enable register config access
  _comms.transact(_comms.SPI_TRANS::WRITE, MeterEn, 0x0001);     // 00 Enable Metering

  _comms.transact(_comms.SPI_TRANS::WRITE, SagPeakDetCfg, 0x143F);  // 05 Sag and Voltage peak detect period set to 20ms
  _comms.transact(_comms.SPI_TRANS::WRITE, SagTh, vSagTh);          // 08 Voltage sag threshold
  _comms.transact(_comms.SPI_TRANS::WRITE, FreqHiTh, FreqHiThresh); // 0D High frequency threshold
  _comms.transact(_comms.SPI_TRANS::WRITE, FreqLoTh, FreqLoThresh); // 0C Lo frequency threshold
  _comms.transact(_comms.SPI_TRANS::WRITE, EMMIntEn0, 0xB76F);      // 75 Enable interrupts
  _comms.transact(_comms.SPI_TRANS::WRITE, EMMIntEn1, 0xDDFD);      // 76 Enable interrupts
  _comms.transact(_comms.SPI_TRANS::WRITE, EMMIntState0, 0x0001);   // 73 Clear interrupt flags
  _comms.transact(_comms.SPI_TRANS::WRITE, EMMIntState1, 0x0001);   // 74 Clear interrupt flags
  _comms.transact(_comms.SPI_TRANS::WRITE, ZXConfig, 0xD654);       // 07 ZX2, ZX1, ZX0 pin config - set to current channels, all polarity

  // Set metering config values (CONFIG)
  _comms.transact(_comms.SPI_TRANS::WRITE, PLconstH, 0x0861);   // 31 PL Constant MSB (default) - Meter Constant = 3200 - PL Constant = 140625000
  _comms.transact(_comms.SPI_TRANS::WRITE, PLconstL, 0xC468);   // 32 PL Constant LSB (default) - this is 4C68 in the application note, which is incorrect
  _comms.transact(_comms.SPI_TRANS::WRITE, MMode0, cal.mmode0); // 33 Mode Config (frequency set in main program)
  _comms.transact(_comms.SPI_TRANS::WRITE, MMode1, cal.mmode1); // 34 PGA Gain Configuration for Current Channels - 0x002A (x4) // 0x0015 (x2) // 0x0000 (1x)
  _comms.transact(_comms.SPI_TRANS::WRITE, PStartTh, 0x1D4C);   // 35 All phase Active Startup Power Threshold - 50% of startup current = 0.02A/0.00032 = 7500
  _comms.transact(_comms.SPI_TRANS::WRITE, QStartTh, 0x1D4C);   // 36 All phase Reactive Startup Power Threshold
  _comms.transact(_comms.SPI_TRANS::WRITE, SStartTh, 0x1D4C);   // 37 All phase Apparent Startup Power Threshold
  _comms.transact(_comms.SPI_TRANS::WRITE, PPhaseTh, 0x02EE);   // 38 Each phase Active Phase Threshold = 10% of startup current = 0.002A/0.00032 = 750
  _comms.transact(_comms.SPI_TRANS::WRITE, QPhaseTh, 0x02EE);   // 39 Each phase Reactive Phase Threshold
  _comms.transact(_comms.SPI_TRANS::WRITE, SPhaseTh, 0x02EE);   // 3A Each phase Apparent Phase Threshold

  // Set metering calibration values (CALIBRATION)
  _comms.transact(_comms.SPI_TRANS::WRITE, PQGainA, 0x0000);  // 47 Line calibration gain
  _comms.transact(_comms.SPI_TRANS::WRITE, PhiA, 0x0000);     // 48 Line calibration angle
  _comms.transact(_comms.SPI_TRANS::WRITE, PQGainB, 0x0000);  // 49 Line calibration gain
  _comms.transact(_comms.SPI_TRANS::WRITE, PhiB, 0x0000);     // 4A Line calibration angle
  _comms.transact(_comms.SPI_TRANS::WRITE, PQGainC, 0x0000);  // 4B Line calibration gain
  _comms.transact(_comms.SPI_TRANS::WRITE, PhiC, 0x0000);     // 4C Line calibration angle
  _comms.transact(_comms.SPI_TRANS::WRITE, PoffsetA, 0x0000); // 41 A line active power offset FFDC
  _comms.transact(_comms.SPI_TRANS::WRITE, QoffsetA, 0x0000); // 42 A line reactive power offset
  _comms.transact(_comms.SPI_TRANS::WRITE, PoffsetB, 0x0000); // 43 B line active power offset
  _comms.transact(_comms.SPI_TRANS::WRITE, QoffsetB, 0x0000); // 44 B line reactive power offset
  _comms.transact(_comms.SPI_TRANS::WRITE, PoffsetC, 0x0000); // 45 C line active power offset
  _comms.transact(_comms.SPI_TRANS::WRITE, QoffsetC, 0x0000); // 46 C line reactive power offset

  // Set metering calibration values (HARMONIC)
  _comms.transact(_comms.SPI_TRANS::WRITE, POffsetAF, 0x0000); // 51 A Fund. active power offset
  _comms.transact(_comms.SPI_TRANS::WRITE, POffsetBF, 0x0000); // 52 B Fund. active power offset
  _comms.transact(_comms.SPI_TRANS::WRITE, POffsetCF, 0x0000); // 53 C Fund. active power offset
  _comms.transact(_comms.SPI_TRANS::WRITE, PGainAF, 0x0000);   // 54 A Fund. active power gain
  _comms.transact(_comms.SPI_TRANS::WRITE, PGainBF, 0x0000);   // 55 B Fund. active power gain
  _comms.transact(_comms.SPI_TRANS::WRITE, PGainCF, 0x0000);   // 56 C Fund. active power gain

  // Set measurement calibration values (ADJUST)
  _comms.transact(_comms.SPI_TRANS::WRITE, UgainA, cal.meas_gain_a.ugain);       // 61 A Voltage rms gain
  _comms.transact(_comms.SPI_TRANS::WRITE, IgainA, cal.meas_gain_a.igain);       // 62 A line current gain
  _comms.transact(_comms.SPI_TRANS::WRITE, UoffsetA, cal.meas_offset_a.uoffset); // 63 A Voltage offset - 61A8
  _comms.transact(_comms.SPI_TRANS::WRITE, IoffsetA, cal.meas_offset_a.ioffset); // 64 A line current offset - FE60
  _comms.transact(_comms.SPI_TRANS::WRITE, UgainB, cal.meas_gain_b.ugain);       // 65 B Voltage rms gain
  _comms.transact(_comms.SPI_TRANS::WRITE, IgainB, cal.meas_gain_b.igain);       // 66 B line current gain
  _comms.transact(_comms.SPI_TRANS::WRITE, UoffsetB, cal.meas_offset_b.uoffset); // 67 B Voltage offset - 1D4C
  _comms.transact(_comms.SPI_TRANS::WRITE, IoffsetB, cal.meas_offset_b.ioffset); // 68 B line current offset - FE60
  _comms.transact(_comms.SPI_TRANS::WRITE, UgainC, cal.meas_gain_c.ugain);       // 69 C Voltage rms gain
  _comms.transact(_comms.SPI_TRANS::WRITE, IgainC, cal.meas_gain_c.igain);       // 6A C line current gain
  _comms.transact(_comms.SPI_TRANS::WRITE, UoffsetC, cal.meas_offset_c.uoffset); // 6B C Voltage offset - 1D4C
  _comms.transact(_comms.SPI_TRANS::WRITE, IoffsetC, cal.meas_offset_c.ioffset); // 6C C line current offset

  _comms.transact(_comms.SPI_TRANS::WRITE, CfgRegAccEn, 0x0000); // 7F end configuration
}

void ATM90E32::setCalibration(atm90e32_calibration &cal)
{
  this->cal = cal;
  applyCalibration();
}

void ATM90E32::readCalibration(atm90e32_calibration *cal)
{
  cal->mmode0 = _comms.transact(_comms.SPI_TRANS::READ, MMode0);
  cal->mmode1 = _comms.transact(_comms.SPI_TRANS::READ, MMode1);

  // Read channel A calibration registers
  cal->meas_gain_a.ugain = _comms.transact(_comms.SPI_TRANS::READ, UgainA);
  cal->meas_gain_a.igain = _comms.transact(_comms.SPI_TRANS::READ, IgainA);
  cal->meas_offset_a.uoffset = _comms.transact(_comms.SPI_TRANS::READ, UoffsetA);
  cal->meas_offset_a.ioffset = _comms.transact(_comms.SPI_TRANS::READ, IoffsetA);

  // Read channel B calibration registers
  cal->meas_gain_b.ugain = _comms.transact(_comms.SPI_TRANS::READ, UgainB);
  cal->meas_gain_b.igain = _comms.transact(_comms.SPI_TRANS::READ, IgainB);
  cal->meas_offset_b.uoffset = _comms.transact(_comms.SPI_TRANS::READ, UoffsetB);
  cal->meas_offset_b.ioffset = _comms.transact(_comms.SPI_TRANS::READ, IoffsetB);

  // Read channel C calibration registers
  cal->meas_gain_c.ugain = _comms.transact(_comms.SPI_TRANS::READ, UgainC);
  cal->meas_gain_c.igain = _comms.transact(_comms.SPI_TRANS::READ, IgainC);
  cal->meas_offset_c.uoffset = _comms.transact(_comms.SPI_TRANS::READ, UoffsetC);
  cal->meas_offset_c.ioffset = _comms.transact(_comms.SPI_TRANS::READ, IoffsetC);
}

void ATM90E32::getCalibration(atm90e32_calibration *cal)
{
  readCalibration(cal);
}