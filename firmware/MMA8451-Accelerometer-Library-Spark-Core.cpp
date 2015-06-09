/******************************************************************************
MMA8452-Accelerometer-Library-Spark-Core.cpp
MMA8452-Accelerometer-Library-Spark-Core Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: June 3, 2014
Modified by Dan Dawson: December 19, 2014
Modified by Elliot Smith: June 9, 2015
Original Source:
https://github.com/sparkfun/MMA8452_Accelerometer

This file implements all functions of the MMA8451Q class. Functions here range
from higher level stuff, like reading/writing MMA8451Q registers to low-level,
hardware I2C reads and writes.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Uno

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "MMA8451-Accelerometer-Library-Spark-Core.h"
/* #include <Arduino.h> */	// Not needed for Spark Core
/* #include <Wire.h> */		// Not needed for Spark Core

// CONSTRUCTUR
//   This function, called when you initialize the class will simply write the
//   supplied address into a private variable for future use.
//   The variable addr should be either 0x1C or 0x1D, depending on which voltage
//   the SA0 pin is tied to (GND or 3.3V respectively).
MMA8451Q::MMA8451Q(byte addr)
{
	address = addr; // Store address into private variable
}

// INITIALIZATION
//	This function initializes the MMA8451Q. It sets up the scale (either 2, 4,
//	or 8g), output data rate, portrait/landscape detection and tap detection.
//	It also checks the WHO_AM_I register to make sure we can communicate with
//	the sensor. Returns a 0 if communication failed, 1 if successful.
byte MMA8451Q::init(MMA8451Q_Scale fsr, MMA8451Q_ODR odr)
{
	scale = fsr; // Haul fsr into our class variable, scale
	
	Wire.begin(); // Initialize I2C
	
	byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
	
	if (c != 0x1A) // WHO_AM_I should always be 0x1A for the 8451Q, 2A for the other
	{
		return 0;
	}
	
	standby();  // Must be in standby to change registers
	
	setScale(scale);  // Set up accelerometer scale
	setODR(odr);  // Set up output data rate
	setupPL();  // Set up portrait/landscape detection
	// Multiply parameter by 0.0625g to calculate threshold.
	setupTap(0x80, 0x80, 0x08); // Disable x, y, set z to 0.5g
	
	active();  // Set to active to start reading
	
	return 1;
}

// READ ACCELERATION DATA
//  This function will read the acceleration values from the MMA8451Q. After
//	reading, it will update two triplets of variables:
//		* int's x, y, and z will store the signed 14-bit values read out
//		  of the acceleromter.
//		* floats cx, cy, and cz will store the calculated acceleration from
//		  those 14-bit values. These variables are in units of g's.
void MMA8451Q::read()
{
	byte rawData[6];  // x/y/z accel register data stored here

	readRegisters(OUT_X_MSB, rawData, 6);  // Read the six raw data registers into data array
	
	x = (rawData[0]<<8 | rawData[1]) >> 2;
	y = (rawData[2]<<8 | rawData[3]) >> 2;
	z = (rawData[4]<<8 | rawData[5]) >> 2;
	cx = (float) x / (float)(1<<13) * (float)(scale);
	cy = (float) y / (float)(1<<13) * (float)(scale);
	cz = (float) z / (float)(1<<13) * (float)(scale);
}

// CHECK IF NEW DATA IS AVAILABLE
//	This function checks the status of the MMA8451Q to see if new data is availble.
//	returns 0 if no new data is present, or a 1 if new data is available.
byte MMA8451Q::available()
{
	return (readRegister(STATUS) & 0x08) >> 3;
}

// SET FULL-SCALE RANGE
//	This function sets the full-scale range of the x, y, and z axis accelerometers.
//	Possible values for the fsr variable are SCALE_2G, SCALE_4G, or SCALE_8G.
void MMA8451Q::setScale(MMA8451Q_Scale fsr)
{
	// Must be in standby mode to make changes!!!
	byte cfg = readRegister(XYZ_DATA_CFG);
	cfg &= 0xFC; // Mask out scale bits
	cfg |= (fsr >> 2);  // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
	writeRegister(XYZ_DATA_CFG, cfg);
}

// SET THE OUTPUT DATA RATE
//	This function sets the output data rate of the MMA8451Q.
//	Possible values for the odr parameter are: ODR_800, ODR_400, ODR_200, 
//	ODR_100, ODR_50, ODR_12, ODR_6, or ODR_1
void MMA8451Q::setODR(MMA8451Q_ODR odr)
{
	// Must be in standby mode to make changes!!!
	byte ctrl = readRegister(CTRL_REG1);
	ctrl &= 0xCF; // Mask out data rate bits
	ctrl |= (odr << 3);
	writeRegister(CTRL_REG1, ctrl);
}

// SET UP TAP DETECTION
//	This function can set up tap detection on the x, y, and/or z axes.
//	The xThs, yThs, and zThs parameters serve two functions:
//		1. Enable tap detection on an axis. If the 7th bit is SET (0x80)
//			tap detection on that axis will be DISABLED.
//		2. Set tap g's threshold. The lower 7 bits will set the tap threshold
//			on that axis.
void MMA8451Q::setupTap(byte xThs, byte yThs, byte zThs)
{
	// Set up single and double tap - 5 steps:
	// for more info check out this app note:
	// http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
	// Set the threshold - minimum required acceleration to cause a tap.
	byte temp = 0;
	if (!(xThs & 0x80)) // If top bit ISN'T set
	{
		temp |= 0x3; // Enable taps on x
		writeRegister(PULSE_THSX, xThs);  // x thresh
	}
	if (!(yThs & 0x80))
	{
		temp |= 0xC; // Enable taps on y
		writeRegister(PULSE_THSY, yThs);  // y thresh
	}
	if (!(zThs & 0x80))
	{
		temp |= 0x30; // Enable taps on z
		writeRegister(PULSE_THSZ, zThs);  // z thresh
	}
	// Set up single and/or double tap detection on each axis individually.
	writeRegister(PULSE_CFG, temp | 0x40);
	// Set the time limit - the maximum time that a tap can be above the thresh
	writeRegister(PULSE_TMLT, 0x30);  // 30ms time limit at 800Hz odr
	// Set the pulse latency - the minimum required time between pulses
	writeRegister(PULSE_LTCY, 0xA0);  // 200ms (at 800Hz odr) between taps min
	// Set the second pulse window - maximum allowed time between end of
	//	latency and start of second pulse
	writeRegister(PULSE_WIND, 0xFF);  // 5. 318ms (max value) between taps max
}

// READ TAP STATUS
//	This function returns any taps read by the MMA8451Q. If the function 
//	returns no new taps were detected. Otherwise the function will return the
//	lower 7 bits of the PULSE_SRC register.
byte MMA8451Q::readTap()
{
	byte tapStat = readRegister(PULSE_SRC);
	if (tapStat & 0x80) // Read EA bit to check if a interrupt was generated
	{
		return tapStat & 0x7F;
	}
	else
		return 0;
}

// SET UP PORTRAIT/LANDSCAPE DETECTION
//	This function sets up portrait and landscape detection.
void MMA8451Q::setupPL()
{
	// Must be in standby mode to make changes!!!
	// For more info check out this app note:
	//	http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
	// 1. Enable P/L
	writeRegister(PL_CFG, readRegister(PL_CFG) | 0x40); // Set PL_EN (enable)
	// 2. Set the debounce rate
	writeRegister(PL_COUNT, 0x50);  // Debounce counter at 100ms (at 800 hz)
}

// READ PORTRAIT/LANDSCAPE STATUS
//	This function reads the portrait/landscape status register of the MMA8451Q.
//	It will return either PORTRAIT_U, PORTRAIT_D, LANDSCAPE_R, LANDSCAPE_L,
//	or LOCKOUT. LOCKOUT indicates that the sensor is in neither p or ls.
byte MMA8451Q::readPL()
{
	byte plStat = readRegister(PL_STATUS);
	
	if (plStat & 0x40) // Z-tilt lockout
		return LOCKOUT;
	else // Otherwise return LAPO status
		return (plStat & 0x6) >> 1;
}

// SET STANDBY MODE
//	Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8451Q::standby()
{
	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// SET ACTIVE MODE
//	Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8451Q::active()
{
	byte c = readRegister(CTRL_REG1);
	writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// WRITE A SINGLE REGISTER
// 	Write a single byte of data to a register in the MMA8451Q.
void MMA8451Q::writeRegister(MMA8451Q_Register reg, byte data)
{
	writeRegisters(reg, &data, 1);
}

// WRITE MULTIPLE REGISTERS
//	Write an array of "len" bytes ("buffer"), starting at register "reg", and
//	auto-incrmenting to the next.
void MMA8451Q::writeRegisters(MMA8451Q_Register reg, byte *buffer, byte len)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	for (int x = 0; x < len; x++)
		Wire.write(buffer[x]);
	Wire.endTransmission(); //Stop transmitting
}

// READ A SINGLE REGISTER
//	Read a byte from the MMA8451Q register "reg".
byte MMA8451Q::readRegister(MMA8451Q_Register reg)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission(false); //endTransmission but keep the connection active

	Wire.requestFrom(address, (byte) 1); //Ask for 1 byte, once done, bus is released by default

	while(!Wire.available()) ; //Wait for the data to come back

	return Wire.read(); //Return this one byte
}

// READ MULTIPLE REGISTERS
//	Read "len" bytes from the MMA8451Q, starting at register "reg". Bytes are stored
//	in "buffer" on exit.
void MMA8451Q::readRegisters(MMA8451Q_Register reg, byte *buffer, byte len)
{
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission(false); //endTransmission but keep the connection active

	Wire.requestFrom(address, len); //Ask for bytes, once done, bus is released by default

	while(Wire.available() < len); //Hang out until we get the # of bytes we expect

	for(int x = 0 ; x < len ; x++)
		buffer[x] = Wire.read();    
}
