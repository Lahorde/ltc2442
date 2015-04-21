/*!
LTC2449: 24-Bit, 16-Channel Delta Sigma ADCs with Selectable Speed/Resolution.

@verbatim

The LTC2444/LTC2445/LTC2448/LTC2449 are 8-/16-channel (4-/8-differential)
high speed 24-bit No Latency Delta Sigma ADCs. They use a proprietary
delta-sigma architecture enabling variable speed/resolution. Through a
simple 4-wire serial interface, ten speed/resolution combinations
6.9Hz/280nVRMS to 3.5kHz/25uVRMS (4kHz with external oscillator) can be
selected with no latency between conversion results or shift in DC accuracy
(offset, full-scale, linearity, drift). Additionally, a 2X speed mode can
be selected enabling output rates up to 7kHz (8kHz if an external
oscillator is used) with one cycle latency.

@endverbatim

http://www.linear.com/product/LTC2449

http://www.linear.com/product/LTC2449#demoboards

REVISION HISTORY
$Revision: 2026 $
$Date: 2013-10-14 13:52:48 -0700 (Mon, 14 Oct 2013) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

//! @defgroup LTC2449 LTC2449: 24-Bit, 16-Channel Delta Sigma ADCs with Selectable Speed/Resolution

/*! @file
    @ingroup LTC2449
    Library for LTC2449: 24-Bit, 16-Channel Delta Sigma ADCs with Selectable Speed/Resolution
*/

#include <stdint.h>
#include <Arduino.h>
#include <PinChangeInt.h>
#include <logger.h>
#include "pinout.h"
#include "LT_SPI.h"
#include "LTC2449.h"
#include "events.h"

#define ADC_DATA_AND_SUB_LSB_MASK     ((1UL << (CONVERSION_24_BITS_POS + LTC2449_RESOLUTION)) - 1)
#define ADC_NEGATIVE_VALUE_MASK       ((1UL << EOC_BIT_POS) | (1UL << DUMMY_BIT_POS) | (1UL << SIGN_BIT_POS))

//! Lookup table to build the command for single-ended mode
const uint16_t BUILD_COMMAND_SINGLE_ENDED[16] = {LTC2449_CH0, LTC2449_CH1, LTC2449_CH2, LTC2449_CH3,
    LTC2449_CH4, LTC2449_CH5, LTC2449_CH6, LTC2449_CH7,
    LTC2449_CH8, LTC2449_CH9, LTC2449_CH10, LTC2449_CH11,
    LTC2449_CH12, LTC2449_CH13, LTC2449_CH14, LTC2449_CH15
                                                };    //!< Builds the command for single-ended mode

//! Lookup table to build the command for differential mode
const uint16_t BUILD_COMMAND_DIFF[16] = {LTC2449_P0_N1, LTC2449_P2_N3, LTC2449_P4_N5, LTC2449_P6_N7,
                                        LTC2449_P8_N9, LTC2449_P10_N11, LTC2449_P12_N13, LTC2449_P14_N15,
                                        LTC2449_P1_N0, LTC2449_P3_N2, LTC2449_P5_N4, LTC2449_P7_N6,
                                        LTC2449_P9_N8, LTC2449_P11_N10, LTC2449_P13_N12, LTC2449_P15_N14
                                        };      //!< Build the command for differential mode

//! Lookup table to build the command for OSR
const uint16_t BUILD_OSR_COMMAND[10] = {LTC2449_OSR_32768, LTC2449_OSR_64, LTC2449_OSR_128, LTC2449_OSR_256, LTC2449_OSR_512,
                                        LTC2449_OSR_1024, LTC2449_OSR_2048, LTC2449_OSR_4096, LTC2449_OSR_8192, LTC2449_OSR_16384
                                       };       //!< Build the command for OSR

//! Lookup table to build 1X / 2X bits
const uint16_t BUILD_1X_2X_COMMAND[2] = {LTC2449_SPEED_1X, LTC2449_SPEED_2X};   //!< Build the command for 1x or 2x mode

//! MISO timeout constant
const uint16_t MISO_TIMEOUT = 1000;

static int16_t OSR_mode = LTC2449_OSR_4096;    //!< The LTC2449 OSR setting
static int16_t two_x_mode = LTC2449_SPEED_1X;   //!< The LTC2449 2X Mode settings


LTC2449::LTC2449(ILTC2449Listener* arg_p_conv_listener) :
		_u16_adc_cmd(0x00),
		_u32_adc_code(0x00),
		_last_conv(),
		_b_trash_next_conv(false)
{
	if(arg_p_conv_listener != NULL)
	{
		register_listener(arg_p_conv_listener);
	}
	EventManager::getInstance()->addListener(ADC_CODE_READY_EVENT, this);
}

LTC2449::~LTC2449()
{
	EventManager::getInstance()->removeListener(ADC_CODE_READY_EVENT, this);
	unregister_listener(_p_conv_listener);

	_last_conv = LTC2449Conversion();
	_u32_adc_code = 0x00;
	_u16_adc_cmd = 0x00;
}

//! Read channels in single-ended mode
LTC2449::EError LTC2449::read_single_ended(uint8_t arg_u8_channel, int32_t* arg_s32_adc_value)
{
	uint16_t adc_command;     // The LTC2449 command word
	uint32_t u32_adc_code = 0; // The LTC2449 code

	adc_command = BUILD_COMMAND_SINGLE_ENDED[arg_u8_channel] | OSR_mode | two_x_mode;
	LOG_DEBUG_LN("\nADC Command: B%b", adc_command);
	if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))     // Checks for EOC with a timeout
		return(CONVERSION_TIME_OUT);
	read(LTC2449_CS, adc_command, &u32_adc_code);     // Throws out last reading
	if (two_x_mode)
	{
		read(LTC2449_CS, adc_command, &u32_adc_code);   // Throws out an extra reading in 2x mode
		if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))   // Checks for EOC with a timeout
			return(CONVERSION_TIME_OUT);
	}
	if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))     // Checks for EOC with a timeout
		return(CONVERSION_TIME_OUT);

	read(LTC2449_CS, adc_command, &u32_adc_code);     // Now we're ready to read the desired data
	LOG_DEBUG_LN("Channel %d - Received Code: 0x%x", arg_u8_channel, u32_adc_code);

	return adc_code_to_value(u32_adc_code, arg_s32_adc_value);
}

//! Read channels in single-ended mode
LTC2449::EError LTC2449::read_single_ended(uint8_t arg_u8_channel)
{
	uint32_t u32_adc_code = 0; // The LTC2449 code

	_u16_adc_cmd = BUILD_COMMAND_SINGLE_ENDED[arg_u8_channel] | OSR_mode | two_x_mode;
	_last_conv.setChannel(arg_u8_channel);

	/** Synchronously throws out last readings */
	LOG_DEBUG_LN(F("\nADC Command: B%b"), _u16_adc_cmd);

	if(!digitalRead(MISO))
	{
		/** conversion ready - read it and trash it */
		read(LTC2449_CS, _u16_adc_cmd, &u32_adc_code);
		_b_trash_next_conv = false;
	}
	else
	{
		/** conversion on going - remember to trash it */
		_b_trash_next_conv = true;
	}

	/** wait next conversion */
	EventManager::getInstance()->enableListener(ADC_CODE_READY_EVENT, this, true);
	PCintPort::attachInterrupt(MISO, (PCIntvoidFuncPtr) (&endOfConversion), FALLING, this);

	return NO_ERROR;
}

//! Read channels in differential mode
//! @return 0 if successful, 1 is failure

LTC2449::EError LTC2449::read_differential(EDiffPair arg_e_diffPair, int32_t* arg_s32_adc_value)
{
	uint32_t u32_adc_code = 0;    // The LTC2449 code

    // Reads and displays a selected channel
	_u16_adc_cmd = BUILD_COMMAND_DIFF[arg_e_diffPair] | OSR_mode | two_x_mode;
    LOG_DEBUG_LN(F("\nADC Command: B%b"), adc_command);
    if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))             // Checks for EOC with a timeout
      return(CONVERSION_TIME_OUT);
    read(LTC2449_CS, _u16_adc_cmd, &u32_adc_code);             // Throws out last reading
    if (two_x_mode)
    {
      if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))           // Checks for EOC with a timeout
        return(CONVERSION_TIME_OUT);
      read(LTC2449_CS, _u16_adc_cmd, &u32_adc_code);           // Throws out an extra reading in 2x mode
    }
    if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))             // Checks for EOC with a timeout
      return(CONVERSION_TIME_OUT);
    read(LTC2449_CS, _u16_adc_cmd, &_u32_adc_code);
    LOG_DEBUG_LN("Received Code: 0x%x", _u32_adc_code);

    return adc_code_to_value(_u32_adc_code, arg_s32_adc_value);
}

LTC2449::EError LTC2449::read_differential(EDiffPair arg_e_diffPair)
{
	uint32_t u32_adc_code = 0;    // The LTC2449 code

	/** Synchronously throws out last readings */
	_last_conv.setDiffPair(arg_e_diffPair);
	_u16_adc_cmd = BUILD_COMMAND_DIFF[arg_e_diffPair] | OSR_mode | two_x_mode;
	LOG_DEBUG_LN(F("\nADC Command: B%b"), _u16_adc_cmd);

	if(!digitalRead(MISO))
	{
		/** conversion ready - read it and trash it */
		read(LTC2449_CS, _u16_adc_cmd, &u32_adc_code);
		_b_trash_next_conv = false;
	}
	else
	{
		/** conversion on going - remember to trash it */
		_b_trash_next_conv = true;
	}

	// Now we're ready to start capture
	EventManager::getInstance()->enableListener(ADC_CODE_READY_EVENT, this, true);
	PCintPort::attachInterrupt(MISO, (PCIntvoidFuncPtr) (&endOfConversion), FALLING, this);
	return NO_ERROR;
}


int8_t LTC2449::EOC_timeout(uint8_t cs, uint16_t miso_timeout)
// Checks for EOC with a specified timeout (ms)
{
  uint16_t timer_count = 0;             // Timer count for MISO
  digitalWrite(cs, LOW);                       //! 1) Pull CS low
  while (1)                             //! 2) Wait for SDO (MISO) to go low
  {
    if (digitalRead(MISO) == 0) break;        //! 3) If SDO is low, break loop
    if (timer_count++>miso_timeout)     // If timeout, return 1 (failure)
    {
    	digitalWrite(cs, HIGH);                  // Pull CS high
    	LOG_INFO_LN(F("timeout"));
    	return(1);
    }
    else
      delay(1);
  }
  digitalWrite(cs, HIGH);                  // Pull CS high
  return(0);
}

// Reads from LTC2449.
void LTC2449::read(uint8_t cs, uint16_t adc_command, uint32_t *adc_code)
{
  uint8_t data[4];
  uint8_t command[4];

  command[3] = (adc_command >> 8) & 0xFF;
  command[2] = adc_command & 0xFF;
  command[1] = 0;
  command[0] = 0;

  spi_transfer_block(cs, command, data, (uint8_t)4);

  *adc_code = (((uint32_t)data[3] & 0xFF) << 24) | (((uint32_t)data[2] & 0xFF) << 16) | (((uint32_t)data[1] & 0xFF) << 8) | ((uint32_t)data[0] & 0xFF);
}

// Calculates the voltage corresponding to an adc code, given lsb weight (in volts) and the calibrated
// adc offset code (zero code that is subtracted from adc_code).
float LTC2449::code_to_voltage(int32_t adc_code, float LTC2449_lsb, int32_t LTC2449_offset_code)
{
  float adc_voltage;
  adc_code -= 536870912;                                            //! 1) Converts offset binary to binary
  adc_voltage=(float)(adc_code+LTC2449_offset_code)*LTC2449_lsb;    //! 2) Calculate voltage from ADC code, lsb, offset.
  return(adc_voltage);
}

// Calculate the lsb weight and offset code given a full-scale code and a measured zero-code.
void LTC2449::cal_voltage(int32_t zero_code, int32_t fs_code, float zero_voltage, float fs_voltage, float *LTC2449_lsb, int32_t *LTC2449_offset_code)
{
  zero_code -= 536870912;   //! 1) Converts zero code from offset binary to binary
  fs_code -= 536870912;     //! 2) Converts full scale code from offset binary to binary
  
  float temp_offset;
  *LTC2449_lsb = (fs_voltage-zero_voltage)/((float)(fs_code - zero_code));                              //! 3) Calculate the LSB
  
  temp_offset = (zero_voltage/ *LTC2449_lsb) - zero_code;                                               //! 4) Calculate Unipolar offset
  temp_offset = (temp_offset > (floor(temp_offset) + 0.5)) ? ceil(temp_offset) : floor(temp_offset);    //! 5) Round
  *LTC2449_offset_code = (int32_t)temp_offset;                                                          //! 6) Cast as int32_t
}

LTC2449::EError LTC2449::adc_code_to_value(uint32_t arg_u32_adcCode, int32_t* arg_ps32_adcValue)
{
	bool loc_b_positive = (arg_u32_adcCode >> SIGN_BIT_POS) & 1;
	*arg_ps32_adcValue = 0;

	if( arg_u32_adcCode & (1UL << EOC_BIT_POS)){
		return CONVERSION_ERROR;
	}

	if(loc_b_positive && (arg_u32_adcCode & (1UL << (CONVERSION_24_BITS_POS + LTC2449_RESOLUTION -1))))
	{
		return CONVERSION_OVERRANGE;
	}
	else if(!loc_b_positive && !(arg_u32_adcCode & (1UL << (CONVERSION_24_BITS_POS + LTC2449_RESOLUTION -1))))
	{
		return CONVERSION_UNDERRANGE;
	}

	/** Apply mask to get adc value including sub-lsb bits */
	*arg_ps32_adcValue |= arg_u32_adcCode &  ADC_DATA_AND_SUB_LSB_MASK;

	/** Set sign */
	if(!loc_b_positive)
	{
		*arg_ps32_adcValue |= ADC_NEGATIVE_VALUE_MASK;
	}

	return NO_ERROR;
}

/**
 * CALLED UNDER INTERRUPT CONTEXT
 */
void LTC2449::endOfConversion(LTC2449* arg_p_ltc2449)
{
	uint32_t loc_u32_conv_to_trash = 0;

	PCintPort::detachInterrupt(MISO);

	/** Be sure of LOW pin state */
	if(!digitalRead(MISO))
	{
		if(arg_p_ltc2449->_b_trash_next_conv)
		{
			arg_p_ltc2449->read(LTC2449_CS, arg_p_ltc2449->_u16_adc_cmd, &loc_u32_conv_to_trash);
			arg_p_ltc2449->_b_trash_next_conv = false;
			PCintPort::attachInterrupt(MISO, (PCIntvoidFuncPtr) (&endOfConversion), FALLING, arg_p_ltc2449);
		}
		else
		{
			/** Read and process data later (in normal mode) */
			EventManager::getInstance()->queueEvent( ADC_CODE_READY_EVENT,
				(int) arg_p_ltc2449,
				EventManager::kHighPriority );
		}
	}
	else
	{
		PCintPort::attachInterrupt(MISO, (PCIntvoidFuncPtr) (&endOfConversion), FALLING, arg_p_ltc2449);
	}
}

void LTC2449::processEvent(uint8_t eventCode, int eventParam)
{
	if(eventCode == ADC_CODE_READY_EVENT)
	{
		read(LTC2449_CS, _u16_adc_cmd, &_u32_adc_code);
		adc_code_to_value(_u32_adc_code, &_last_conv._s32_conv_value);

		if(_p_conv_listener != NULL)
		{
			_p_conv_listener->conversionAvailable(_last_conv);
		}
	}
	else
	{
		/** should never be here */
		ASSERT(false);
	}
}
