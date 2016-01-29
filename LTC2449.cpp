/******************************************************************************
 * @file    LTC2449.cpp
 * @author  Rémi Pincent - INRIA
 * @date    15 avr. 2015
 *
 * @brief Driver for LTC2449: 24-Bit, 16-Channel Delta Sigma ADCs with Selectable Speed/Resolution.
 * Initial version from : http://www.linear.com/product/LTC2442#code
 * Raspberry implementation
 * Refer copyright below
 *
 * Project : ltc2449
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History: refer https://github.com/Lahorde/ltc2442
 *
 * Copyright (c) 2013, Linear Technology Corp.(LTC)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of Linear Technology Corp.
 *
 * The Linear Technology Linduino is not affiliated with the official Arduino team.
 * However, the Linduino is only possible because of the Arduino team's commitment
 * to the open-source community.  Please, visit http://www.arduino.cc and
 * http://store.arduino.cc , and consider a purchase that will help fund their
 * ongoing work.
 * http://www.linear.com/product/LTC2449
 *
 * http://www.linear.com/product/LTC2449#demoboards
 *****************************************************************************/


//! @defgroup LTC2449 LTC2449: 24-Bit, 16-Channel Delta Sigma ADCs with Selectable Speed/Resolution

/*! @file
    @ingroup LTC2449
    Library for LTC2449: 24-Bit, 16-Channel Delta Sigma ADCs with Selectable Speed/Resolution
*/

#include <stdint.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <log4c.h>

#include "LTC2449.h"
#include "pinout.h"
#include "events.h"

#include <stdio.h>

#define ADC_DATA_AND_SUB_LSB_MASK     ((1UL << (CONVERSION_24_BITS_POS + LTC2449_RESOLUTION)) - 1)
#define ADC_NEGATIVE_VALUE_MASK       ((1UL << EOC_BIT_POS) | (1UL << DUMMY_BIT_POS) | (1UL << SIGN_BIT_POS))

/** default SPI on rpi */
#ifndef LTC_2449_SPI_CHANNEL
#define LTC_2449_SPI_CHANNEL 0
#endif

/** SPI Mode is 0 for LTC244x */
#define LTC_2449_SPI_MODE 0

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

static int16_t OSR_mode = LTC2449_OSR_16384;    //!< The LTC2449 OSR setting
static int16_t two_x_mode = LTC2449_SPEED_1X;   //!< The LTC2449 2X Mode settings
static int32_t SPI_SPEED = 4000000;            //!< SPI speed

/** LOGGER */
static const char* FILE_NAME_LOGGER = "LTC2449";
static const log4c_category_t* file_cat;

LTC2449::LTC2449(ILTC2449Listener* arg_p_conv_listener) :
  _u16_adc_cmd(0x00),
  _u32_adc_code(0x00),
  _p_conv_listener(NULL),
  _next_conv(),
  _async_conv()
{
  file_cat = log4c_category_get(FILE_NAME_LOGGER);

  if(arg_p_conv_listener != NULL)
  {
    register_listener(arg_p_conv_listener);
  }
  wiringPiSPISetup(LTC_2449_SPI_MODE, SPI_SPEED);
  EventManager::getInstance()->addListener(ADC_CODE_READY_EVENT, this);
}

LTC2449::~LTC2449()
{
  wiringPiSPIClose(LTC_2449_SPI_CHANNEL);
  EventManager::getInstance()->removeListener(ADC_CODE_READY_EVENT, this);
  unregister_listener(_p_conv_listener);

  _next_conv = LTC2449Conversion();
  _u32_adc_code = 0x00;
  _u16_adc_cmd = 0x00;
}

//! Read channels in single-ended mode
LTC2449::EError LTC2449::read_single_ended(uint8_t arg_u8_channel, int32_t* arg_s32_adc_value)
{
  uint16_t adc_command;     // The LTC2449 command word
  uint32_t u32_adc_code = 0; // The LTC2449 code

  /** In all cases first 3 bits set to 101 => current conversion channel / speed are
  * those selected in previous read command
  */
  adc_command = BUILD_COMMAND_SINGLE_ENDED[arg_u8_channel] | OSR_mode | two_x_mode;
  _next_conv = LTC2449Conversion();
  _next_conv.setChannel(arg_u8_channel);

  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,
                     "Single ended sync - ADC Command: 0x%x",
                     adc_command);

  if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))     // Checks for EOC with a timeout
    return(CONVERSION_TIME_OUT);
  read(LTC2449_CS, adc_command, _next_conv, &u32_adc_code);     // Throws out last reading

  if (two_x_mode)
  {
    read(LTC2449_CS, adc_command, _next_conv, &u32_adc_code);   // Throws out an extra reading in 2x mode
    if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))   // Checks for EOC with a timeout
      return(CONVERSION_TIME_OUT);
  }
  if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))     // Checks for EOC with a timeout
    return(CONVERSION_TIME_OUT);

  read(LTC2449_CS, adc_command, _next_conv, &u32_adc_code);     // Now we're ready to read the desired data
  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,
                     "Channel %d - Received Code: %d",
                     arg_u8_channel,
                     u32_adc_code);

  return adc_code_to_value(u32_adc_code, arg_s32_adc_value);
}

//! Read channels in single-ended mode
LTC2449::EError LTC2449::read_single_ended(uint8_t arg_u8_channel)
{
  /** In all cases first 3 bits set to 101 => current conversion channel / speed are
   * those selected in previous read command
   */
  _u16_adc_cmd = BUILD_COMMAND_SINGLE_ENDED[arg_u8_channel] | OSR_mode | two_x_mode;

  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,
                     "Single ended async on channel %d - ADC Command: 0x%x",
                     arg_u8_channel,
                     _u16_adc_cmd);

  _async_conv = LTC2449Conversion();
  _async_conv.setChannel(arg_u8_channel);
  prepareAsyncRead();

  return NO_ERROR;
}

//! Read channels in differential mode
//! @return 0 if successful, 1 is failure

LTC2449::EError LTC2449::read_differential(EDiffPair arg_e_diffPair, int32_t* arg_s32_adc_value)
{
  uint32_t u32_adc_code = 0;    // The LTC2449 code

  /** In all cases first 3 bits set to 101 => current conversion channel / speed are
   * those selected in previous read command
   */
  _u16_adc_cmd = BUILD_COMMAND_DIFF[arg_e_diffPair] | OSR_mode | two_x_mode;
  _next_conv = LTC2449Conversion();
  _next_conv.setDiffPair(arg_e_diffPair);
  
  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,
                     "Diff sync - ADC Command: 0x%x",
                     _u16_adc_cmd);

  if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))             // Checks for EOC with a timeout
    return(CONVERSION_TIME_OUT);
    read(LTC2449_CS, _u16_adc_cmd, _next_conv, &u32_adc_code);             // Throws out last reading
    if (two_x_mode)
  {
    if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))           // Checks for EOC with a timeout
        return(CONVERSION_TIME_OUT);
      read(LTC2449_CS, _u16_adc_cmd, _next_conv, &u32_adc_code);           // Throws out an extra reading in 2x mode
    }
  if(EOC_timeout(LTC2449_CS, MISO_TIMEOUT))             // Checks for EOC with a timeout
    return(CONVERSION_TIME_OUT);
    read(LTC2449_CS, _u16_adc_cmd, _next_conv, &_u32_adc_code);
    
    log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,
                     "Received Code: 0x%x", 
                     _u32_adc_code);

    return adc_code_to_value(_u32_adc_code, arg_s32_adc_value);
  }

LTC2449::EError LTC2449::read_differential(EDiffPair arg_e_diffPair)
{
  /** In all cases first 3 bits set to 101 => current conversion channel / speed are
   * those selected in previous read command
   */
  _u16_adc_cmd = BUILD_COMMAND_DIFF[arg_e_diffPair] | OSR_mode | two_x_mode;
  
  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,
                     "Diff async on pair %d- ADC Command: 0x%x", 
                     arg_e_diffPair, 
                     _u16_adc_cmd);
  
  _async_conv = LTC2449Conversion();
  _async_conv.setDiffPair(arg_e_diffPair);
  prepareAsyncRead();

  return NO_ERROR;
}


int8_t LTC2449::EOC_timeout(uint8_t cs, uint16_t miso_timeout)
// Checks for EOC with a specified timeout (ms)
{
  uint16_t timer_count = 0;             // Timer count for MISO
  digitalWrite(cs, LOW);                //! 1) Pull CS low
  while (1)                             //! 2) Wait for SDO (MISO) to go low
  {
    if (digitalRead(LTC2449_MISO) == 0) break;        //! 3) If SDO is low, break loop
    if (timer_count++ > miso_timeout)   // If timeout, return 1 (failure)
    {
      digitalWrite(cs, HIGH);                  // Pull CS high
      return(1);
    }
    else
      delay(1);
  }
  digitalWrite(cs, HIGH);                  // Pull CS high
  return(0);
}

// Reads from LTC2449.
void LTC2449::read(uint8_t cs, uint16_t adc_command, LTC2449Conversion& arg_p_nextConv, uint32_t *adc_code)
{
  uint8_t data[4];

  data[0] = (adc_command >> 8) & 0xFF;
  data[1] = adc_command & 0xFF;
  data[2] = 0;
  data[3] = 0;
  
  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,
                     "read ADC command: 0x%x",
                     adc_command);

  wiringPiSPIDataRW(LTC_2449_SPI_CHANNEL, data, 4);
  _next_conv = arg_p_nextConv;
  *adc_code = (((uint32_t)data[0] & 0xFF) << 24) | (((uint32_t)data[1] & 0xFF) << 16) | (((uint32_t)data[2] & 0xFF) << 8) | ((uint32_t)data[3] & 0xFF);
}

// Calculates the voltage corresponding to an adc code, given lsb weight (in volts) and the calibrated
// adc offset code (zero code that is subtracted from adc_code).
float LTC2449::code_to_voltage(int32_t adc_code, float LTC2449_lsb, int32_t LTC2449_offset_code)
{
  float adc_voltage;
  adc_code -= 536870912;                                            //! 1) Converts offset binary to binary
  adc_voltage = (float)(adc_code + LTC2449_offset_code) * LTC2449_lsb; //! 2) Calculate voltage from ADC code, lsb, offset.
  return(adc_voltage);
}

// Calculate the lsb weight and offset code given a full-scale code and a measured zero-code.
void LTC2449::cal_voltage(int32_t zero_code, int32_t fs_code, float zero_voltage, float fs_voltage, float *LTC2449_lsb, int32_t *LTC2449_offset_code)
{
  zero_code -= 536870912;   //! 1) Converts zero code from offset binary to binary
  fs_code -= 536870912;     //! 2) Converts full scale code from offset binary to binary

  float temp_offset;
  *LTC2449_lsb = (fs_voltage - zero_voltage) / ((float)(fs_code - zero_code));                          //! 3) Calculate the LSB

  temp_offset = (zero_voltage / *LTC2449_lsb) - zero_code;                                              //! 4) Calculate Unipolar offset
  temp_offset = (temp_offset > (floor(temp_offset) + 0.5)) ? ceil(temp_offset) : floor(temp_offset);    //! 5) Round
  *LTC2449_offset_code = (int32_t)temp_offset;                                                          //! 6) Cast as int32_t
}

LTC2449::EError LTC2449::adc_code_to_value(uint32_t arg_u32_adcCode, int32_t* arg_ps32_adcValue)
{
  bool loc_b_positive = (arg_u32_adcCode >> SIGN_BIT_POS) & 1;
  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,
                     "ADC code: 0x%x",
                     arg_u32_adcCode);

  *arg_ps32_adcValue = 0;

  if( arg_u32_adcCode & (1UL << EOC_BIT_POS)) {
    return CONVERSION_ERROR;
  }

  if(loc_b_positive && (arg_u32_adcCode & (1UL << (CONVERSION_24_BITS_POS + LTC2449_RESOLUTION - 1))))
  {
    return CONVERSION_OVERRANGE;
  }
  else if(!loc_b_positive && !(arg_u32_adcCode & (1UL << (CONVERSION_24_BITS_POS + LTC2449_RESOLUTION - 1))))
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
  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,
                     "adc with sub LSB bits = %d - without LSB bits = %d",
                     *arg_ps32_adcValue,
                     (*arg_ps32_adcValue >> 5));
  return NO_ERROR;
}

void LTC2449::endOfConversion(LTC2449* arg_p_ltc2449)
{
  uint32_t loc_u32_conv_to_trash = 0;

  /** Be sure of LOW pin state */
  if(!digitalRead(LTC2449_MISO))
  {
    if(arg_p_ltc2449->_next_conv.getChannel() == INVALID_CHANNEL &&
        arg_p_ltc2449->_next_conv.getDiffPair() == INVALID_DIFF_PAIR)
    {
      /** read it and trash it - in next read _async_conv value will be available */
      arg_p_ltc2449->read(LTC2449_CS, arg_p_ltc2449->_u16_adc_cmd, arg_p_ltc2449->_async_conv, &loc_u32_conv_to_trash);
    }
    else
    {
      detachInterrupt(LTC2449_MISO);
      /** Read and process data later (in normal mode) */
      EventManager::getInstance()->queueEvent( ADC_CODE_READY_EVENT,
          (int) arg_p_ltc2449,
          EventManager::kHighPriority );
    }
  }
  else
  {
    /** Nothing to do - wait next conversion */
  }
}

void LTC2449::readLastConv(void)
{
  LTC2449::EError loc_e_ret = NO_ERROR;

  /** current read corresponds to _next_conv */
  LTC2449Conversion loc_lastConv = _next_conv;
    
  read(LTC2449_CS, _u16_adc_cmd, _async_conv, &_u32_adc_code);
  _async_conv = LTC2449Conversion();

  loc_e_ret = adc_code_to_value(_u32_adc_code, &loc_lastConv._s32_conv_value);
  if(loc_e_ret != NO_ERROR) {
    log4c_category_log(file_cat,
                       LOG4C_PRIORITY_ERROR,
                       "readLastConv - conversion error - code = %d",
                       loc_e_ret);

    if(_p_conv_listener != NULL)
    {
      _p_conv_listener->conversionError(loc_e_ret, loc_lastConv, _next_conv);
    }
  }
  else
  {
    if(_p_conv_listener != NULL)
    {
      log4c_category_log(file_cat,
                     LOG4C_PRIORITY_DEBUG,"conversionAvailable diff measure = %d - value = %d", 
                     loc_lastConv.isDiffMeasure(), 
                     loc_lastConv._s32_conv_value);
      
      _p_conv_listener->conversionAvailable(loc_lastConv, _next_conv);
    }
  }
}


void LTC2449::prepareAsyncRead(void)
{
  uint32_t u32_adc_code = 0;    // The LTC2449 code

  /** this calls takes some ms - call it before checking EOC */
  wiringPiISR(LTC2449_MISO, INT_EDGE_FALLING, (void (*)(void*))&endOfConversion, (void*) this);

  if(!digitalRead(LTC2449_MISO)) /** conversion ready */
  {
    if(_next_conv.getChannel() == INVALID_CHANNEL &&
        _next_conv.getDiffPair() == INVALID_DIFF_PAIR)
    {
      //wiringPiISR(LTC2449_MISO, INT_EDGE_FALLING, (void (*)(void*))&endOfConversion, (void*) this);
      /**  read it and trash it */
      read(LTC2449_CS, _u16_adc_cmd, _async_conv, &u32_adc_code);
      _async_conv = LTC2449Conversion();

      // Now we're ready to start capture
      EventManager::getInstance()->enableListener(ADC_CODE_READY_EVENT, this, true);
      wiringPiISRStart(LTC2449_MISO);
    }
    else /** current conversion valid - read it asynchronously */
    {
      detachInterrupt(LTC2449_MISO);
      EventManager::getInstance()->enableListener(ADC_CODE_READY_EVENT, this, true);
      /** Read and process data later */
      EventManager::getInstance()->queueEvent( ADC_CODE_READY_EVENT,
          (int)this,
          EventManager::kHighPriority );
    }
  }
  else /** conversion not ready - wait EOC low  */
  {
    // Now we're ready to start capture
    EventManager::getInstance()->enableListener(ADC_CODE_READY_EVENT, this, true);
    wiringPiISRStart(LTC2449_MISO);
  }
}

void LTC2449::processEvent(uint8_t eventCode, int eventParam)
{
  if(eventCode == ADC_CODE_READY_EVENT)
  {
    EventManager::getInstance()->enableListener(ADC_CODE_READY_EVENT, this, false);
    readLastConv();
  }
  else
  {
    /** should never be here */
    assert(false);
  }
}
