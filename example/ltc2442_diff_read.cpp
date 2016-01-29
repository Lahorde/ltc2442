/******************************************************************************
 * @file    ltc2442_diff_read.cpp
 * @author  Rémi Pincent - INRIA
 * @date    28/01/2016
 *
 * @brief Test differential reads on ltc2442 adc
 *
 * Project : ltc2442
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 ******************************************************************************/
#include <stdio.h>
#include <LTC2449.h>
#include <log4c.h>
#include <wiringPi.h>

class ADCListener : public LTC2449::ILTC2449Listener
{
  /** from ILTC2449Listener */
  void conversionAvailable(LTC2449::LTC2449Conversion& arg_ltc2449_conversion, LTC2449::LTC2449Conversion& arg_ltc2449_nextConversion);

  /** from ILTC2449Listener */
  void conversionError(LTC2449::EError& loc_e_error, LTC2449::LTC2449Conversion& arg_ltc2449_badConversion, LTC2449::LTC2449Conversion& arg_ltc2449_nextConversion);
};

/** LOGGER */
static const char* FILE_NAME_LOGGER = "LTC2449_example";
static const log4c_category_t* file_cat;

static ADCListener adc_listener;
static  LTC2449 ltc2449(&adc_listener);

static int32_t s32_conv0Value = 0;
static int32_t s32_conv1Value = 0;

int main(void) {
  /** initialize logger */
  if (log4c_init())
  {
    printf("log4c_init() failed");
    return 1;
  }
  file_cat = log4c_category_get(FILE_NAME_LOGGER);
  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_INFO,
                     "ltc244_diff_read example");

  wiringPiSetup () ;

  if(ltc2449.read_differential(LTC2449::DIFF_2P_3N) != LTC2449::NO_ERROR)
  {
    return 1;
  }
  while(1)
  {
    EventManager::getInstance()->applicationTick(10);
  }

  return 0;
}

void ADCListener::conversionAvailable(LTC2449::LTC2449Conversion& arg_ltc2449_conversion, LTC2449::LTC2449Conversion& arg_ltc2449_nextConversion)
{
  static int count = 0;

  if(arg_ltc2449_conversion.getDiffPair() == LTC2449::DIFF_2P_3N)
  {
    log4c_category_log(file_cat,
                       LOG4C_PRIORITY_INFO,
                       "Conversion available on diff pair %d - ADC value = %d - diff from previous = %d",
                       arg_ltc2449_conversion.getDiffPair(),
                       arg_ltc2449_conversion._s32_conv_value,
                       arg_ltc2449_conversion._s32_conv_value - s32_conv1Value);
    s32_conv1Value = arg_ltc2449_conversion._s32_conv_value;
  }
  else
  {
    log4c_category_log(file_cat,
                       LOG4C_PRIORITY_INFO,
                       "Conversion available on diff pair %d - ADC value = %d - diff from previous = %d",
                       arg_ltc2449_conversion.getDiffPair(),
                       arg_ltc2449_conversion._s32_conv_value,
                       arg_ltc2449_conversion._s32_conv_value - s32_conv0Value);
    s32_conv0Value = arg_ltc2449_conversion._s32_conv_value;
  }

  /** handle next conv */
  if(count < 30)
  {
    /** keep converting on same differential channels */
    if(arg_ltc2449_nextConversion.getDiffPair() == LTC2449::DIFF_2P_3N)
      ltc2449.read_differential(LTC2449::DIFF_2P_3N);
    else
      ltc2449.read_differential(LTC2449::DIFF_0P_1N);
  }
  else
  {
    /** change channel */
    count = 0;
    if(arg_ltc2449_nextConversion.getDiffPair() == LTC2449::DIFF_2P_3N)
      ltc2449.read_differential(LTC2449::DIFF_0P_1N);
    else
      ltc2449.read_differential(LTC2449::DIFF_2P_3N);
  }

  count++;
}

void ADCListener::conversionError(LTC2449::EError& loc_e_error, LTC2449::LTC2449Conversion& arg_ltc2449_badConversion, LTC2449::LTC2449Conversion& arg_ltc2449_nextConversion)
{
  log4c_category_log(file_cat,
                     LOG4C_PRIORITY_ERROR,
                     "Conversion error on diff pair %d ",
                     arg_ltc2449_badConversion.getDiffPair());
}