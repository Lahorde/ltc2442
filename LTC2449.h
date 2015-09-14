/******************************************************************************
 * @file    LTC2449.h
 * @author  Rémi Pincent - INRIA
 * @date    15 avr. 2015
 *
 * @brief Driver for LTC2449: 24-Bit, 16-Channel Delta Sigma ADCs with Selectable Speed/Resolution.
 * Initial version from : http://www.linear.com/product/LTC2442#code
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


SPI DATA FORMAT (MSB First):

            Byte #1                            Byte #2

Data Out :  !EOC DMY SIG D28 D27 D26 D25 D24   D23  D22  D21  D20  D19 D18 D17 D16
Data In  :  1    0   EN  SGL OS  S2  S1  S0    OSR3 OSR2 OSR1 OSR1 SPD X   X   X

Byte #3                        Byte #4
D15 D14 D13 D12 D11 D10 D9 D8  D7 D6 D5 D4 *D3 *D2 *D1 *D0
X   X   X   X   X   X   X  X   X  X  X  X   X  X   X   X

!EOC : End of Conversion Bit (Active Low)
DMY  : Dummy Bit (Always 0)
SIG  : Sign Bit (1-data positive, 0-data negative)
Dx   : Data Bits
*Dx  : Data Bits Below lsb
EN   : Enable Bit (0-keep previous mode, 1-change mode)
SGL  : Enable Single-Ended Bit (0-differential, 1-single-ended)
OS   : ODD/Sign Bit
Sx   : Address Select Bit
0SRX : Over Sampling Rate Bits
SPD  : Double Output Rate Select Bit (0-Normal rate, auto-calibration on, 2x rate, auto_calibration off)

Command Byte #1
1    0    EN   SGL  OS   S2   S1   S0   Comments
1    0    0    X    X    X    X    X    Keep Previous Mode
1    0    1    0    X    X    X    X    Differential Mode
1    0    1    1    X    X    X    X    Single-Ended Mode

|  Coversion Rate    |  RMS  | ENOB | OSR  | Latency
Command Byte #2          |Internal | External | Noise |      |      |
|  9MHz   | 10.24MHz |       |      |      |
OSR3 OSR2 OSR1 OSR1 SPD  | Clock   |  Clock   |       |      |      |
0    0    0    0    0      Keep Previous Speed/Resolution
0    0    0    1    0      3.52kHz   4kHz       23uV    17     64     none
0    0    1    0    0      1.76kHz   2kHz       3.5uV   20.1   128    none
0    0    1    1    0      880Hz     1kHz       2uV     21.3   256    none
0    1    0    0    0      440Hz     500Hz      1.4uV   21.8   512    none
0    1    0    1    0      220Hz     250Hz      1uV     22.4   1024   none
0    1    1    0    0      110Hz     125Hz      750nV   22.9   2048   none
0    1    1    1    0      55Hz      62.5Hz     510nV   23.4   4096   none
1    0    0    0    0      27.5Hz    31.25Hz    375nV   24     8192   none
1    0    0    1    0      13.75Hz   15.625Hz   250nV   24.4   16384  none
1    1    1    1    0      6.87kHz   7.8125Hz   200nV   24.6   32768  none
0    0    0    0    1      Keep Previous Speed/Resolution
OSR3 OSR2 OSR1 OSR1 1      2X Mode  *all clock speeds double

Example Code:

Read Channel 0 in Single-Ended with OSR of 65536

    uint16_t miso_timeout = 1000;
    adc_command = LTC2449_CH0 | LTC2449_OSR_32768 | LTC2449_SPEED_2X;   // Build ADC command for channel 0
                                                                        // OSR = 32768*2 = 65536

    if(LTC2449_EOC_timeout(LTC2449_CS, miso_timeout))    // Check for EOC
        return;                                          // Exit if timeout is reached
    LTC2449_read(LTC2449_CS, adc_command, &adc_code);    // Throws out last reading
    
    if(LTC2449_EOC_timeout(LTC2449_CS, miso_timeout))    // Check for EOC
        return;                                          // Exit if timeout is reached
    LTC2449_read(LTC2449_CS, adc_command, &adc_code);    // Obtains the current reading and stores to adc_code variable

    // Convert adc_code to voltage
    adc_voltage = LTC2449_code_to_voltage(adc_code, LTC2449_lsb, LTC2449_offset_code);

@endverbatim
*/

#ifndef LTC2449_H
#define LTC2449_H

#include <event_manager.h>
#include <logger.h>

#define LTC2449_RESOLUTION          24
#define LTC2449_SUB_LSB_RESOLUTION  5

/*! @name Mode Configuration
 @{
*/
#define LTC2449_KEEP_PREVIOUS_MODE              0x8000
#define LTC2449_KEEP_PREVIOUS_SPEED_RESOLUTION  0x0000
#define LTC2449_SPEED_1X                        0x0000
#define LTC2449_SPEED_2X                        0x0008
/*!
 @}
*/

/*! @name Single-Ended Channels Configuration
@{ */
#define LTC2449_CH0            0xB000
#define LTC2449_CH1            0xB800
#define LTC2449_CH2            0xB100
#define LTC2449_CH3            0xB900
#define LTC2449_CH4            0xB200
#define LTC2449_CH5            0xBA00
#define LTC2449_CH6            0xB300
#define LTC2449_CH7            0xBB00
#define LTC2449_CH8            0xB400
#define LTC2449_CH9            0xBC00
#define LTC2449_CH10           0xB500
#define LTC2449_CH11           0xBD00
#define LTC2449_CH12           0xB600
#define LTC2449_CH13           0xBE00
#define LTC2449_CH14           0xB700
#define LTC2449_CH15           0xBF00
/*! @} */

/*! @name Differential Channel Configuration
@{ */
#define LTC2449_P0_N1          0xA000
#define LTC2449_P1_N0          0xA800

#define LTC2449_P2_N3          0xA100
#define LTC2449_P3_N2          0xA900

#define LTC2449_P4_N5          0xA200
#define LTC2449_P5_N4          0xAA00

#define LTC2449_P6_N7          0xA300
#define LTC2449_P7_N6          0xAB00

#define LTC2449_P8_N9          0xA400
#define LTC2449_P9_N8          0xAC00

#define LTC2449_P10_N11        0xA500
#define LTC2449_P11_N10        0xAD00

#define LTC2449_P12_N13        0xA600
#define LTC2449_P13_N12        0xAE00

#define LTC2449_P14_N15        0xA700
#define LTC2449_P15_N14        0xAF00
/*! @} */

/*Commands
Construct a channel / resolution control word by bitwise ORing one choice from the channel configuration
and one choice from the Oversample ratio configuration. You can also enable 2Xmode, which will increase
sample rate by a factor of 2 but introduce one cycle of latency.

Example - read channel 3 single-ended at OSR2048, with 2X mode enabled.
adc_command = (LTC2449_CH3 | LTC2449_OSR_2048) | LTC2449_SPEED_2X;
*/

/*! @name Oversample Ratio (OSR) Commands
@{ */
#define LTC2449_OSR_64         0xA010
#define LTC2449_OSR_128        0xA020
#define LTC2449_OSR_256        0xA030
#define LTC2449_OSR_512        0xA040
#define LTC2449_OSR_1024       0xA050
#define LTC2449_OSR_2048       0xA060
#define LTC2449_OSR_4096       0xA070
#define LTC2449_OSR_8192       0xA080
#define LTC2449_OSR_16384      0xA090
#define LTC2449_OSR_32768      0xA0F0
/*! @}*/

/** out data frame */
#define EOC_BIT_POS                   31
#define DUMMY_BIT_POS                 30
#define SIGN_BIT_POS                  29
#define CONVERSION_24_BITS_POS        5
#define CONVERSION_5_BITS_SUB_LSB_POS 0

class LTC2449 : public EventManager, public EventListener
{
/** types */
public:
    	typedef enum{
    		NO_ERROR = 0,
    		CONVERSION_ERROR = -1,
    		CONVERSION_UNDERRANGE = -2,
    		CONVERSION_OVERRANGE = -3,
    		CONVERSION_TIME_OUT = -4,
    		BAD_ADC_CHANNEL = -5,
		CONVERSION_ON_GOING = -6,
    	}EError;

    	typedef enum{
    		DIFF_0P_1N = 0,
    		DIFF_2P_3N = 1,
    		DIFF_1P_0N = 8,
    		DIFF_3P_2N = 9,
			INVALID_DIFF_PAIR = 0xFF,
    	}EDiffPair;

	class LTC2449Conversion{
	private :
		uint8_t _u8_channel;
		EDiffPair _e_diff_pair;

	public :
		int32_t _s32_conv_value;

		LTC2449Conversion(void):
				_u8_channel(INVALID_CHANNEL),
				_e_diff_pair(INVALID_DIFF_PAIR),
				_s32_conv_value(0)
		{

		}

		inline bool isDiffMeasure(void)
		{
			return (_e_diff_pair != INVALID_DIFF_PAIR) && (_u8_channel == INVALID_CHANNEL);
		}

		inline void setChannel(uint8_t _arg_u8_channel)
		{
			_e_diff_pair = INVALID_DIFF_PAIR;
			_u8_channel = _arg_u8_channel;
		}

		inline void setDiffPair(EDiffPair arg_e_diff_pair)
		{
			_u8_channel = INVALID_CHANNEL;
			_e_diff_pair = arg_e_diff_pair;
		}

		inline EDiffPair getDiffPair(void)
		{
			return _e_diff_pair;
		}

		inline uint8_t getChannel(void)
		{
			return _u8_channel;
		}
	};

	class ILTC2449Listener
	{
	public:
		virtual ~ILTC2449Listener(){};
		virtual void conversionAvailable(LTC2449Conversion& arg_ltc2449_conversion) = 0;
	};

/** members */
public:
    static const uint8_t INVALID_CHANNEL   = 0xFF;
    static const int32_t OUT_OF_RANGE_ADC_VALUE = 1UL << (LTC2449_RESOLUTION + CONVERSION_24_BITS_POS) ;

private:
	uint16_t _u16_adc_cmd;
	uint32_t _u32_adc_code;
	ILTC2449Listener* _p_conv_listener;
	LTC2449Conversion _last_conv;
	bool _b_trash_next_conv;

public:
	LTC2449(ILTC2449Listener* arg_p_conv_listener = NULL);
	~LTC2449();

	//! Synchronous read channels in single-ended mode
	EError read_single_ended(uint8_t arg_u8_channel, int32_t* arg_s32_adc_value);

	//! Asynchronous read channels in single-ended mode - listener will be notified with conversion value
	EError read_single_ended(uint8_t arg_u8_channel);

	//! Synchronous Read channels in differential mode
	EError read_differential(EDiffPair arg_e_diffPair, int32_t* arg_s32_adc_value);

	//! Asynchronous Read channels in differential mode - listener will be notified with conversion value
	EError read_differential(EDiffPair arg_e_diffPair);

	/**
	 * A single listener can be registered
	 * @param arg_p_conv_listener
	 */
	inline void register_listener(ILTC2449Listener* arg_p_conv_listener){
		ASSERT(arg_p_conv_listener != NULL);
		ASSERT(_p_conv_listener == NULL);
		_p_conv_listener = arg_p_conv_listener;
	}

	/**
	 * @param arg_p_conv_listener
	 */
	inline void unregister_listener(ILTC2449Listener* arg_p_conv_listener){
		ASSERT(arg_p_conv_listener != NULL);
		ASSERT(_p_conv_listener == arg_p_conv_listener);
		/** No synchronization need : listener called in normal context */
		_p_conv_listener = NULL;
	}

private:

	//! Checks for EOC with a specified timeout
	//! @return Returns 0=successful, 1=unsuccessful (exceeded timeout)
	static int8_t EOC_timeout(uint8_t cs,           //!< Chip Select pin
	                          uint16_t miso_timeout  //!< Timeout (in milliseconds)
	                         );

	//! Reads from LTC2449.
	//! @return  void
	static void read(uint8_t cs,               //!< Chip Select pin
	                  uint16_t adc_command,     //!< 2 byte command written to LTC2449
	                  uint32_t *adc_code        //!< 4 byte conversion code read from LTC2449
	                 );

	//! Calculates the voltage corresponding to an adc code, given lsb weight (in volts) and the calibrated
	//! ADC offset code (zero code that is subtracted from adc_code).
	//! @return Returns voltage calculated from ADC code.
	static float code_to_voltage(int32_t adc_code,             //!< Code read from adc
	                              float LTC2449_lsb,            //!< LSB weight (in volts)
	                              int32_t LTC2449_offset_code   //!< The calibrated offset code (This is the adc code zero code that will be subtraced from adc_code)
	                             );

	//! Calculate the lsb weight and offset code given a full-scale code and a measured zero-code.
	//! @return Void
	static void cal_voltage(int32_t zero_code,             //!< Measured code with the inputs shorted to ground
	                         int32_t fs_code,               //!< Measured code at nearly full-scale
	                         float zero_voltage,            //!< Measured zero voltage
	                         float fs_voltage,              //!< Voltage measured at input (with voltmeter) when fs_code was read from adc
	                         float *LTC2449_lsb,            //!< Overwritten with lsb weight (in volts)
	                         int32_t *LTC2449_offset_code   //!< Overwritten with offset code (zero code)
	                        );

	/**
	 * Converts given adc code to an adc value
	 * @param arg_u32_adcCode
	 * @param arg_ps32_adcValue
	 * @return
	 */
	static EError adc_code_to_value(uint32_t arg_u32_adcCode, int32_t* arg_ps32_adcValue);

	/**
     * Called when a conversion is ready to be read
	 */
	static void endOfConversion(LTC2449* arg_p_ltc2449);

	/** EventListener */
	void processEvent(uint8_t eventCode, int eventParam);

};

#endif  // LTC2449_H
