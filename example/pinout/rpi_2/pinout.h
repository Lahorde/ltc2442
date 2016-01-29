/******************************************************************************
 * @file    pinout.h
 * @author  Rémi Pincent - INRIA
 * @date    28/01/2016
 *
 * @brief   pinout on raspberry 2 - pins are set according to wiringPi
 * http://wiringpi.com/pins/
 *
 * Project : ltc2442
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * TODO_revision history
 *****************************************************************************/

#ifndef PINOUT_H_
#define PINOUT_H_

/** Set to this value for unused pins */
static const uint8_t UNUSED_PIN = 0xFF;

/*********************************
 * 4 Channel LTC2442 ADC
 *********************************/
/** Connected in 2-wire with external clock to rpi SPI threw PINS :
SCK  14
MOSI 12
MISO 13
CS on ADC2442 grounded - refer External Serial Clock, 2-Wire I/O in LTC244S datasheet
*/

/** LTC2442 CS - Arduino SPI CS digital output - NOT USED!!!*/
static const uint8_t LTC2449_CS = 10;

/** LTC2442 MISO Pin wired to pin 6 - reading EOC directly on MISO fails - */
static const uint8_t LTC2449_MISO = 6;

#endif /* PINOUT_H_ */