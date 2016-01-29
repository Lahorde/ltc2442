/******************************************************************************
 * @file    events.h 
 * @author  Rémi Pincent - INRIA
 * @date    28/01/2016   
 *
 * @brief TODO_one_line_description_of_file
 * 
 * Project : ltc2442
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 * 
 * Revision History:
 * TODO_revision history
 *****************************************************************************/

#ifndef EVENTS_H_
#define EVENTS_H_

#include <stdint.h>
#include <limits.h>

typedef enum __attribute__ ((__packed__)) {
/************************************************
 * Sensor events
 ************************************************/
 
  /** Emitted by LTC2442 when adc conversion code can be read */
  ADC_CODE_READY_EVENT,
  /** Emitted by LTC2442 proxy when a data is ready to be read - given parameter is diff measure id */
  ADC_VALUE_CHANGED_EVENT,

/************************************************
 * INTERACTION events
 ************************************************/

/************************************************
 * PERCEPTION events
 ************************************************/

/************************************************
 * State events
 ************************************************/

  MAX_EVENTS = UCHAR_MAX,
  OUT_OF_ENUM_EVENT = MAX_EVENTS,
}EEvents;
#endif /* EVENTS_H_ */
