/******************************************************************************
 * @file    EventListener.h 
 * @author  Rémi Pincent - INRIA
 * @date    16 juin 2014   
 *
 * @brief Event manager listener interface description
 * 
 * Project : event_manager
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 * 
 * Revision History:
 * TODO_revision history
 *****************************************************************************/

#ifndef EVENTLISTENER_H_
#define EVENTLISTENER_H_

#include <stdint.h>

/**
 * Event listener interface
 */
class EventListener
{
public:
	virtual void processEvent(uint8_t eventCode, int eventParam) = 0;
	virtual ~EventListener(){};
};



#endif /* EVENTLISTENER_H_ */
