/*
 * EventManager.cpp
 *

 * An event handling system for Arduino.
 *
 * Author: igormt@alumni.caltech.edu
 * Copyright (c) 2013 Igor Mikolic-Torreira
 * 
 * Inspired by and adapted from the 
 * Arduino Event System library by
 * Author: mromani@ottotecnica.com
 * Copyright (c) 2010 OTTOTECNICA Italy
 *
 * This library is free software; you can redistribute it
 * and/or modify it under the terms of the GNU Lesser
 * General Public License as published by the Free Software
 * Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * This library is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser
 * General Public License along with this library; if not,
 * write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */


#include "event_manager.h"
#include <cstdio>
#include <ctime>
#include <chrono>
#include <unistd.h>


#if EVENTMANAGER_DEBUG
#define EVTMGR_DEBUG_PRINT( x )		Serial.print( x );
#define EVTMGR_DEBUG_PRINTLN( x )	Serial.println( x );
#define EVTMGR_DEBUG_PRINT_PTR( x )	Serial.print( reinterpret_cast<unsigned long>( x ), HEX );
#define EVTMGR_DEBUG_PRINTLN_PTR( x )	Serial.println( reinterpret_cast<unsigned long>( x ), HEX );
#else
#define EVTMGR_DEBUG_PRINT( x )
#define EVTMGR_DEBUG_PRINTLN( x )
#define EVTMGR_DEBUG_PRINT_PTR( x )	
#define EVTMGR_DEBUG_PRINTLN_PTR( x )	
#endif

EventManager* EventManager::_my_instance = NULL;

EventManager* EventManager::getInstance()
{
	if(_my_instance == NULL)
	{
		_my_instance = new  EventManager();
	}
	return _my_instance;
}


EventManager::EventManager() :
mHighPriorityQueue(),
mLowPriorityQueue()
{
}


int EventManager::processEvent() 
{
    uint8_t eventCode;
    int param;
    int handledCount = 0;

    if ( mHighPriorityQueue.popEvent( &eventCode, &param ) )
    {
        handledCount = mListeners.sendEvent( eventCode, param );
        
        EVTMGR_DEBUG_PRINT( "processEvent() hi-pri event " )
        EVTMGR_DEBUG_PRINT( eventCode )
        EVTMGR_DEBUG_PRINT( ", " )
        EVTMGR_DEBUG_PRINT( param )
        EVTMGR_DEBUG_PRINT( " sent to " )
        EVTMGR_DEBUG_PRINTLN( handledCount )
    }

    // If no high-pri events handled (either because there are no high-pri events or 
    // because there are no listeners for them), then try low-pri events
    if ( !handledCount && mLowPriorityQueue.popEvent( &eventCode, &param ) ) 
    {
        handledCount = mListeners.sendEvent( eventCode, param );
        
        EVTMGR_DEBUG_PRINT( "processEvent() lo-pri event " );
        EVTMGR_DEBUG_PRINT( eventCode )
        EVTMGR_DEBUG_PRINT( ", " )
        EVTMGR_DEBUG_PRINT( param )
        EVTMGR_DEBUG_PRINT( " sent to " )
        EVTMGR_DEBUG_PRINTLN( handledCount )
    }
    return handledCount;
}

/**
 * Not extremely precise
 * TODO handle microsec
 * @param ms
 */
void EventManager::applicationTick(unsigned long ms)
{
	long long  loc_u64_temp_time = 0;
	unsigned long loc_u32_curr_count = 0;
	static const uint16_t loc_u16_warn_duration = 2000;
	static const uint16_t loc_u16_wait_duration = loc_u16_warn_duration + 8000;

	if(ms == 0)
	{
		EVTMGR_DEBUG_PRINT(F("0 duration given for delay\n"));
		return;
	}

	while (loc_u32_curr_count < ms) {
		auto loc_u16_temp_sauv = std::chrono::high_resolution_clock::now();
		getInstance()->processEvent();

	    auto now   = std::chrono::high_resolution_clock::now();
		loc_u64_temp_time = std::chrono::duration_cast<std::chrono::microseconds>(now - loc_u16_temp_sauv).count();
		if(loc_u64_temp_time > loc_u16_warn_duration)
		{
			EVTMGR_DEBUG_PRINT(F("long processing %uµs\n"));
			EVTMGR_DEBUG_PRINTLN(loc_u16_temp_time);
		}

		loc_u64_temp_time = std::chrono::duration_cast<std::chrono::microseconds>(now - loc_u16_temp_sauv).count();

		/** send tick event */
	    if(getInstance()->mListeners.hasActiveListeners(EVENT_TICK))
	    {
	    	/** some listeners listen on event tick - continue notification */
	    	getInstance()->mListeners.sendEvent( EVENT_TICK, 0 );
	    }

		if(loc_u64_temp_time > loc_u16_wait_duration)
		{
			/** Continue iteration without waiting */
			loc_u32_curr_count += loc_u64_temp_time/1000;
		}
		else
		{
			/** TODO for now delay - to be changed - decisive for low power modes */
			usleep(loc_u16_wait_duration - loc_u64_temp_time);
			loc_u32_curr_count += loc_u16_wait_duration/1000;
		}
	}
}

int EventManager::processAllEvents() 
{
    uint8_t eventCode;
    int param;
    int handledCount = 0;

    while ( mHighPriorityQueue.popEvent( &eventCode, &param ) )
    {
        handledCount += mListeners.sendEvent( eventCode, param );
        
        EVTMGR_DEBUG_PRINT( "processEvent() hi-pri event " )
        EVTMGR_DEBUG_PRINT( eventCode )
        EVTMGR_DEBUG_PRINT( ", " )
        EVTMGR_DEBUG_PRINT( param )
        EVTMGR_DEBUG_PRINT( " sent to " )
        EVTMGR_DEBUG_PRINTLN( handledCount )
    }
    
    while ( mLowPriorityQueue.popEvent( &eventCode, &param ) ) 
    {
        handledCount += mListeners.sendEvent( eventCode, param );
        
        EVTMGR_DEBUG_PRINT( "processEvent() lo-pri event " )
        EVTMGR_DEBUG_PRINT( eventCode )
        EVTMGR_DEBUG_PRINT( ", " )
        EVTMGR_DEBUG_PRINT( param )
        EVTMGR_DEBUG_PRINT( " sent to " )
        EVTMGR_DEBUG_PRINTLN( handledCount )
    }
    return handledCount;
}



/********************************************************************/



EventManager::ListenerList::ListenerList() : 
mNumListeners( 0 )
{
}


bool EventManager::ListenerList::addListener( uint8_t eventCode, EventListener* const listener )
{
    EVTMGR_DEBUG_PRINT( "addListener() enter " )
    EVTMGR_DEBUG_PRINT( eventCode )
    EVTMGR_DEBUG_PRINT( ", " )
    EVTMGR_DEBUG_PRINTLN_PTR( listener )

    // Argument check
    if ( !listener ) 
    {
    	EVTMGR_DEBUG_PRINTLN(F("invalid listener given"));
        return false;
    }

    // Check for full dispatch table
    if ( isFull() ) 
    {
        EVTMGR_DEBUG_PRINTLN( "addListener() list full" )
        return false;
    }

    mListeners[ mNumListeners ].listener = listener;
    mListeners[ mNumListeners ].eventCode = eventCode;
    mListeners[ mNumListeners ].enabled 	= true;
    mNumListeners++;

    EVTMGR_DEBUG_PRINTLN( "addListener() listener added" )
    
    return true;
}


bool EventManager::ListenerList::removeListener( uint8_t eventCode, EventListener* const listener )
{
    EVTMGR_DEBUG_PRINT( "removeListener() enter " )
    EVTMGR_DEBUG_PRINT( eventCode )
    EVTMGR_DEBUG_PRINT( ", " )
    EVTMGR_DEBUG_PRINTLN_PTR( listener )
    
    if ( mNumListeners == 0 ) 
    {
        EVTMGR_DEBUG_PRINTLN( "removeListener() no listeners" )
        return false;
    }

    int k = searchListeners( eventCode, listener );
    if ( k < 0 )
    {
        EVTMGR_DEBUG_PRINTLN( "removeListener() not found" )
        return false;
    }

    for ( int i = k; i < mNumListeners - 1; i++ ) 
    {
        mListeners[ i ].listener  = mListeners[ i + 1 ].listener;
        mListeners[ i ].eventCode = mListeners[ i + 1 ].eventCode;
        mListeners[ i ].enabled   = mListeners[ i + 1 ].enabled;
        mNumListeners--;
    }
     
    EVTMGR_DEBUG_PRINTLN( "removeListener() removed" )
    
    return true;
}


int EventManager::ListenerList::removeListener( EventListener* const listener )
{
    EVTMGR_DEBUG_PRINT( "removeListener() enter " )
    EVTMGR_DEBUG_PRINTLN_PTR( listener )

    if ( mNumListeners == 0 ) 
    {
        EVTMGR_DEBUG_PRINTLN( "removeListener() no listeners" )
        return 0;
    }
    
    int removed = 0;
    while ( int k = searchListeners( listener ) >= 0 )
    {
        for ( int i = k; i < mNumListeners - 1; i++ ) 
        {
            mListeners[ i ].listener  = mListeners[ i + 1 ].listener;
            mListeners[ i ].eventCode = mListeners[ i + 1 ].eventCode;
            mListeners[ i ].enabled   = mListeners[ i + 1 ].enabled;
            mNumListeners--;
            removed++;
        }
    }
    
    EVTMGR_DEBUG_PRINT( "removeListener() removed " )
    EVTMGR_DEBUG_PRINTLN( removed )
    
    return removed;
}


bool EventManager::ListenerList::enableListener( uint8_t eventCode, EventListener* const listener, bool enable )
{
    EVTMGR_DEBUG_PRINT( "enableListener() enter " )
    EVTMGR_DEBUG_PRINT( eventCode )
    EVTMGR_DEBUG_PRINT( ", " )
    EVTMGR_DEBUG_PRINT_PTR( listener )
    EVTMGR_DEBUG_PRINT( ", " )
    EVTMGR_DEBUG_PRINTLN( enable )

    if ( mNumListeners == 0 ) 
    {
        EVTMGR_DEBUG_PRINTLN( "enableListener() no listeners" )
        return false;
    }

    int k = searchListeners( eventCode, listener );
    if ( k < 0 ) 
    {
        EVTMGR_DEBUG_PRINTLN( "enableListener() not found fail" )
        return false;
    }

    mListeners[ k ].enabled = enable;

    EVTMGR_DEBUG_PRINTLN( "enableListener() success" )
    return true;
}


bool EventManager::ListenerList::isListenerEnabled( uint8_t eventCode, EventListener* const listener )
{
    if ( mNumListeners == 0 ) 
    {
        return false;
    }

    int k = searchListeners( eventCode, listener );
    if ( k < 0 ) 
    {
        return false;
    }

    return mListeners[ k ].enabled;
}


int EventManager::ListenerList::sendEvent( uint8_t eventCode, int param )
{
    EVTMGR_DEBUG_PRINT( "sendEvent() enter " )
    EVTMGR_DEBUG_PRINT( eventCode )
    EVTMGR_DEBUG_PRINT( ", " )
    EVTMGR_DEBUG_PRINTLN( param )

    int handlerCount = 0;
    for ( int i = 0; i < mNumListeners; i++ ) 
    {
        if ( ( mListeners[ i ].listener != 0 ) && ( mListeners[ i ].eventCode == eventCode ) && mListeners[ i ].enabled ) 
        {
            handlerCount++;
            (*mListeners[ i ].listener).processEvent(eventCode, param);
        }
    }
    
    EVTMGR_DEBUG_PRINT( "sendEvent() sent to " )
    EVTMGR_DEBUG_PRINTLN( handlerCount )

    if ( !handlerCount ) 
    {
    	EVTMGR_DEBUG_PRINTLN( "NO handler registered => event not used" )
    }
    
    return handlerCount;
}

bool EventManager::ListenerList::hasActiveListeners( uint8_t eventCode )
{
    for ( int i = 0; i < mNumListeners; i++ )
    {
        if ( mListeners[i].eventCode == eventCode && mListeners[i].enabled)
        {
            return true;
        }
    }

    return false;
}

int EventManager::ListenerList::searchListeners( uint8_t eventCode, EventListener* const listener )
{
    for ( int i = 0; i < mNumListeners; i++ ) 
    {
        if ( ( mListeners[i].eventCode == eventCode ) && ( mListeners[i].listener == listener ) ) 
        {
            return i;
        }
    }

    return -1;
}


int EventManager::ListenerList::searchListeners( EventListener* const listener )
{
    for ( int i = 0; i < mNumListeners; i++ ) 
    {
        if ( mListeners[i].listener == listener ) 
        {
            return i;
        }
    }

    return -1;
}


int EventManager::ListenerList::searchEventCode( uint8_t eventCode )
{
    for ( int i = 0; i < mNumListeners; i++ ) 
    {
        if ( mListeners[i].eventCode == eventCode ) 
        {
            return i;
        }
    }

    return -1;
}



/******************************************************************************/




EventManager::EventQueue::EventQueue( void ) :
mEventQueueHead( 0 ), 
mEventQueueTail( 0 ), 
mNumEvents( 0 )
{
    for ( int i = 0; i < kEventQueueSize; i++ ) 
    {
        mEventQueue[i].code = EventManager::kEventNone;
        mEventQueue[i].param = 0;
    }
}



bool EventManager::EventQueue::queueEvent( uint8_t eventCode, int eventParam )
{  
    /*
    * The call to noInterrupts() MUST come BEFORE the full queue check.  
    * 
    * If the call to isFull() returns FALSE but an asynchronous interrupt queues
    * an event, making the queue full, before we finish inserting here, we will then
    * corrupt the queue (we'll add an event to an already full queue). So the entire 
    * operation, from the call to isFull() to completing the inserting (if not full) 
    * must be atomic.
    * 
    * Note that this race condition can only arise IF both interrupt and non-interrupt (normal)
    * code add events to the queue.  If only normal code adds events, this can't happen
    * because then there are no asynchronous additions to the queue.  If only interrupt
    * handlers add events to the queue, this can't happen because further interrupts are 
    * blocked while an interrupt handler is executing.  This race condition can only happen
    * when an event is added to the queue by normal (non-interrupt) code and simultaneously 
    * an interrupt handler tries to add an event to the queue.  This is the case that the 
    * cli() (= noInterrupts()) call protects against.
    * 
    * Contrast this with the logic in popEvent().
    * 
    */

    // Because this function may be called from interrupt handlers, debugging is 
    // only available in when NOT in interrupt safe mode.
    
#if EVENTMANAGER_DEBUG
    if ( !mInterruptSafeMode )
    {
        EVTMGR_DEBUG_PRINT( "queueEvent() enter " )
        EVTMGR_DEBUG_PRINT( eventCode )
        EVTMGR_DEBUG_PRINT( ", " )
        EVTMGR_DEBUG_PRINTLN( eventParam )
    }
#endif

    bool retVal = false;
    if(!getInstance()->mListeners.hasActiveListeners(eventCode))
    {
    	/** Do not push an event without any listeners */
    	return retVal;
    }

    // ATOMIC BLOCK BEGIN (only atomic **if** mInterruptSafeMode is on)
    if ( !isFull() ) 
    {
        // Store the event at the tail of the queue
        mEventQueue[ mEventQueueTail ].code = eventCode;
        mEventQueue[ mEventQueueTail ].param = eventParam;

        // Update queue tail value
        mEventQueueTail = ( mEventQueueTail + 1 ) % kEventQueueSize;;

        // Update number of events in queue
        mNumEvents++;

        retVal = true;
    }
    else
    {

    }


#if EVENTMANAGER_DEBUG
    if ( !mInterruptSafeMode )
    {
        EVTMGR_DEBUG_PRINT( "queueEvent() " )
        EVTMGR_DEBUG_PRINTLN( retVal ? "added" : "full" )
    }
#endif

    return retVal;
}


bool EventManager::EventQueue::popEvent( uint8_t* eventCode, int* eventParam )
{
    /*
    * The call to noInterrupts() MUST come AFTER the empty queue check.  
    * 
    * There is no harm if the isEmpty() call returns an "incorrect" TRUE response because 
    * an asynchronous interrupt queued an event after isEmpty() was called but before the 
    * return is executed.  We'll pick up that asynchronously queued event the next time 
    * popEvent() is called. 
    * 
    * If noInterrupts() is set before the isEmpty() check, we pretty much lock-up the Arduino.  
    * This is because popEvent(), via processEvents(), is normally called inside loop(), which
    * means it is called VERY OFTEN.  Most of the time (>99%), the event queue will be empty.
    * But that means that we'll have interrupts turned off for a significant fraction of the 
    * time.  We don't want to do that.  We only want interrupts turned off when we are 
    * actually manipulating the queue.
    * 
    * Contrast this with the logic in queueEvent().
    * 
    */

    if ( isEmpty() ) 
    {
        return false;
    }

    // Pop the event from the head of the queue
    // Store event code and event parameter into the user-supplied variables
    *eventCode  = mEventQueue[ mEventQueueHead ].code;
    *eventParam = mEventQueue[ mEventQueueHead ].param;

    // Clear the event (paranoia)
    mEventQueue[ mEventQueueHead ].code = EventManager::kEventNone;
    
    // Update the queue head value
    mEventQueueHead = ( mEventQueueHead + 1 ) % kEventQueueSize;

    // Update number of events in queue
    mNumEvents--;


    EVTMGR_DEBUG_PRINT( "popEvent() return " )
    EVTMGR_DEBUG_PRINT( *eventCode )
    EVTMGR_DEBUG_PRINT( ", " )
    EVTMGR_DEBUG_PRINTLN( *eventParam )

    return true;
}
