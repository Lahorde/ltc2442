/*
 * EventManager.h
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


#ifndef EventManager_h
#define EventManager_h

#include <cstdint>
#include "EventListener.h"

// Size of the listener list.  Adjust as appropriate for your application.
// Requires a total of sizeof(*f())+sizeof(int)+sizeof(bool) bytes of RAM for each unit of size
#ifndef EVENTMANAGER_LISTENER_LIST_SIZE
#define EVENTMANAGER_LISTENER_LIST_SIZE		8
#endif

// Size of the event two queues.  Adjust as appropriate for your application.
// Requires a total of 4 * sizeof(int) bytes of RAM for each unit of size 
#ifndef EVENTMANAGER_EVENT_QUEUE_SIZE
#define EVENTMANAGER_EVENT_QUEUE_SIZE		8
#endif

class EventManager 
{

public:
    
    // EventManager can be instantiated in either as interrupt safe or 
    // non-interupt safe configuration.  The default is interrupt safe, but 
    // these constants can be used to explicitly set the configuration.
    // If you queue events from an interrupt handler, you must instantiate  
    // the EventManager in interrupt safe mode.
    enum SafetyMode { kNotInterruptSafe, kInterruptSafe };

    // EventManager recognizes two kinds of events.  By default, events are
    // are queued as low priority, but these constants can be used to explicitly
    // set the priority when queueing events
    //
    // NOTE high priority events are always handled before any low priority events.
    enum EventPriority { kHighPriority, kLowPriority };
    
    // Various pre-defined event type codes.  These are completely optional and 
    // provided for convenience.  Any integer value can be used as an event code.
    enum EventType 
    {
        // No event occurred; param: none
        kEventNone = 200,

        // A key was pressed;  param: key code
        kEventKeyPress,

        // A key was released;  param: key code
        kEventKeyRelease,

        // Use this to notify a character;  param: the character to be notified
        kEventChar,

        // Generic time event
        // param: a time value (exact meaning is defined by the code inserting this event into the queue)
        kEventTime,

        // Generic timer events; param: same as EV_TIME
        kEventTimer0,
        kEventTimer1,
        kEventTimer2,
        kEventTimer3,

        // Analog read (last number = analog channel);  param: value read
        kEventAnalog0,
        kEventAnalog1,
        kEventAnalog2,
        kEventAnalog3,
        kEventAnalog4,
        kEventAnalog5,

        // Menu events
        kEventMenu0,
        kEventMenu1,
        kEventMenu2,
        kEventMenu3,
        kEventMenu4,
        kEventMenu5,
        kEventMenu6,
        kEventMenu7,
        kEventMenu8,
        kEventMenu9,

        // Serial event, example: a new char is available
        // param: the return value of Serial.read()
        kEventSerial,

        // LCD screen needs to be refreshed
        kEventPaint,

        // User events
        kEventUser0,
        kEventUser1,
        kEventUser2,
        kEventUser3,
        kEventUser4,
        kEventUser5,
        kEventUser6,
        kEventUser7,
        kEventUser8,
        kEventUser9
    };

    /**
     * Event manager has a special event called EVENT_TICK
     * Listeners registered to this event will be notified each
     * time an event manager tick occurs.
     * This event must have been defined in your applications
     * events.h
     */
    static const uint8_t EVENT_TICK = -1;

    //Singleton design pattern
    // Return EventManagerInstance, it operates in interrupt safe mode
    static EventManager* getInstance();
    
    // Create an event manager
    EventManager(void);

    // Add a listener
    // Returns true if the listener is successfully installed, false otherwise (e.g. the dispatch table is full)
    bool addListener( uint8_t eventCode, EventListener*  const listener );

    // Remove (event, listener) pair (all occurrences)
    // Other listeners with the same function or event code will not be affected
    bool removeListener( uint8_t eventCode, EventListener* const listener );
 
    // Remove all occurrances of a listener
    // Removes this listener regardless of the event code; returns number removed
    // Useful when one listener handles many different events
    int removeListener( EventListener* listener );

    // Enable or disable a listener
    // Return true if the listener was successfully enabled or disabled, false if the listener was not found
    bool enableListener( uint8_t eventCode, EventListener* const listener, bool enable );

    // Returns the current enabled/disabled state of the (eventCode, listener) combo
    bool isListenerEnabled( uint8_t eventCode, EventListener* const listener );

    // The default listener is a callback function that is called when an event with no listener is processed
    // These functions set, clear, and enable/disable the default listener
    bool setDefaultListener( EventListener* const listener );
    void removeDefaultListener();
    void enableDefaultListener( bool enable );
    
    // Is the ListenerList empty?
    bool isListenerListEmpty();
    
    // Is the ListenerList full?
    bool isListenerListFull();
    
    // Returns true if no events are in the queue
    bool isEventQueueEmpty( EventPriority pri = kLowPriority );

    // Returns true if no more events can be inserted into the queue
    bool isEventQueueFull( EventPriority pri = kLowPriority );

    // Actual number of events in queue
    int getNumEventsInQueue( EventPriority pri = kLowPriority );

    // tries to insert an event into the queue;
    // returns true if successful, false if the
    // queue if full and the event cannot be inserted
    bool queueEvent( uint8_t eventCode, int eventParam, EventPriority pri = kLowPriority );

    // this must be called regularly (usually by calling it inside the loop() function)
    int processEvent();

    // this function can be called to process ALL events in the queue
    // WARNING:  if interrupts are adding events as fast as they are being processed
    // this function might never return.  YOU HAVE BEEN WARNED.
    int processAllEvents();

    /**
     * Application tick - must be called to schedule application
     * after given duration
     * @param ms
     * @return
     */
    static void applicationTick(unsigned long ms);

private:  
    
    // EventQueue class used internally by EventManager
    class EventQueue 
    {

    public:
    
        // Queue constructor
        EventQueue( void );

        // Returns true if no events are in the queue
        bool isEmpty();

        // Returns true if no more events can be inserted into the queue
        bool isFull();

        // Actual number of events in queue
        int getNumEvents();

        // Tries to insert an event into the queue;
        // Returns true if successful, false if the queue if full and the event cannot be inserted
        //
        // NOTE: if EventManager is instantiated in interrupt safe mode, this function can be called
        // from interrupt handlers.  This is the ONLY EventManager function that can be called from
        // an interrupt.
        bool queueEvent( uint8_t eventCode, int eventParam );

        // Tries to extract an event from the queue;
        // Returns true if successful, false if the queue is empty (the parameteres are not touched in this case)
        bool popEvent( uint8_t* eventCode, int* eventParam );

    private:
    
        // Event queue size.
        // The maximum number of events the queue can hold is kEventQueueSize
        // Increasing this number will consume 2 * sizeof(int) bytes of RAM for each unit.
        static const int kEventQueueSize = EVENTMANAGER_EVENT_QUEUE_SIZE;

        struct EventElement
        {
            uint8_t code;	// each event is represented by an integer code
            int param;	// each event has a single integer parameter
        };
        
        // The event queue
        EventElement mEventQueue[ kEventQueueSize ];

        // Index of event queue head
        int mEventQueueHead;

        // Index of event queue tail
        int mEventQueueTail;

        // Actual number of events in queue
        int mNumEvents;
        
    };
    
    
    // ListenerList class used internally by EventManager
    class ListenerList 
    {

    public:

        // Create an event manager
        ListenerList();

        // Add a listener
        // Returns true if the listener is successfully installed, false otherwise (e.g. the dispatch table is full)
        bool addListener( uint8_t eventCode, EventListener* const listener );

        // Remove event listener pair (all occurrences)
        // Other listeners with the same function or eventCode will not be affected
        bool removeListener( uint8_t eventCode, EventListener* const listener );
        
        // Remove all occurrances of a listener
        // Removes this listener regardless of the eventCode; returns number removed
        int removeListener( EventListener* const listener );

        // Enable or disable a listener
        // Return true if the listener was successfully enabled or disabled, false if the listener was not found
        bool enableListener( uint8_t eventCode, EventListener* const listener, bool enable );

        bool isListenerEnabled( uint8_t eventCode, EventListener* const listener );
        
        // Is the ListenerList empty?
        bool isEmpty();
        
        // Is the ListenerList full?
        bool isFull();
        
        // Send an event to the listeners; returns number of listeners that handled the event
        int sendEvent( uint8_t eventCode, int param );

        //checks wether given event has active listener
        bool hasActiveListeners( uint8_t eventCode );

    private:
        // Maximum number of event/listener entries
        // Can be changed to save memory or allow more events to be dispatched
        static const int kMaxListeners = EVENTMANAGER_LISTENER_LIST_SIZE;
        
        // Actual number of event listeners
        int mNumListeners;
        
        // Listener structure and corresponding array
        struct ListenerItem
        {
            EventListener* listener;		// The listener
            uint8_t				eventCode;		// The event code
            bool			enabled;			// Each listener can be enabled or disabled
        };
        ListenerItem mListeners[ kMaxListeners ];

        // get the current number of entries in the dispatch table
        int getNumEntries();

        // returns the array index of the specified listener or -1 if no such event/function couple is found
        int searchListeners( uint8_t eventCode, EventListener* const listener);
        int searchListeners( EventListener* const listener );
        int searchEventCode( uint8_t eventCode );

        };
        //Singleton design pattern
        static EventManager* _my_instance;

        EventQueue 	mHighPriorityQueue;
        EventQueue 	mLowPriorityQueue;
        
        ListenerList		mListeners;
};

//*********  INLINES   EventManager::  ***********

inline bool EventManager::addListener( uint8_t eventCode, EventListener* const listener )
{ 
    return mListeners.addListener( eventCode, listener ); 
}

inline bool EventManager::removeListener( uint8_t eventCode, EventListener* const listener )
{
    return mListeners.removeListener( eventCode, listener );
}

inline int EventManager::removeListener( EventListener* const listener )
{ 
    return mListeners.removeListener( listener );
}

inline bool EventManager::enableListener( uint8_t eventCode, EventListener* const listener, bool enable )
{ 
    return mListeners.enableListener( eventCode, listener, enable ); 
}

inline bool EventManager::isListenerEnabled( uint8_t eventCode, EventListener* const listener )
{ 
    return mListeners.isListenerEnabled( eventCode, listener ); 
}
    
inline bool EventManager::isListenerListEmpty()
{ 
    return mListeners.isEmpty(); 
}
    
inline bool EventManager::isListenerListFull()
{ 
    return mListeners.isFull();
}
    
inline bool EventManager::isEventQueueEmpty( EventPriority pri )
{ 
    return ( pri == kHighPriority ) ? mHighPriorityQueue.isEmpty() : mLowPriorityQueue.isEmpty(); 
}

inline bool EventManager::isEventQueueFull( EventPriority pri )
{ 
    return ( pri == kHighPriority ) ? mHighPriorityQueue.isFull() : mLowPriorityQueue.isFull(); 
}

inline int EventManager::getNumEventsInQueue( EventPriority pri ) 
{ 
    return ( pri == kHighPriority ) ? mHighPriorityQueue.getNumEvents() : mLowPriorityQueue.getNumEvents(); 
}

inline bool EventManager::queueEvent( uint8_t eventCode, int eventParam, EventPriority pri )
{ 
    return ( pri == kHighPriority ) ? 
        mHighPriorityQueue.queueEvent( eventCode, eventParam ) : mLowPriorityQueue.queueEvent( eventCode, eventParam ); 
}




//*********  INLINES   EventManager::EventQueue::  ***********

inline bool EventManager::EventQueue::isEmpty()
{
    return ( mNumEvents == 0 );
}


inline bool EventManager::EventQueue::isFull()
{
    return ( mNumEvents == kEventQueueSize );
}


inline int EventManager::EventQueue::getNumEvents() 
{
    return mNumEvents;
}



//*********  INLINES   EventManager::ListenerList::  ***********

inline bool EventManager::ListenerList::isEmpty()
{ 
    return (mNumListeners == 0); 
}

inline bool EventManager::ListenerList::isFull()
{ 
    return (mNumListeners == kMaxListeners); 
}

inline int EventManager::ListenerList::getNumEntries() 
{ 
    return mNumListeners; 
}


#endif
