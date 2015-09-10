# Description
Linear Technology LTC2442 driver on Arduino. ADC communicates over SPI in 2-wire configuration (refer datasheet p18 "External Serial Clock, 2-Wire I/O"). External interrupt used to detect
EOC. 

# Prerequisities

## Depends on
* Arduino core - https://github.com/Lahorde/arduino_core_avr
* Event manager - https://github.com/Lahorde/event_manager
* Logger - https://github.com/Lahorde/logger.git 
* PinChangeInt - https://github.com/asheeshr/PinChangeInt
* LT_SPI - http://www.linear.com/product/LTC2442#code

## Pinout
A pinout file pinout.h describing used PINS for your application must be provided

## Events
An event file events.h describing all application events. All events listeners will be notified in user context. Events can be emitted in interrupted context.

#TODO
Deliver sample application
