CROSS_COMPILER="arm-bcm2708hardfp-linux-gnueabi"

WIRING_PI_INCLUDE_PATH="/home/remi/Projects/raspberry/wiringPi/wiringPi"
LOG_4C_INCLUDE_PATH="/home/data/remi/dev_tools/Embedded/Raspberry/cross_compiled_libraries/log4c-1.2.4/log4c-rpi/include"

WIRING_PI_LIB_PATH="/home/remi/Projects/raspberry/wiringPi/wiringPi"
LOG_4C_LIB_PATH="/home/data/remi/dev_tools/Embedded/Raspberry/cross_compiled_libraries/log4c-1.2.4/log4c-rpi/lib"

EXPAT_LIB_PATH="/home/data/remi/dev_tools/Embedded/Raspberry/cross_compiled_libraries/expat-2.1.0/expat-rpi/lib"

CPP_FLAGS+=-std=gnu++11 -O0 -g -Wall

all : ltc2442_diff_read

ltc2442_diff_read:	 ./ltc2442_diff_read.cpp event_manager.o
	$(CROSS_COMPILER)-g++ ${CPP_FLAGS} -o ltc2442_diff_read ../LTC2449.cpp ./ltc2442_diff_read.cpp event_manager.o -I.. -I../event_manager            \
	-I${WIRING_PI_INCLUDE_PATH} -I${LOG_4C_INCLUDE_PATH} -I./pinout/rpi_2 -I./events -L${LOG_4C_LIB_PATH} -L${WIRING_PI_LIB_PATH} -Wl,-rpath,${EXPAT_LIB_PATH} \
	-lwiringPi -llog4c

event_manager.o: ../event_manager/event_manager.cpp
	$(CROSS_COMPILER)-g++ ${CPP_FLAGS} -o event_manager.o -c ../event_manager/event_manager.cpp -I../event_manager
	
.PHONY:clean
clean:
	rm -f ltc2442_diff_read event_manager.o