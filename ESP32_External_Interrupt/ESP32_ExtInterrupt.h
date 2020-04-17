#ifndef _ESP32_EXTINTERRUPT_h
#define _ESP32_EXTINTERRUPT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class ExternalInterrupt
{
public:
	ExternalInterrupt();
	~ExternalInterrupt();
	void begin(uint8_t pin, uint8_t type, uint16_t bouncing_time_us);
	bool available();
	bool getState();
	bool getStatePrev();
	bool getEdge();
	ulong getElapsedMicros();
protected:
	void IRAM_ATTR handle();
	uint8_t _pin;
	uint16_t _bouncing_time;
	ulong _delta_micros;
	ulong _micros_prev;
	bool _interrupt;
	bool _state;
	bool _state_prev;
	uint8_t _edge;
};

#endif

