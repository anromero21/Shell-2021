#ifndef _PWM_ESP32_h
#define _PWM_ESP32_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class PWM
{
public:
	PWM();
	~PWM();
	void setup(uint8_t pin, uint8_t channel, double frequency, uint8_t bits_resolution, bool on_state);
	void setDuty(float duty_cycle);
	void setBits(uint32_t bits);
	void setFrequency(float frequency);
	void attachPin(uint8_t pin);
protected:
	uint8_t _pin;
	uint8_t _channel;
	uint32_t _resolution;
	bool _on_state;
	float _duty_cycle;
	float _frequency;
};

#endif

