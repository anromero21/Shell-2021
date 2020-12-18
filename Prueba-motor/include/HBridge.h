#ifndef _HBRIDGE_h
#define _HBRIDGE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <PWM_ESP32.h>

#define STOP 0
#define COAST 1

class HBridge
{
public:
	HBridge(uint8_t pin1, uint8_t channel1, uint8_t pin2, uint8_t channel2, double frequency, uint8_t bits_resolution);
	~HBridge();
	PWM pwm1;
	PWM pwm2;
	void setSpeed(float speed);
	void setStop(uint8_t mode);
	void setFrequency(float frequency);
private:
	float _pwm;
};

#endif

