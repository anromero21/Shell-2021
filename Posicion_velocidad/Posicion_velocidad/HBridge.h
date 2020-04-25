// HBridge.h
#ifndef _HBRIDGE_h
#define _HBRIDGE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"

#endif

#include <PWM_ESP32.h>
#define COAST 0
#define BRAKE 1
class HBridge
{
public:
	HBridge(uint8_t in1, uint8_t in2, uint8_t channel1, uint8_t channel2, float frecuency);
	PWM pwm1;
	PWM pwm2;
	void setSpeed(float Speed);
	void setStop(uint8_t stop);
	void setFrecuency(float frecuency);
};
#endif

