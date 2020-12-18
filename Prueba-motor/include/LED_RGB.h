#ifndef _LED_RGB_h
#define _LED_RGB_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "PWM_ESP32.h"

#define OFF 0
#define WHITE 1
#define RED 2
#define GREEN 3
#define BLUE 4
#define YELLOW 5
#define PURPLE 6
#define ORANGE 7

class RGB
{
public:
	RGB(uint8_t rgb_pins[3], uint8_t rgb_channels[3], bool on_state);
	~RGB();
	void setColor(uint8_t red, uint8_t green, uint8_t blue);
	void setColor(uint8_t color);
protected:
	PWM pwmR;
	PWM pwmG;
	PWM pwmB;
	bool _on_state;
};

#endif

