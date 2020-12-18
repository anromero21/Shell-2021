#include "LED_RGB.h"

RGB::RGB(uint8_t rgb_pins[3], uint8_t rgb_channels[3], bool on_state)
{
	pwmR.setup(rgb_pins[0], rgb_channels[0], 1000, 8, on_state);
	pwmG.setup(rgb_pins[1], rgb_channels[1], 1000, 8, on_state);
	pwmB.setup(rgb_pins[2], rgb_channels[2], 1000, 8, on_state);
	_on_state = on_state;
}

RGB::~RGB()
{
}

void RGB::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
	pwmR.setBits(red);
	pwmG.setBits(green);
	pwmB.setBits(blue);
}

void RGB::setColor(uint8_t color)
{
	switch (color)
	{
	case OFF:
		pwmR.setBits(0);
		pwmG.setBits(0);
		pwmB.setBits(0);
		break;
	case WHITE:
		pwmR.setBits(255);
		pwmG.setBits(255);
		pwmB.setBits(255);
		break;
	case RED:
		pwmR.setBits(255);
		pwmG.setBits(0);
		pwmB.setBits(0);
		break;
	case GREEN:
		pwmR.setBits(0);
		pwmG.setBits(255);
		pwmB.setBits(0);
		break;
	case BLUE:
		pwmR.setBits(0);
		pwmG.setBits(0);
		pwmB.setBits(255);
		break;
	case YELLOW:
		pwmR.setBits(255);
		pwmG.setBits(255);
		pwmB.setBits(0);
		break;
	case PURPLE:
		pwmR.setBits(255);
		pwmG.setBits(0);
		pwmB.setBits(255);
		break;
	case ORANGE:
		pwmR.setBits(255);
		pwmG.setBits(128);
		pwmB.setBits(0);
		break;
	}
}