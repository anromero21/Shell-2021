#include "HBridge.h"

HBridge::HBridge(uint8_t pin1, uint8_t channel1, uint8_t pin2, uint8_t channel2, double frequency, uint8_t bits_resolution)
{
	pwm1.setup(pin1, channel1, frequency, bits_resolution, HIGH);
	pwm2.setup(pin2, channel2, frequency, bits_resolution, HIGH);
}

HBridge::~HBridge()
{
}

void HBridge::setSpeed(float speed)
{
	if (speed < 0)
	{
		pwm1.setDuty(0);
		pwm2.setDuty(abs(speed));
	}
	else
	{
		pwm1.setDuty(speed);
		pwm2.setDuty(0);
	}
}

void HBridge::setStop(uint8_t mode)
{
	switch (mode)
	{	
		case STOP:
			pwm1.setDuty(100);
			pwm2.setDuty(100);
			break;
		case COAST:
			pwm1.setDuty(0);
			pwm2.setDuty(0);
			break;
	}
}

void HBridge::setFrequency(float frequency)
{
	pwm1.setFrequency(frequency);
	pwm2.setFrequency(frequency);
}
