#include "HBridge.h"
HBridge::HBridge(uint8_t in1, uint8_t in2, uint8_t channel1, uint8_t channel2, float frecuency)
{
	pwm1.setup(in1, channel1, frecuency, 10, HIGH);
	pwm2.setup(in2, channel2, frecuency, 10, HIGH);
}
void HBridge::setSpeed(float Speed)
{
	if (Speed > 0)
	{
		pwm1.setDuty(0);
		pwm2.setDuty(Speed);
	}
	else
	{
		pwm2.setDuty(0);
		pwm1.setDuty(abs(Speed));
	}
}
void HBridge::setStop(uint8_t stop)
{
	if (stop == BRAKE)
	{
		pwm1.setDuty(100);
		pwm2.setDuty(100);
	}
	if (stop == COAST)
	{
		pwm1.setDuty(100);
		pwm2.setDuty(100);
	}
}
void HBridge::setFrecuency(float frecuency)
{
	pwm1.setFrequency(frecuency);
	pwm2.setFrequency(frecuency);
}

