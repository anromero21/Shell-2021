#include "ESP32_ExtInterrupt.h"
#include <FunctionalInterrupt.h>

ExternalInterrupt::ExternalInterrupt()
{
}

ExternalInterrupt::~ExternalInterrupt()
{
	detachInterrupt(_pin);
}

void ExternalInterrupt::begin(uint8_t pin, uint8_t type, uint16_t bouncing_time_us)
{
	_pin = pin;
	_bouncing_time = bouncing_time_us;
	pinMode(_pin, INPUT_PULLUP);
	attachInterrupt(_pin, std::bind(&ExternalInterrupt::handle, this), type);
}

void IRAM_ATTR ExternalInterrupt::handle()
{
	_delta_micros = micros() - _micros_prev;
	if (_delta_micros >= _bouncing_time)
	{
		_interrupt = true;
		_micros_prev = micros();
		_state_prev = _state;
		_state = digitalRead(_pin);
		if (_state_prev > _state)
			_edge = FALLING;
		else
			_edge = RISING;
	}
}

bool ExternalInterrupt::available()
{
	bool interrupt = _interrupt;
	_interrupt = false;
	return interrupt;
}

bool ExternalInterrupt::getState()
{
	return _state;
}

bool ExternalInterrupt::getStatePrev()
{
	return _state_prev;
}

bool ExternalInterrupt::getEdge()
{
	return _edge;
}

ulong ExternalInterrupt::getElapsedMicros() 
{
	return _delta_micros;
}

