// 
// 
// 

#include "Control.h"

PID::PID(float kp, float ki, float kd, float reference, uint16_t dt_ms)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
	_reference = reference;
	_dt = dt_ms / 1000;
	_error_prev = 0;
	_integral = 0;
}

void PID::setdt(uint16_t dt_ms)
{
	_dt = dt_ms/1000;
}

void PID::setGains(float kp, float ki, float kd)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
}

void PID::setReference(float reference)
{
	_reference = reference;
}

float PID::doControl(float measurement)
{
	float error = _reference - measurement;
	_integral = _integral + error * _dt;
	float output = _kp * error + _ki*_integral + _kd*(error - _error_prev)/_dt;
	_error_prev = error;
	return output;
}
