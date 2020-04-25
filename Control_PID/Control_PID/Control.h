// ControlPID.h

#ifndef _CONTROL_h
#define _CONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
class PID
{
public:
	PID(float kp, float ki, float kd, float reference, uint16_t dt_ms);
	void setdt(uint16_t dt_ms);
	void setGains(float kp, float ki, float kd);
	void setReference(float reference);
	float doControl(float measurement);
private:
	float _kp;
	float _ki;
	float _kd;
	float _reference;
	float _dt;
	float _error_prev =0;
	float _integral
	
};

#endif


