/*
 Name:		Posicion_velocidad.ino
 Created:	29/10/2019 11:17:01
 Author:	irvingsixto
*/

// the setup function runs once when you press reset or power the board

#include <BluetoothSerial.h>
#include <Control.h>
#include <PWM_ESP32.h>
#include <ESP32_ExtInterrupt.h>
#include "HBridge.h"


#define DELTA_ANGLE 0.640324431f
#define PMODE 14
#define SLEEP 25
#define IMODE 33


ExternalInterrupt extint;
HBridge motor(26, 27, 0, 1, 1000);
PID control(1,2,0,0,10);
BluetoothSerial SerialBT; 


void setup() {
	extint.begin(19, RISING, 400);
	pinMode(5, INPUT);
	pinMode(PMODE, OUTPUT);
	pinMode(SLEEP, OUTPUT);
	pinMode(IMODE, OUTPUT);
	digitalWrite(PMODE, HIGH);
	digitalWrite(SLEEP, HIGH);
	pinMode(IMODE, HIGH);

	Serial.begin(115200);
	motor.setSpeed(0);
	SerialBT.begin("Equipo cool");
}
// the loop function runs over and over again until power down or reset

ulong tiempo;
float velocidad;
ulong delta;
ulong anterior = 0.0;
float rpm_encoder;
float rpm_motor;
int8_t sentido;
int count = 0;
String data_in;
float velocidad_p;
ulong micros_prev;
float output;
float posicion_e;
float posicion_m;
int i;

void loop() {
	//Función rampa a diferentes velocidades
	for (i = 0; i <= 100; i++)
	{
	 motor.setSpeed(i);
	 Serial.println(i);
	 delay(100);
	}
	for (i = 100; i >=-100; i--)
	{
	 motor.setSpeed(i);
	 Serial.println(i);
	 delay(100);
	}
	for (i = -100; i <= 0; i++)
	{
	 motor.setSpeed(i);
	 Serial.println(i);
	 delay(100);
	}
}

/*
void loop() {
	//if (Serial.available()){
	//	data_in = Serial.readStringUntil('\n');
	//	control.setReference(data_in.toFloat());
	//	//motor.setSpeed(data_in.toFloat());
	//	Serial.println(data_in);
	//}
	
	if (extint.available())

	{

		//       Metodología nuestra
		// //delta = micros() - anterior;
		// //anterior = micros();
		// //Serial.println(delta);
		// //Serial.println(micros());

		tiempo = extint.getElapsedMicros();		
		velocidad = DELTA_ANGLE / (tiempo / 1000000.0);
		rpm_motor = (velocidad * 60.0f) / 360.0f;
		//rpm_motor = rpm_encoder / 34.014;
		//Serial.println(rpm_motor);
		velocidad_p = rpm_motor / 2.9f;
		

		// Dirección de giro 
		 if (digitalRead(34))
		 {
		  sentido = 1;
		  //Serial.println("CCW");
		  count++;
		 }
		 else
		 {
		  sentido = 0;
		  //Serial.println("CW");
		  count--;
		 }
		 posicion_m = count*DELTA_ANGLE;
		 //Serial.println(posicion_m);
		// Serial.println(sentido);
		// Serial.print(",");
		// Serial.println(rpm_motor);
		// //Serial.println(delta/1000000.0);

	}
	/*if (micros() - micros_prev >= 10000)
	{
		micros_prev = micros();
		output = control.doControl(posicion_m);
		motor.setSpeed(output);
		Serial.print(count);
		Serial.print(",");
		Serial.println(posicion_m);
		Serial.print(",");
		Serial.println(output);
	}*/
	//Función rampa a diferentes velocidades
	/*for (i = 0; i <= 100; i++)
	{
	 motor.setSpeed(i);
	 Serial.println(i);
	 delay(100);
	}
	for (i = 100; i >=-100; i--)
	{
	 motor.setSpeed(i);
	 Serial.println(i);
	 delay(100);
	}
	for (i = -100; i <= 0; i++)
	{
	 motor.setSpeed(i);
	 Serial.println(i);
	 delay(100);
	}*/
	//Serial.println(i);
//}
