#include <DRV8323H.h>
#include <ESP32_ExtInterrupt.h>

/*
    Name:       BLDC_Test.ino
    Created:  September 15, 2019
    Author:     Israel Cayetano
*/


//#include <BluetoothSerial.h>
//#include <Exponential_Smoothing.h>
//#include <Useful_Methods.h>
//#include <ADC_Read.h>

#define CONVERSION 15000000.0f

uint8_t inh_pins[] = { 26, 27, 14 };
uint8_t inl_pins[] = { 32, 33, 25 };
uint8_t sensor_pins[] = { 19, 18, 5 };  // C B A
uint8_t nfault_pin = 23;
uint8_t en_pin = 0;
uint8_t cal_pin = 12;
uint8_t channels[] = { 0,1,2 };
BLDC bldc(inh_pins, inl_pins, sensor_pins, nfault_pin, en_pin);
ExternalInterrupt hallA;
ExternalInterrupt hallB;
ExternalInterrupt hallC;
// ExpSmoothing filter(0.95);
String string_in;
float duty_cycle = 20;
float data_in;
float rpm;
int elapsed_millis;

void setup()
{
  //SerialBT.begin("Shell Borregos CCM");
  Serial.begin(115200);
  hallA.begin(sensor_pins[0], CHANGE, 0);
  hallB.begin(sensor_pins[1], CHANGE, 0);
  hallC.begin(sensor_pins[2], CHANGE, 0);
  bldc.begin(channels, 1000);
  Serial.println("BLDC initialized");
  delay(1000);
}

void loop()
{
  if (hallA.available() || hallB.available() || hallC.available())
  {
    bldc.readSensors();
    bldc.doSequence(bldc.sens_a, bldc.sens_b, bldc.sens_c, FORWARD, duty_cycle);
  }
  if (Serial.available())
  {
    string_in = Serial.readStringUntil('\n');
    duty_cycle = string_in.toFloat();
  }
  if (millis() % 200 == 0)
  {
    elapsed_millis = hallA.getElapsedMicros()/1000;
    if (elapsed_millis < 2000)
      rpm = CONVERSION / hallB.getElapsedMicros();
    else
      rpm = 0;
    Serial.printf("%.2f, %.2f\n", duty_cycle,rpm);
  }
}
