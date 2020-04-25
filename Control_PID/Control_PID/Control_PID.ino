/*
 Name:		Control_PID.ino
 Created:	31/10/2019 12:12:35
 Author:	irvingsixto
*/

// the setup function runs once when you press reset or power the board

void setup() {
	Serial.begin(115200);
	pinMode(34, INPUT); //Fotoresistencia en Colector (cable naranja);
	pinMode(32, INPUT); //Fotoresistencia en Proto (cable azul)
}
float FProto;
float FColec;

// the loop function runs over and over again until power down or reset
void loop() {
	FProto=digitalRead(32);
	FColec=digitalRead(34);
	Serial.println("Voltaje en colector: ");
	Serial.println(34);
	Serial.println("Voltaje en protoboard: ");
	Serial.println(32);

	delay(2000);
}
