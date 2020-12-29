#include <Arduino.h>
#include <ESP32_ExtInterrupt.h>
#include <PWM_ESP32.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

//#include <BluetoothSerial.h>

/*

HA = blanco
HB = naranja
HC = morado

*/

//***********************************
//************* FORMULAS ************
//***********************************
/* 

Itrip = (Vref - Voos)/(Rsense*Av)    Av = 19   Voos = 320mV   Rsense = .005ohm    0 < Vref < 4V     Itrip(max) = 38.7368A

Vcsout = Iload*Av*Rsense + Voos   Vcsout < 5V   Iload(max) = 49.2632A

fosc = 1/(RtCt + tblank + tdead)   fosc = 20kHz     5 < Rt < 400kohms
tblank(internal)us = 1260*Ct(uF)
tblank(external)us = 2000*Ct(uF)    tblank = ns

tdead(us) = 0.1 + 33/(5 + 2000/Rdead)   tdead = us
tdead = 6u
Rt = 51kohm
Ct = 844pF

duty_cycle

*/


//***********************************
//*************** OTA ***************
//***********************************
const char* ssid = "angeleivan";
const char* password = "2805042113";


//***********************************
//*********** DEFINITIONS ***********
//***********************************
// INPUT    D-I
#define FF1 36  // VP
#define FF2 39  // VN
#define CSOUT 34
#define DIRO 35
#define TACHO 32

// BUTTONS    I-D
#define B_DIR 33
#define B_COAST 25
#define B_BRAKE 26
#define B_MODE 27
#define B_RESET 14

// OUTPUT
#define RST 23
#define DIR 22
#define PWM_PIN 21
#define MODE 19
#define COAST 18
#define BRAKE 5
#define ESF 17
#define REF 4
#define VDSTH 0

// CONSTANTS
#define AV 19
#define VOOS 0.32     // V
#define RSENSE 0.005  // ohms
#define RT 200000     // 200kohms
#define DELTA_ANGLE 0.0951998
#define CONVERSION 9.5493 

const char *mqttServer = "broker.mqttdashboard.com";
const int mqttPort = 1883;
const char *mqttUser = "ARL210701";
const char *mqttPass = "Shell2020";
const char *subscribe = "ESBCCM2020/input";
const char *publish = "ESBCCM2020";
const char *publish_R = "ESBCCM2020/RPM";
const char *publish_LC = "ESBCCM2020/LOAD";
const char *publish_RV = "ESBCCM2020/REF";
const char *publish_DR = "ESBCCM2020/DUTY";

ExternalInterrupt tachoInterrupt;
ExternalInterrupt brakeInterrupt;
ExternalInterrupt coastInterrupt;
ExternalInterrupt resetInterrupt;
ExternalInterrupt dirInterrupt;
ExternalInterrupt FF1Interrupt;
ExternalInterrupt FF2Interrupt;

PWM pwmPin;
PWM vdsthPin;
PWM refPin;

SemaphoreHandle_t xSemaphore;

float dutyCycle = 0;
float rpm;
int elapsedMillis;

float tripCurrent = 0;
float fosc = 0;
int refVoltge = 0;
int tBlank = 0;
int tDead = 0;
int csoutDividerVoltage = 0;
float csoutVoltage = 0;
float loadCurrent = 0;
float motorRads = 0;

float refVoltage = 0;
int dutyRef = 0;


//***********************************
//************** MQTT ***************
//***********************************
WiFiClient esp;
PubSubClient client(esp);
String speed;
String voltage;
String current;
String error;
String gps;
char sendR[6];
char sendLC[6];
char sendRV[6];
char sendDR[6];
String buffer;
int count = 1;


//***********************************
//*************** RTOS **************
//***********************************
TaskHandle_t Task1, Task2, Task3, Task4, Task5;


//***********************************
//************ FUNCTIONS ************
//***********************************
void faults(void *parameter);
void sensing(void *parameter);
void speedDir(void *parameter);
void currentControl(void* parameter);
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void communication(void* parameter);


//***********************************
//************** SETUP **************
//***********************************
void setup() {
  Serial.begin(115200);

  pinMode(RST, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MODE, OUTPUT);
  pinMode(COAST, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(ESF, OUTPUT);
  pinMode(CSOUT, INPUT);
  pinMode(DIRO, INPUT);
  // pinMode(POT, INPUT);

  pwmPin.setup(PWM_PIN, 1, 15000, 12, 1);
  vdsthPin.setup(VDSTH, 2, 15000, 12, 1);
  refPin.setup(REF, 3, 15000, 12, 1);

  tachoInterrupt.begin(TACHO, CHANGE, 100);  
  brakeInterrupt.begin(B_BRAKE, CHANGE, 100);  
  dirInterrupt.begin(B_DIR, CHANGE, 100); 
  coastInterrupt.begin(B_COAST, CHANGE, 100); 
  resetInterrupt.begin(B_RESET, CHANGE, 100); 
  FF1Interrupt.begin(FF1, CHANGE, 100); 
  FF2Interrupt.begin(FF2, CHANGE, 100); 

  digitalWrite(DIR, HIGH);
  digitalWrite(ESF, HIGH);
  digitalWrite(MODE, LOW);

  tripCurrent = 8;


  xTaskCreatePinnedToCore(
		  faults, // Función elegida
		  "Task 1", 
		  1000, // Stack
		  NULL, // No parameters
		  2, // Priority
		  &Task1, // Created Task
		  0 // Core
	  );  

  xTaskCreatePinnedToCore(
		  sensing, // Función elegida
		  "Task 2", 
		  1000, // Stack
		  NULL, // No parameters
		  1, // Priority
		  &Task2, // Created Task
		  0 // Core
	  );  

    xTaskCreatePinnedToCore(
		  speedDir, // Función elegida
		  "Task 3", 
		  1000, // Stack
		  NULL, // No parameters
		  1, // Priority
		  &Task3, // Created Task
		  1 // Core
	  );  
    
    xTaskCreatePinnedToCore(
		  currentControl, // Función elegida
		  "Task 4", 
		  1000, // Stack
		  NULL, // No parameters
		  1, // Priority
		  &Task4, // Created Task
		  1 // Core
	  );

    xTaskCreatePinnedToCore(
		communication, // Función elegida
		"Task_5", 
		1000, // Stack
		NULL, // No parameters
		2, // Priority
		&Task5, // Created Task
		0 // Core
	);

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  client.setServer(mqttServer, mqttPort);
	client.setCallback(callback);

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


//***********************************
//*************** LOOP **************
//***********************************
void loop() { // OK
ArduinoOTA.handle();

if (!client.connected()) {
			reconnect();
		}

		if (client.connected()) {
      dtostrf(rpm, 4, 3, sendR);
      dtostrf(loadCurrent, 4, 3, sendLC);
      dtostrf(dutyRef, 4, 3, sendDR);
      dtostrf(refVoltage, 4, 3, sendRV);

			// rpm.toCharArray(sendR, 6);
			// loadCurrent.toCharArray(sendLC, 6);
			// dutyRef.toCharArray(sendDR, 6);
			// refVoltage.toCharArray(sendRV, 6);
    
    //if( xSemaphore != NULL ){
			client.publish(publish_R, sendR);
			client.publish(publish_LC, sendLC);
			client.publish(publish_DR, sendDR);
			client.publish(publish_RV, sendRV);

      delay(1500);
    //}
		}

		client.loop();

/*
  ArduinoOTA.handle();
  
  vdsthPin.setDuty(30);

  bool dReset = resetInterrupt.getState();
  bool dDir = dirInterrupt.getState();
  bool dCoast = coastInterrupt.getState();
  bool dBrake = coastInterrupt.getState();

  Serial.print("DIR: ");
  Serial.print(dDir);
  Serial.print("  ");
  Serial.print("COAST: ");
  Serial.print(dCoast);
  Serial.print("BRAKE: ");
  Serial.print(dBrake);
  Serial.print("RESET: ");
  Serial.print(dReset);

  digitalWrite(DIR, HIGH);
  digitalWrite(COAST, dCoast);
  digitalWrite(BRAKE, HIGH);
  digitalWrite(RST, HIGH);
  */
}


//***********************************
//*********** COMMUNICATION *********
//***********************************
void communication(void *parameters){
  //xSemaphore = xSemaphoreCreateMutex();

	for(;;){  
  vdsthPin.setDuty(30);

  bool dReset = resetInterrupt.getState();
  bool dDir = dirInterrupt.getState();
  bool dCoast = coastInterrupt.getState();
  bool dBrake = coastInterrupt.getState();

  Serial.print("DIR: ");
  Serial.print(dDir);
  Serial.print("  ");
  Serial.print("COAST: ");
  Serial.print(dCoast);
  Serial.print("BRAKE: ");
  Serial.print(dBrake);
  Serial.print("RESET: ");
  Serial.print(dReset);

  // digitalWrite(DIR, HIGH);
  digitalWrite(COAST, dCoast);
  digitalWrite(BRAKE, dBrake);
  digitalWrite(RST, HIGH);
/*
    if (!client.connected()) {
			reconnect();
		}

		if (client.connected()) {
      dtostrf(rpm, 4, 3, sendR);
      dtostrf(loadCurrent, 4, 3, sendLC);
      dtostrf(dutyRef, 4, 3, sendDR);
      dtostrf(refVoltage, 4, 3, sendRV);

			// rpm.toCharArray(sendR, 6);
			// loadCurrent.toCharArray(sendLC, 6);
			// dutyRef.toCharArray(sendDR, 6);
			// refVoltage.toCharArray(sendRV, 6);
    
    if( xSemaphore != NULL ){
			client.publish(publish_R, sendR);
			client.publish(publish_LC, sendLC);
			client.publish(publish_DR, sendDR);
			client.publish(publish_RV, sendRV);
    }
		}

		client.loop();
    */

    vTaskDelay(10);
	}
}


//***********************************
//************ SPEED-DIR ************
//***********************************
void speedDir(void* parameter) { // OK
  for (;;) {
    if (tachoInterrupt.available() && dirInterrupt.available()){
      bool direction = dirInterrupt.getState();
      float time = tachoInterrupt.getElapsedMicros();		
	    motorRads = DELTA_ANGLE / (time / 1000000.0);
	    rpm = (motorRads * CONVERSION); //60.0f) / 360.0f;

      digitalWrite(DIR, direction);

      Serial.print("RPM: ");
	    Serial.println(rpm);
    }

    vTaskDelay(10);
	}
}


//***********************************
//************* SENSING *************
//***********************************
void sensing(void* parameter) { // OK
  for (;;) {
    csoutDividerVoltage = analogRead(CSOUT); // 3.3V/4096
    Serial.print("CSOUT DIVIDER VOLTAGE: ");
    Serial.println(csoutDividerVoltage);

    csoutVoltage = (3.3 * csoutDividerVoltage / 4096) * 3.2/2.2; // Divisor de voltaje
    Serial.print("CSOUT: ");
    Serial.println(csoutVoltage);

    loadCurrent = (csoutVoltage - VOOS) / (AV * RSENSE);   
    Serial.print("LOAD CURRENT: ");
    Serial.println(loadCurrent);

    vTaskDelay(10);
	}
}


//***********************************
//********* CURRENT CONTROL *********
//***********************************
void currentControl(void* parameter) { // OK
  for (;;) {
    // int lec = analogRead(POT); // 3.3V/4096
    // int conv = map(lec, 0, 4096, 0, 100);

    refVoltage = tripCurrent * (RSENSE * AV) + VOOS;

    dutyRef = refVoltage * 100 / 3.3;
    if (dutyRef > 100) dutyRef = 100;

    pwmPin.setDuty(100); // 100-Torque Control     0-Current recirculate
    refPin.setDuty(dutyRef) ;

    vTaskDelay(10);
	}
}


//***********************************
//************** FAULTS *************
//***********************************
void faults(void* parameter) {
  for (;;) {
    int l1 = FF1Interrupt.getState();
    int l2 = FF2Interrupt.getState();

		if (l1 == 1 && l2 == 1) continue;
    else if (l1 == 0 && l2 == 0) Serial.println("Undervoltage, Overtemperature or Logic fault");
    else if (l1 == 1 && l2 == 0) Serial.println("Short to ground, short to supply or shorted motor winding");
    else if (l1 == 0 && l2 == 1) Serial.println("Low load current");
    
    vTaskDelay(10);
	}
}


//***********************************
//*********** MQTT CONNECT **********
//***********************************
void reconnect() {
	while (!client.connected()) {
		Serial.print("Intentando conexión MQTT...");

		// Creamos un cliente ID
		String clientId = "Shell";
		clientId += String(random(0xffff), HEX);

		// Conectamos
		if (client.connect(clientId.c_str(), mqttUser, mqttPass)) {
			Serial.println("Conectado");

			// Nos suscribimos
			if (client.subscribe(subscribe)) {
				Serial.println("Suscripción ok");
			}

			else
				Serial.println("Fallo en suscripción");
		}

		else {
			Serial.print("Falló conexión. Error ->" + String(client.state()));
			Serial.println("Reintentando en 5 segundos");
			delay(5000);
		}
	}
}


//***********************************
//************* CALLBACK ************
//***********************************
void callback(char* topic, byte* payload, unsigned int length) {
	String incoming = "";
	Serial.println("Mensaje recibido desde ->" + String(topic));
	
	for (int i = 0; i < length; i++) {
		incoming += (char)payload[i];
	}
	incoming.trim();
	Serial.println("Mensaje ->" + incoming);
}
