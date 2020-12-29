#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <iostream>

//***********************************
//*********** DEFINITIONS ***********
//***********************************


//***********************************
//*********** MQTT CONFIG ***********
//***********************************
const char *mqttServer = "broker.mqttdashboard.com";
const int mqttPort = 1883;
const char *mqttUser = "ARL210701";
const char *mqttPass = "Shell2020";
const char *subscribe = "ESBCCM2020/input";
const char *publish = "ESBCCM2020";

//***********************************
//*********** WIFI CONFIG ***********
//***********************************
const char* ssid = "angeleivan";
const char* password = "2805042113";


//***********************************
//************* GLOBALS *************
//***********************************
WiFiClient esp;
PubSubClient client(esp);
String speed;
String voltage;
String current;
String error;
String gps;
char send[5];
String buffer;
int count = 1;

//***********************************
//*************** RTOS **************
//***********************************
TaskHandle_t Task1,Task2;

//***********************************
//************ FUNCTIONS ************
//***********************************
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void setup_wifi();
//void communication(void* parameter);


//***********************************
//************** SETUP **************
//***********************************
void setup() {

/*
	xTaskCreatePinnedToCore(
		communication, // Función elegida
		"Task_1", 
		1000, // Stack
		NULL, // No parameters
		1, // Priority
		&Task1, // Created Task
		0 // Core
	);
*/
	Serial.begin(115200);

	setup_wifi();
	client.setServer(mqttServer, mqttPort);
	client.setCallback(callback);
	
}


//***********************************
//************** LOOPS **************
//***********************************

// Communication with MQTT Broker
void loop() {
	if (!client.connected()) {
		reconnect();
	}

	if (client.connected()) {
		itoa(count, send, 10);
		// speed.toCharArray(send, 5);
		client.publish(publish, send);
	}

	count++;

	client.loop();

	delay(1000);
}

//***********************************
//*********** WIFI CONNECT **********
//***********************************
void setup_wifi() {
	delay(10);

	Serial.println();
	Serial.println("Conectando a " + String(ssid));

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("Conectado a red WiFi");
	Serial.println("Dirección IP: " + String(WiFi.localIP()));
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


//***********************************
//*********** MQTT CONNECT **********
//***********************************
void reconnect() {
	while (!client.connected()) {
		Serial.print("Intentando conexión MQTT...");

		// Creamos un cliente ID
		String clientId = "EBC05";
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

