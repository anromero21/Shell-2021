#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

//***********************************
//*********** DEFINITIONS ***********
//***********************************


//***********************************
//*********** MQTT CONFIG ***********
//***********************************
const char *mqttServer = "ioticos.org";
const int mqttPort = 1883;
const char *mqttUser = "VjclYKIMveCXbG0";
const char *mqttPass = "2xAfdxh1sV7owRE";
const char *subscribe = "Fb91eInFO93GxEe/input";
const char *publish_s = "Fb91eInFO93GxEe/speed";
const char *publish_e = "Fb91eInFO93GxEe/error";
const char *publish_v = "Fb91eInFO93GxEe/voltage";
const char *publish_c = "Fb91eInFO93GxEe/current";
const char *publish_g = "Fb91eInFO93GxEe/gps";

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
char sendS[5];
char sendV[5];
char sendC[5];
char sendE[5];
char sendG[5];
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
void communication(void* parameter);


//***********************************
//************** SETUP **************
//***********************************
void setup() {

	xTaskCreatePinnedToCore(
		communication, // Función elegida
		"Task_1", 
		1000, // Stack
		NULL, // No parameters
		1, // Priority
		&Task1, // Created Task
		0 // Core
	);
	
/*
	xTaskCreatePinnedToCore(
		GPS, // Función elegida
		"Task 2",
		1000, // Stack
		NULL, // No parameters
		1, // Priority
		&Task2, // Created Task
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
			//String vel = String(v);
			//values[5].toCharArray(sendVal, 10);
			speed.toCharArray(sendS, 5);
			voltage.toCharArray(sendV, 5);
			current.toCharArray(sendC, 5);
			error.toCharArray(sendE, 5);
			gps.toCharArray(sendG, 5);
			client.publish(publish_s, sendS);
			client.publish(publish_v, sendV);
			client.publish(publish_c, sendC);
			client.publish(publish_e, sendE);
			client.publish(publish_g, sendG);
		}

		client.loop();
}

void communication(void *parameters){
	for(;;){
		while(Serial.available()){
			char in = (char)Serial.read();
			
			if (in != '/'){
				buffer += in;
			}

			else if(in == '/' && count == 1){ // Speed
				speed = buffer;
				buffer = "";
				count++;
			}

			else if(in == '/' && count == 2){ // Voltage
				voltage = buffer;
				buffer = "";
				count++;
			}

			else if(in == '/' && count == 3){ // Current
				current = buffer;
				buffer = "";
				count++;
			}

			else if(in == '/' && count == 4){ // Error
				error = buffer;
				buffer = "";
				count++;
			}

			else if(in == '/' && count == 5){ // Gps
				gps = buffer;
				buffer = "";
				count = 1;
			}
		}
	}
	vTaskDelay(10);
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

