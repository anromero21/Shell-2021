#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "string.h"
/*
#include <ESP32_ExtInterrupt.h>
#include <PWM_ESP32.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>


/*
HA = blanco
HB = naranja
HC = morado

//***********************************
//************* FORMULAS ************
//***********************************
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
const char *ssid = "angeleivan";
const char *password = "2805042113";

//***********************************
//*********** DEFINITIONS ***********
//***********************************
// INPUT    D-I
#define DIRO (GPIO_NUM_36) // VP    // IT
#define FF2 (GPIO_NUM_39)  // VN    // IT
#define FF1 (GPIO_NUM_34)           // IT
#define TACHO (GPIO_NUM_35)         // IT
#define CSOUT (GPIO_NUM_32)

// BUTTONS    I-D       // 2da ESP
#define B_DIR GPIO_NUM_33
#define B_COAST GPIO_NUM_25
#define B_BRAKE GPIO_NUM_26
#define B_MODE GPIO_NUM_27
#define B_RESET GPIO_NUM_14

// OUTPUT
#define COAST (GPIO_NUM_23)
#define ESF (GPIO_NUM_22)
#define RST (GPIO_NUM_21)
#define BRAKE (GPIO_NUM_19)
#define DIR (GPIO_NUM_18)
#define PWM_PIN (GPIO_NUM_5)    // PWM
#define MODE (GPIO_NUM_25)
#define REF (GPIO_NUM_4)        // PWM
#define VDSTH (GPIO_NUM_0)      // PWM

// COMMUNICATION
#define TXD_PIN_2 (GPIO_NUM_17)
#define RXD_PIN_2 (GPIO_NUM_16)

// CONSTANTS
#define AV 19
#define VOOS 0.32    // V
#define RSENSE 0.005 // ohms
#define RT 200000    // 200kohms
#define DELTA_ANGLE 0.0951998
#define CONVERSION 9.5493

// BIT-MASK
#define GPIO_I_BIT_MASK  ((1ULL<<CSOUT)  
#define GPIO_O_BIT_MASK  ((1ULL<<COAST) | (1ULL<<ESF)) | (1ULL<<RST)) | (1ULL<<BRAKE)) | (1ULL<<DIR)) | (1ULL<<MODE))  

const char *mqttServer = "broker.mqttdashboard.com";
const int mqttPort = 8000;
const char *mqttUser = "ARL210701";
const char *mqttPass = "Shell2020";
const char *subscribe = "ESBCCM2020/input";
const char *publish = "ESBCCM2020";
const char *publish_R = "ESBCCM2020/RPM";
const char *publish_LC = "ESBCCM2020/LOAD";
const char *publish_RV = "ESBCCM2020/REF";
const char *publish_DR = "ESBCCM2020/DUTY";
const char *publish_CV = "ESBCCM2020/CSOUT";
const char *publish_F = "ESBCCM2020/FAULTS";

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
char sendR[6];
char sendLC[6];
char sendRV[6];
char sendDR[6];
char sendCV[6];
char sendF[20];
String fault;

//***********************************
//*************** RTOS **************
//***********************************
TaskHandle_t Task1, Task2, Task3, Task4, Task5, Task6, Task7;

//***********************************
//************ FUNCTIONS ************
//***********************************
void faults(void *parameter);
void sensing(void *parameter);
void speedDir(void *parameter);
void currentControl(void *parameter);
void buttons(void *parameter);
void communication(void *parameter);
void lora(void *parameter);

void setup_wifi();
void callback(char *topic, byte *payload, unsigned int length);
void reconnect();

void app_main(void)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    gpio_config_t i_conf;
    i_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    i_conf.mode = GPIO_MODE_INPUT;
    i_conf.pin_bit_mask = GPIO_I_BIT_MASK;
    i_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    i_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    esp_err_t error=gpio_config(&i_conf);
    if(error!=ESP_OK){
        printf("Error configuring inputs \n");
    }

    gpio_config_t o_conf;
    o_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    o_conf.mode = GPIO_MODE_OUTPUT;
    o_conf.pin_bit_mask = GPIO_O_BIT_MASK;
    o_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    o_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    esp_err_t error=gpio_config(&o_conf);
    if(error!=ESP_OK){
        printf("Error configuring outputs \n");
    }

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

    xTaskCreatePinnedToCore(faults, "Task 1", 1000, NULL, 2, &Task1, 0);
    xTaskCreatePinnedToCore(sensing, "Task 2", 1000, NULL, 1, &Task2, 0);
    xTaskCreatePinnedToCore(speedDir, "Task 3", 1000, NULL, 1, &Task3, 1);
    xTaskCreatePinnedToCore(currentControl, "Task 4", 1000, NULL, 1, &Task4, 1);
    xTaskCreatePinnedToCore(buttons, "Task_5", 1000, NULL, 2, &Task5, 0);
    xTaskCreatePinnedToCore(communication, "Task_6", 1000, NULL, 2, &Task6, 0);
    xTaskCreatePinnedToCore(lora, "Task_7", 1000, NULL, 2, &Task7, 0);

    gpio_set_level(DIR, 1);
    gpio_set_level(ESF, 1);
    gpio_set_level(MODE, 0);

    tripCurrent = 15;

    static const int RX_BUF_SIZE = 1024;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN_2, RXD_PIN_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

//***********************************
//************* BUTTONS *************
//***********************************
void buttons(void *parameters)
{
    while (1)
    {
        vdsthPin.setDuty(30);

        bool dReset = resetInterrupt.getState();
        bool dDir = dirInterrupt.getState();
        bool dCoast = coastInterrupt.getState();
        bool dBrake = coastInterrupt.getState();

        printf("DIR: %d  ", dDir);
        printf("COAST: %d  ", dCoast);
        printf("BRAKE: %d  ", dBrake);
        printf("RESET: %d  ", dReset);

        // gpio_set_level(DIR, 1);
        gpio_set_level(COAST, dCoast);
        gpio_set_level(BRAKE, dBrake);
        gpio_set_level(RST, HIGH);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//***********************************
//************ SPEED-DIR ************
//***********************************
void speedDir(void *parameter)
{
    while (1)
    {
        if (tachoInterrupt.available() && dirInterrupt.available())
        {
            bool direction = dirInterrupt.getState();
            float time = tachoInterrupt.getElapsedMicros();
            motorRads = DELTA_ANGLE / (time / 1000000);
            rpm = (motorRads * CONVERSION); //60.0f) / 360.0f;

            gpio_set_level(DIR, direction);

            printf("RPM: %0.2f", rpm);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//***********************************
//************* SENSING *************
//***********************************
void sensing(void *parameter)
{
    while (1)
    {
        csoutDividerVoltage = analogRead(CSOUT); // 3.3V/4096
        printf("CSOUT DIVIDER VOLTAGE: %d", csoutDividerVoltage);

        csoutVoltage = (3.3 * csoutDividerVoltage / 4096) * 3.2 / 2.2; // Divisor de voltaje
        printf("CSOUT: %d", csoutVoltage);

        loadCurrent = (csoutVoltage - VOOS) / (AV * RSENSE);
        printf("LOAD CURRENT: %d", loadCurrent);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//***********************************
//********* CURRENT CONTROL *********
//***********************************
void currentControl(void *parameter)
{
    while (1)
    {
        // int lec = analogRead(POT); // 3.3V/4096
        // int conv = map(lec, 0, 4096, 0, 100);

        refVoltage = tripCurrent * (RSENSE * AV) + VOOS;

        dutyRef = refVoltage * 100 / 3.3;

        if (dutyRef > 100)
            dutyRef = 100;

        pwmPin.setDuty(100); // 100-Torque Control     0-Current recirculate
        refPin.setDuty(dutyRef);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//***********************************
//************** FAULTS *************
//***********************************
void faults(void *parameter)
{
    while (1)
    {
        int l1 = FF1Interrupt.getState();
        int l2 = FF2Interrupt.getState();

        if (l1 == 1 && l2 == 1)
            fault = "No fault";

        else if (l1 == 0 && l2 == 0)
        {
            fault = "Undervoltage, Overtemperature or Logic fault";
            printf("Undervoltage, Overtemperature or Logic fault");
        }
        else if (l1 == 1 && l2 == 0)
        {
            fault = "Short to ground, short to supply or shorted motor winding";
            printf("Short to ground, short to supply or shorted motor winding");
        }
        else if (l1 == 0 && l2 == 1)
        {
            fault = "Low load current";
            printf("Low load current");
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//***********************************
//********** COMMUNICATION **********
//***********************************
void communication(void *parameter)
{
    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

//***********************************
//************** LORA ***************
//***********************************
void lora(void *parameter)
{
    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


































//***********************************
//************** SETUP **************
//***********************************
void setup()
{
    Serial.begin(115200);
}

//***********************************
//*************** LOOP **************
//***********************************
void loop()
{
    ArduinoOTA.handle();

    if (!client.connected())
    {
        reconnect();
    }

    if (client.connected())
    {
        dtostrf(rpm, 4, 3, sendR);
        dtostrf(loadCurrent, 4, 3, sendLC);
        dtostrf(dutyRef, 4, 3, sendDR);
        dtostrf(refVoltage, 4, 3, sendRV);
        dtostrf(csoutVoltage, 4, 3, sendCV);
        fault.toCharArray(sendF, 20);

        client.publish(publish_R, sendR);   // RPM
        client.publish(publish_LC, sendLC); // LOAD CURRENT
        client.publish(publish_DR, sendDR); // DUTY REF_PIN
        client.publish(publish_RV, sendRV); // REF VOLTAGE
        client.publish(publish_CV, sendCV); // CSOUT VOLTAGE
        client.publish(publish_F, sendF);   // FAULTS

        delay(1500);
    }
    client.loop();
}

//***********************************
//*********** MQTT CONNECT **********
//***********************************
void reconnect()
{
    while (!client.connected())
    {
        Serial.print("Intentando conexión MQTT...");

        // Creamos un cliente ID
        String clientId = "Shell";
        clientId += String(random(0xffff), HEX);

        // Conectamos
        if (client.connect(clientId.c_str(), mqttUser, mqttPass))
        {
            Serial.println("Conectado");

            // Nos suscribimos
            if (client.subscribe(subscribe))
            {
                Serial.println("Suscripción ok");
            }

            else
                Serial.println("Fallo en suscripción");
        }

        else
        {
            Serial.print("Falló conexión. Error ->" + String(client.state()));
            Serial.println("Reintentando en 5 segundos");
            delay(5000);
        }
    }
}

//***********************************
//************* CALLBACK ************
//***********************************
void callback(char *topic, byte *payload, unsigned int length)
{
    String incoming = "";
    Serial.println("Mensaje recibido desde ->" + String(topic));

    for (int i = 0; i < length; i++)
    {
        incoming += (char)payload[i];
    }
    incoming.trim();
    Serial.println("Mensaje ->" + incoming);
}