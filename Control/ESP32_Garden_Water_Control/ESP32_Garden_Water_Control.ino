/*
 Name:		ESP32_Garden_Water_Control.ino
 Created:	22 June 2020
 Author:	Didier Coyman
 MIT License
 Copyright (c) 2020 Didier  Coyman
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

#define ARDUINOJSON_ENABLE_STD_STREAM 0
// #define DONT_USE_UPLOADTOBLOB

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOta.h>
#include <BlynkSimpleEsp32_SSL.h>
#include <ArduinoJson.h>
#include <AzureIoTHub.h>
#include "AzureIoTProtocol_MQTT.h"
#include "iothubtransportmqtt.h"
#include "Token.h"
#include "FormatTime.h"
#include "driver/pcnt.h"


#define DEBUG_SERIAL;

#define PCNT_UNIT0_CH1      PCNT_UNIT_1
//#define PCNT_THRESH0_VAL    5
#define PCNT_THRESH1_VAL    10000
#define PCNT_L_LIM_VAL		-553
#define PCNT_H_LIM_VAL		16000


constexpr uint8_t VALVE_PIN_ENABLE = 22;
constexpr uint8_t VALVE_PIN_I1A = 23;
constexpr uint8_t VALVE_PIN_I2A = 21;
constexpr uint8_t VALVE_PULSE_WIDTH = 40;
constexpr uint8_t VALVE_LED = 32;
constexpr uint8_t SIGN_PULSECOUNTER_GPIO = 19;
constexpr uint8_t CTRL_PULSECOUNTER_GPIO = 18;


// create a timer
BlynkTimer blynktimer;

// NTP settings
WiFiUDP ntpUDP;

// Initialize the time format class
FormatTime udpTime;

// quantity of water used
float fquantity;

// WiFi event
void WiFiEvent(WiFiEvent_t event)
{
#ifdef DEBUG_SERIAL  //Debug info
	Serial.printf("[WiFi-event] event: %d - Connected: %d\n", event, WiFi.isConnected());
#endif
	if (event == SYSTEM_EVENT_STA_GOT_IP)
	{
		// using time routine to get the time from ntp, this is a must to have for the azureIot SDK to send message
		uint8_t counter = 0;
		time_t epochTime = 0;
		configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
		while ((epochTime == 0 || epochTime < 1000000) && counter < 100) {
			counter++;
			delay(2000);
			epochTime = time(NULL);
		}
#ifdef DEBUG_SERIAL  //Debug info
		Serial.print("MAC address : ");
		Serial.println(WiFi.macAddress());
		Serial.print("IP address : ");
		Serial.println(WiFi.localIP());
		Serial.printf("Epoch = %u, counter = %u\n", epochTime, counter);
		char dateTime[25];
		udpTime.getFormattedTimeISO8601(dateTime, 25);
		Serial.printf("GMT   = %s\n", dateTime);
		udpTime.getFormattedTime(dateTime, 25);
		Serial.printf("Local = %s\n", dateTime);
#endif
		// start OTA
		ArduinoOTA.begin();

		// start or restart Blynk connection
		Blynk.config(auth);
		//Blynk.connect();

		// indiciate the connection to the internet
		digitalWrite(BUILTIN_LED, HIGH);
	}
	if (event == SYSTEM_EVENT_STA_DISCONNECTED)
	{
		WiFi.begin(mySSID, myPassword);
	}
}


BLYNK_WRITE(V0)
{
	const TickType_t xTicksToWait = pdMS_TO_TICKS(VALVE_PULSE_WIDTH);
	int valveSwitch = param.asInt();

#ifdef DEBUG_SERIAL  //Debug info
	Serial.printf("V0 change state (%d)\n", valveSwitch);
#endif


	if (valveSwitch == 1)
	{
		blynktimer.enableAll();
		digitalWrite(VALVE_PIN_I1A, HIGH);
		digitalWrite(VALVE_PIN_I2A, LOW);
		digitalWrite(VALVE_PIN_ENABLE, HIGH);
		vTaskDelay(xTicksToWait);
		digitalWrite(VALVE_PIN_ENABLE, LOW);
		digitalWrite(VALVE_LED, HIGH);
	}
	else
	{
		digitalWrite(VALVE_PIN_I1A, LOW);
		digitalWrite(VALVE_PIN_I2A, HIGH);
		digitalWrite(VALVE_PIN_ENABLE, HIGH);
		vTaskDelay(xTicksToWait);
		digitalWrite(VALVE_PIN_ENABLE, LOW);
		digitalWrite(VALVE_LED, LOW);
		// stop counting the flow
		blynktimer.disableAll();
		// add the residual quantity of water
		onBlynkTimer;
		// set the flow to 0
		onBlynkTimer;
	}
}

void onBlynkTimer()
{
	static int16_t rotation;
	static float flow;

	pcnt_get_counter_value(PCNT_UNIT0_CH1, &rotation);
	pcnt_counter_clear(PCNT_UNIT0_CH1);
	flow = rotation / 553.0;
	fquantity += flow;

	Blynk.virtualWrite(V1, flow);
	Blynk.virtualWrite(V2, fquantity);
}

static void PulseCounterInitialisation(void)
{
	// prepare the configuration for the PCNT
	pcnt_config_t pcntConfig;
	pcntConfig.pulse_gpio_num = SIGN_PULSECOUNTER_GPIO;
	pcntConfig.ctrl_gpio_num = CTRL_PULSECOUNTER_GPIO;
	pcntConfig.channel = PCNT_CHANNEL_0;
	pcntConfig.unit = PCNT_UNIT0_CH1;
	// What to do on the positive / negative edge of pulse input?
	pcntConfig.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
	pcntConfig.neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
	// What to do when control input is low or high?
	pcntConfig.lctrl_mode = PCNT_MODE_REVERSE; // Reverse counting direction if low
	pcntConfig.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
	// Set the maximum and minimum limit values to watch
	pcntConfig.counter_h_lim = PCNT_H_LIM_VAL;
	pcntConfig.counter_l_lim = PCNT_L_LIM_VAL;

	/* Initialize PCNT unit */
	pcnt_unit_config(&pcntConfig);

	/* Configure and enable the input filter */
	pcnt_filter_disable(PCNT_UNIT0_CH1);

	/* Initialize PCNT's counter */
	pcnt_counter_pause(PCNT_UNIT0_CH1);
	pcnt_counter_clear(PCNT_UNIT0_CH1);

	/* Everything is set up, now go to counting */
	pcnt_counter_resume(PCNT_UNIT0_CH1);
}


// the setup function runs once when you press reset or power the board
void setup()
{
#ifdef DEBUG_SERIAL  //Debug info
	Serial.begin(115200);
	Serial.printf("\nInitializing on core %d\n", xPortGetCoreID());
	Serial.printf("Total heap memory = %d\n", ESP.getHeapSize());
	Serial.printf("Available heap memory = %d\n", ESP.getFreeHeap());
#endif

	// define the GPIO that controls the valve
	pinMode(VALVE_PIN_ENABLE, OUTPUT);
	pinMode(VALVE_PIN_I1A, OUTPUT);
	pinMode(VALVE_PIN_I2A, OUTPUT);
	pinMode(VALVE_LED, OUTPUT);
	pinMode(SIGN_PULSECOUNTER_GPIO, INPUT);
	pinMode(CTRL_PULSECOUNTER_GPIO, INPUT);
	pinMode(BUILTIN_LED, OUTPUT);


	// activate the WiFi events
	WiFi.onEvent(WiFiEvent);

	// set WiFi mode to Station only and enable event
	WiFi.mode(WIFI_STA);
	WiFi.begin(mySSID, myPassword);

	// configure the internal counter
	PulseCounterInitialisation();

	blynktimer.setInterval(3000, onBlynkTimer);
	blynktimer.disableAll();

	// set the OTA parameters
	ArduinoOTA.setHostname("GardenController1");
	ArduinoOTA.setPassword(myOTAPassword);
	//ArduinoOTA.setPort(3232);

	// declare OTA event
	ArduinoOTA
		.onStart([]() {
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

}

// the loop function runs over and over again until power down or reset
void loop() {
	const TickType_t xTicksToWait = pdMS_TO_TICKS(10);

	// manage Blynk update
	Blynk.run();
	blynktimer.run();

	// manage the OTA update
	ArduinoOTA.handle();

	vTaskDelay(xTicksToWait);
}
