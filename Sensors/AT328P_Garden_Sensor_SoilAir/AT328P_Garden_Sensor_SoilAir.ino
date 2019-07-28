/*
Name:    AT328P_Garden_Sensor_SoilAir.ino
Created: 21/02/2018
Last update: 28/08/2018
Version: 1.1
Hardware: Adafruit 32u4 Feather
Author:  Didier Coyman
MIT License
Copyright (c) 2018 Didier  Coyman
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

#define DEBUG_HUB;

#include <SoftwareSerial.h>
#include <LowPower.h>
#include <CRC32.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include "DFRobot_SHT20.h"

const char devID = '0';
const char hubID = '0';

DFRobot_SHT20 sht20;

const uint8_t MAX_RETRY = 3;
const uint8_t MSGSIZE = 136;
const uint16_t TIMEOUT_ACK = 6000;   // timeout in milli second after sending a message

SoftwareSerial softSerial(4, 5);

// sending status
// bit 0 >> sensor data acquired
// bit 1 >> message ready
// bit 2 >> pending ACK
// bit 3 >> receive ACK
// bit 4 >> fail

#define DHTPIN 3 // what digital pin we're connected to
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
float air_temperature = 0, air_humidity = 0, soil_temperature = 0, soil_humidity = 0;

// the setup function runs once when you press reset or power the board
void setup() {
	// Start the Serial hardware & software
#ifdef DEBUG_HUB  //Debug info
	Serial.begin(115200);
#endif
	softSerial.begin(9600);

	// initialize digital pin LED_BUILTIN as an output.
	pinMode(13, OUTPUT);

	// Set the HC-12 in transmitter mode
	pinMode(2, OUTPUT);
	digitalWrite(2, HIGH);

	// start the sensors
	dht.begin();

	// Init SHT20 Sensor
	sht20.initSHT20();
}

// format the json message
size_t formatJsonMessage(char* payLoad, const uint8_t size)
{
	StaticJsonBuffer<200> jsonBuffer;
	JsonObject &jsonRoot = jsonBuffer.createObject();

	// prepare the json message
	jsonRoot.set<char>("deviceid", devID);
	jsonRoot["devicetype"] = "SoilAirSensor";
	jsonRoot["AirTemp"] = air_temperature;
	jsonRoot["AirHum"] = air_humidity;
	char soilHum[6]; char soilTemp[6];
	jsonRoot["SoilTemp"] = dtostrf(soil_temperature, 3, 1, soilTemp);
	jsonRoot["SoilHum"] = dtostrf(soil_humidity, 3, 1, soilHum);

	char crc32Buffer[128];
	jsonRoot.printTo(crc32Buffer);
	uint32_t checksum = CRC32::calculate(crc32Buffer, strlen(crc32Buffer));
	jsonRoot["CRC32"] = checksum;

	// copy the json message to the buffer
	if (jsonRoot.measureLength() < size - 1)
	{
		return jsonRoot.printTo(payLoad, size);
	}
	else
	{
		// buffer overflow
		return 0;
	}
}

// the loop function runs over and over again until power down or reset
void loop() {
	static uint32_t counterMillis = 0;
	static uint8_t measure_retry, msg_retry, send_retry, sendStatus;
	static int8_t command = 0, channel = 0;
	static uint16_t interval = 10;
	static uint8_t interval_byte = 0;
	static char msgBuffer[MSGSIZE];
	static char recBuf[10] = {0,0,0,0,0,0,0,0,0};
	static uint8_t recpos = 0;
	static size_t msgLength;
	static bool myID = false;
	
	if ((sendStatus == 0) && measure_retry < MAX_RETRY) {
		// get the DHT22 sensor data, reading interval need to be bigger than 2 sec
		// Reading temperature or humidity takes about 250 milliseconds!
		air_humidity = dht.readHumidity();
		// Read temperature as Celsius (the default)
		delay(2500);
		air_temperature = dht.readTemperature();
		
		// get the SHT20 sensor data
		soil_humidity = sht20.readHumidity();
		soil_temperature = sht20.readTemperature();
		
		if (isnan(air_humidity) || isnan(air_temperature) || isnan(soil_humidity) || isnan(soil_temperature))
		{
			measure_retry++;
			softSerial.println("Sensor error.");
			if (measure_retry > 2)
			{
				sendStatus = 16;
				measure_retry = 0;
			}
		}
		else
		{
			sendStatus = 1;
			measure_retry = 0;
		}
	}
	
	// format the message once per measure
	if ((sendStatus == 1) && (send_retry == 0) && (msg_retry < MAX_RETRY))
	{
		// prepare the json message

		msgLength = formatJsonMessage(msgBuffer, MSGSIZE);
		if (msgLength > 0)
		{
			sendStatus = 2;
			msg_retry = 0;
		}
		// message invalid
		else
		{
			msg_retry++;
			if (msg_retry == MAX_RETRY)
			{
				sendStatus = 16;
				softSerial.println("Message invalid");
			}
		}
	}

	// send the message and wait for the reply, retry if not successful
	if ((sendStatus == 2) && (send_retry < MAX_RETRY)) {
		// start the time counter
		counterMillis = millis();
		// format the message
		softSerial.write(2);
		softSerial.print(msgLength);
		softSerial.print(":");
		softSerial.write(devID);
		softSerial.print(":");
		softSerial.write(hubID);
		softSerial.write(3);
		softSerial.write(msgBuffer);
		softSerial.println();
#ifdef DEBUG_HUB  //Debug info
		Serial.println(msgBuffer);
#endif
		// pending acknowledgment
		sendStatus = 4;

		// try to prevent buffer full
		softSerial.flush();

		// increase the retry counter
		send_retry++;

		// delay the last transmission for completness
		if (send_retry == MAX_RETRY) delay(50);
	}

	// once the message is send, wait for the acknowledgment or timeout
	if (sendStatus == 4) 
	{
		if ((millis() - counterMillis) < TIMEOUT_ACK) {
			if (softSerial.available())
			{
				uint8_t input = softSerial.read();
#ifdef DEBUG_HUB  //Debug info
				Serial.print("received character ");
				Serial.print(recpos);
				Serial.print(" = ");
				Serial.println(input, DEC);
#endif			
				if (input != 10)
				{
					recBuf[recpos] = input;
					if (recpos < 9) recpos++;
				}
				else
				{
					// process the received buffer
					recBuf[recpos] = 0;
					recpos = 0;
#ifdef DEBUG_HUB  //Debug info
					Serial.print("Received Buffer = ");
					Serial.println(recBuf);
#endif
					// check if the message is for me
					if (recBuf[0] == devID)
					{
#ifdef DEBUG_HUB  //Debug info
						Serial.println("Message for me...");
#endif
						switch (recBuf[1])
						{
						case 6:  // ACK
							sendStatus = 8;
							send_retry = 0;
							break;
						case 21: // NACK
							sendStatus = 2;
							break;
						case 17: // set interval
							recBuf[0] = 32; recBuf[1] = 32;
							interval = atoi(recBuf);
#ifdef DEBUG_HUB  //Debug info
							Serial.print("interval=");
							Serial.println(interval);
#endif
							break;
						case 18: // set channel
							recBuf[0] = 32; recBuf[1] = 32;
							uint8_t desiredChannel = atoi(recBuf);
#ifdef DEBUG_HUB  //Debug info
							Serial.print("Channel = ");
							Serial.println(desiredChannel);
#endif
							if (channel != desiredChannel)
							{
								channel = desiredChannel;
								// set the HC-12 channel by entering AT command AT+Cxxx followed by exiting the AT mode
								//digitalWrite(2, LOW);
								// 50 ms are required for the HC-12 to change mode
								delay(50);
								char c[4];
								sprintf(c, "AT+C%03u", input);
#ifdef DEBUG_HUB  //Debug info
								Serial.println(c);
#endif
								//softSerial.println(c);
								delay(50);
								//digitalWrite(2, HIGH);
								delay(50);
							}
						}
					}
				}	
			}
		}
		// flag time out
		else
		{
			sendStatus = 2;
#ifdef DEBUG_HUB  //Debug info
			Serial.println("ACK time out.");
#endif
		}
	}

	// get to sleep for saving energy if the message has been received or time out
	if ((sendStatus > 4) || (send_retry >= MAX_RETRY)) {
		// set the HC-12 to sleep by entering AT command AT+SLEEP followed by exiting the AT mode
		digitalWrite(2, LOW);
		// stop listening
		softSerial.stopListening();
		// 50 ms are required for the HC-12 to change mode
		delay(50);
		softSerial.println("AT+SLEEP");
		delay(50);
		digitalWrite(2, HIGH);
		delay(50);

#ifdef DEBUG_HUB  //Debug info
		Serial.print("Sleeping for ");
		Serial.print(interval * 8);
		Serial.println(" sec");
		delay(25);
#endif
		// Set the 328P into sleep mode for around 60 min (422*8s)
		for (uint16_t i = 0; i < interval; i++)
		{
			// Enter power down state for 8 s with ADC and BOD module disabled
			LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
		}
		// reset to start a new measure point
		sendStatus = 0;
		send_retry = 0;

		// wake up the HC-12
		digitalWrite(2, LOW);
		delay(10);
		digitalWrite(2, HIGH);
		// start listening again
		softSerial.listen();
#ifdef DEBUG_HUB  //Debug info
		Serial.println("Awake");
#endif
	}
}
