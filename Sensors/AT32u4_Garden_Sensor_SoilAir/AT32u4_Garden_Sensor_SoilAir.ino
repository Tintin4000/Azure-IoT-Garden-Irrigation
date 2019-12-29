/*
 Name:		AT32u4_Garden_Sensor_SoilAir.ino
 Created:	7/28/2019 11:59:31 PM
Last update: 28/12/2019
Version: 1.2
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
#include <DHT.h>
#include <DFRobot_SHT20.h>

#define VBATPIN A9
#define RSTPIN A0
#define HC12RST A3
#define HC12CUR A1
#define HC12SETPIN A2

constexpr char devID = '2';
constexpr char hubID = '0';
constexpr char* location = "MainFrontGarden";
constexpr char* devicetype = "SoilAirSensor";

/* Sensor previous status 0 = OK
                          1 = Sensor error
						  2 = Sleep issue
						  3 = Reboot
*/
uint8_t sensorPreviousStatus = 0;

DFRobot_SHT20 sht20;

const uint8_t MAX_RETRY = 3;
const uint8_t MSGSIZE = 200;
// CRC buffer max size
const uint8_t MAX_CRC_BUFFER_SIZE = 180;
const uint8_t JSONMSG = 220;
const uint16_t TIMEOUT_ACK = 3000;   // timeout in milli second after sending a message

const char atSleep[] = { 'O','K','+','S','L','E','E','P','\r','\n'};

// Arg receive, transmit
SoftwareSerial softSerial(11, 10);

// sending status
// bit 0 >> sensor data acquired
// bit 1 >> message ready
// bit 2 >> pending ACK
// bit 3 >> receive ACK
// bit 4 >> fail

#define DHTPIN 12 // what digital pin we're connected to
#define DHTTYPE DHT22

DHT dht22(DHTPIN, DHTTYPE);
float air_temperature = 0, air_humidity = 0, soil_temperature = 0, soil_humidity = 0, bat_voltage = 0;

// the setup function runs once when you press reset or power the board
void setup() {
#ifdef DEBUG_HUB  //Debug info
	softSerial.begin(9600);
	softSerial.println("** Setup **");
#endif
	// Start the Serial hardware & software
	Serial1.begin(9600);

	// Set the HC-12 in transmitter mode
	digitalWrite(RSTPIN, HIGH);
	digitalWrite(HC12RST, HIGH);
	pinMode(RSTPIN, OUTPUT);
	pinMode(HC12RST, OUTPUT);
	pinMode(HC12SETPIN, OUTPUT);
	pinMode(HC12CUR, INPUT);
	digitalWrite(HC12SETPIN, HIGH);

	// start the sensors
	dht22.begin();

	// Init SHT20 Sensor
	sht20.initSHT20();
}

// format the json message
size_t formatJsonMessage(char* payLoad, const uint8_t size)
{
	StaticJsonDocument<JSONMSG> jsonDoc;

	// prepare the json message
	jsonDoc["deviceid"] = devID;
	jsonDoc["location"] = location;
	jsonDoc["devicetype"] = devicetype;
	char tempBuf[6];
	jsonDoc["AirTemp"] = dtostrf(air_temperature, 3, 1, tempBuf);
	jsonDoc["AirHum"] = dtostrf(air_humidity, 3, 1, tempBuf);
	jsonDoc["SoilTemp"] = dtostrf(soil_temperature, 3, 1, tempBuf);
	jsonDoc["SoilHum"] = dtostrf(soil_humidity, 3, 1, tempBuf);
	jsonDoc["BattVoltage"] = bat_voltage; //dtostrf(bat_voltage, 3, 2, tempBuf);
	jsonDoc["Status"] = sensorPreviousStatus;

#ifdef DEBUG_HUB  //Debug info
	softSerial.println("Json without CRC32.");
	serializeJson(jsonDoc, softSerial);
	softSerial.println();
#endif

	char crc32Buffer[MAX_CRC_BUFFER_SIZE];
	serializeJson(jsonDoc, crc32Buffer);
	uint32_t checksum = CRC32::calculate(crc32Buffer, strlen(crc32Buffer));
	jsonDoc["CRC32"] = checksum; 

#ifdef DEBUG_HUB  //Debug info
	softSerial.println("Json with CRC32.");
	softSerial.print("Json length = ");
	softSerial.println(measureJson(jsonDoc));
	serializeJson(jsonDoc, softSerial);
	softSerial.println();
#endif

	// copy the json message to the buffer
	if (measureJson(jsonDoc) < size - 1)
	{
		return serializeJson(jsonDoc, payLoad, size);
	}
	else
	{
		// buffer overflow
		return 0;
	}
}

bool SetHC12ToSleep()
{
	// set the HC-12 to sleep by entering AT command AT+SLEEP followed by exiting the AT mode
	digitalWrite(HC12SETPIN, LOW);
	// 50 ms are required for the HC-12 to change mode
	delay(50);
	Serial1.println("AT+SLEEP");
	delay(200);
	// read the response from the HC-12
	String hc12Response = "";
	char hc12Receive;
	while (Serial1.available())
	{
		hc12Receive = Serial1.read();
		hc12Response += char(hc12Receive);
		if (hc12Receive == '\n') exit;
	}
	digitalWrite(HC12SETPIN, HIGH);
	delay(25);

#ifdef DEBUG_HUB  //Debug info
	softSerial.print("HC-12 response 200 = ");
	softSerial.println(hc12Response);
	softSerial.print("HC12 current = ");
	softSerial.println(analogRead(HC12CUR) * 3);
#endif

	// test if the HC12 is sleeping by measuring the drain current
	if (analogRead(HC12CUR) == 0) return 1;
	else return 0;
}

// the loop function runs over and over again until power down or reset
void loop() {
	static uint32_t counterMillis = 0;
	static uint8_t measure_retry, msg_retry, send_retry, sendStatus;
	static int8_t command = 0, channel = 0;
	static uint16_t interval = 79;
	static uint8_t interval_byte = 0;
	static char msgBuffer[MSGSIZE];
	static char recBuf[10] = { 0,0,0,0,0,0,0,0,0 };
	static uint8_t recpos = 0;
	static size_t msgLength;
	static bool myID = false;

#ifdef DEBUG_HUB  //Debug info
	softSerial.print("Status = ");
	softSerial.println(sendStatus);
#endif

	if ((sendStatus == 0) && measure_retry < MAX_RETRY) {
		counterMillis = millis();
		// wake up the 
		// get the DHT22 sensor data, reading interval need to be bigger than 2 sec
		// Reading temperature or humidity takes about 250 milliseconds!
		air_humidity = dht22.readHumidity();
		// Read temperature as Celsius (the default)
		air_temperature = dht22.readTemperature();
		// get the SHT20 sensor data
		soil_humidity = sht20.readHumidity();
		soil_temperature = sht20.readTemperature();
		// measure the voltage of the battery, add a pre read and a delay to stabilize the reading
		analogRead(VBATPIN);
		delay(10);
		bat_voltage = analogRead(VBATPIN);
		bat_voltage *= 2;    // divided by 2 on the board, so multiply back
		bat_voltage *= 3.3;  // Multiply by 3.3V, the reference voltage
		bat_voltage /= 1024; // convert to voltage

		if (isnan(air_humidity) || isnan(air_temperature) || isnan(soil_humidity) || isnan(soil_temperature))
		{
			measure_retry++;
			Serial1.println("Sensor error.");
			sensorPreviousStatus = 1;
			if (measure_retry > 2)
			{
				sendStatus = 16;
				measure_retry = 0;
			}
		}
		else
		{
#ifdef DEBUG_HUB  //Debug info
			softSerial.print("Elapse time for sensor measure = ");
			softSerial.println(millis() - counterMillis);
			softSerial.print("Air Temp = ");
			softSerial.println(air_temperature);
			softSerial.print("Air Humidity = ");
			softSerial.println(air_humidity);
			softSerial.print("Soil Temp = ");
			softSerial.println(soil_temperature);
			softSerial.print("Soil Humidity = ");
			softSerial.println(soil_humidity);
			softSerial.print("Battery Voltage = ");
			softSerial.println(bat_voltage);
#endif
			sendStatus = 1;
			measure_retry = 0;
		}
	}

	// format the message once per measure
	if ((sendStatus == 1) && (send_retry == 0) && (msg_retry < MAX_RETRY))
	{
		counterMillis = millis();
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
				Serial1.println("Message invalid");
			}
		}
#ifdef DEBUG_HUB  //Debug info
		softSerial.print("Elapse time for formating message = ");
		softSerial.println(millis() - counterMillis);
#endif
	}

	// send the message and wait for the reply, retry if not successful
	if ((sendStatus == 2) && (send_retry < MAX_RETRY)) {
		// start the time counter
		counterMillis = millis();
		// format the message
		Serial1.write(2);
		Serial1.print(msgLength);
		Serial1.print(":");
		Serial1.write(devID);
		Serial1.print(":");
		Serial1.write(hubID);
		Serial1.write(3);
		Serial1.write(msgBuffer);
		Serial1.println();
		// reset status
		sensorPreviousStatus = 0;

#ifdef DEBUG_HUB  //Debug info
		softSerial.print("HC12 current = ");
		softSerial.println(analogRead(HC12CUR)*3);
		softSerial.write(2);
		softSerial.print(msgLength);
		softSerial.print(":");
		softSerial.write(devID);
		softSerial.print(":");
		softSerial.write(hubID);
		softSerial.write(3);
		softSerial.println(msgBuffer);
#endif
		// pending acknowledgment
		sendStatus = 4;

		// increase the retry counter
		send_retry++;
	}

	// once the message is send, wait for the acknowledgment or timeout
	if (sendStatus == 4)
	{
#ifdef DEBUG_HUB  //Debug info
		softSerial.print("Elapse time since message sent = ");
		softSerial.println(millis() - counterMillis);
#endif
		if ((millis() - counterMillis) < TIMEOUT_ACK) {
			if (Serial1.available())
			{
				uint8_t input = Serial1.read();
#ifdef DEBUG_HUB  //Debug info
				softSerial.print("received character ");
				softSerial.print(recpos);
				softSerial.print(" = ");
				softSerial.println(input, DEC);
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
					softSerial.print("Received Buffer = ");
					softSerial.println(recBuf);
#endif
					// check if the message is for me
					if (recBuf[0] == devID)
					{
#ifdef DEBUG_HUB  //Debug info
						softSerial.println("Message for me...");
#endif
						switch (recBuf[1])
						{
						case 6:  // ACK
							sendStatus = 8;
							send_retry = 0;
							break;
						case 21: // NACK
							sendStatus = 2;
							if (send_retry >= MAX_RETRY) sendStatus = 16;
							break;
						case 22: // Reset HC-12
							sensorPreviousStatus = 2;
							digitalWrite(HC12RST, LOW);
							delay(200);
							digitalWrite(HC12RST, HIGH);
						case 24: // Reset device
							sensorPreviousStatus = 3;
							digitalWrite(RSTPIN, LOW);
						case 17: // set interval
							recBuf[0] = 32; recBuf[1] = 32;
							interval = atoi(recBuf);
#ifdef DEBUG_HUB  //Debug info
							softSerial.print("interval=");
							softSerial.println(interval);
#endif
							break;
						case 18: // set channel
							recBuf[0] = 32; recBuf[1] = 32;
							uint8_t desiredChannel = atoi(recBuf);
#ifdef DEBUG_HUB  //Debug info
							softSerial.print("Channel = ");
							softSerial.println(desiredChannel);
#endif
							if (channel != desiredChannel)
							{
								channel = desiredChannel;
								// set the HC-12 channel by entering AT command AT+Cxxx followed by exiting the AT mode
								digitalWrite(HC12SETPIN, LOW);
								// 50 ms are required for the HC-12 to change mode
								delay(50);
								char c[4];
								sprintf(c, "AT+C%03u", input);
#ifdef DEBUG_HUB  //Debug info
								softSerial.println(c);
#endif
								//Serial1.println(c);
								delay(50);
								digitalWrite(HC12SETPIN, HIGH);
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
			if (send_retry >= MAX_RETRY) sendStatus = 16;
#ifdef DEBUG_HUB  //Debug info
			softSerial.println("ACK time out.");
#endif
		}
	}

	// get to sleep for saving energy if the message has been received or time out
	if ((sendStatus > 4) || (send_retry > MAX_RETRY)) {
		// set the HC-12 to sleep with a recovery safe
		if (!SetHC12ToSleep())
		{
			sensorPreviousStatus = 2;
			digitalWrite(HC12RST, LOW);
			delay(200);
			digitalWrite(HC12RST, HIGH);
			if (!SetHC12ToSleep())
			{
				sensorPreviousStatus = 3;
				digitalWrite(RSTPIN, LOW);
			}
		}
#ifdef DEBUG_HUB  //Debug info
		softSerial.print("Sleeping for ");
		softSerial.print(interval * 8);
		softSerial.println(" sec");
		delay(25);
#endif
		// Set the 32u4 into sleep mode
		for (uint16_t i = 0; i < interval; i++)
		{
			// Enter power down state for 8 s with ADC and BOD module disabled
			LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
		}
		// reset to start a new measure point
		sendStatus = 0;
		send_retry = 0;

		// wake up the HC-12
		digitalWrite(HC12SETPIN, LOW);
		delay(10);
		digitalWrite(HC12SETPIN, HIGH);
#ifdef DEBUG_HUB  //Debug info
		softSerial.println("Awake");
#endif
	}
}