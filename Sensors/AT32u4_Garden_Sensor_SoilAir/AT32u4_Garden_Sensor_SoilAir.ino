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

#ifdef DEBUG_HUB  //Debug info
#include <SoftwareSerial.h>
#endif
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

// sensor ID
constexpr uint8_t mySensorID = 1;
// hub id
constexpr char* myHubID = "ESP32HUB1";
// location
constexpr char* location = "FrontGarden";
//sensor type
constexpr char* devicetype = "SoilAirSensor";

// Sensor previous status
enum sensorStatus
{
	OK,
	SENSOR_ERROR,
	HC12ERROR,
	REBOOT,
	BAD_TRANSMISSION
};
uint8_t sensorPreviousStatus = sensorStatus::OK;

// number of Nack or no response
uint8_t noResponse = 0;

// last known HC-12 current with feedback
uint16_t lastHC12Current = 0;

// declare the class library for the SHT20 sensor
DFRobot_SHT20 sht20;

// number of time the sensor will retry to send the telemetry without acknowledgement
constexpr uint8_t MAX_RETRY = 3;
// maximum size of the text buffer
constexpr uint8_t MSG_SIZE = 210;
// CRC buffer max size
constexpr uint8_t MAX_CRC_BUFFER_SIZE = MSG_SIZE - 18;
// JSON memory pool size
constexpr uint8_t JSON_MSG = 245;
// JSON feedback memeory pool size
constexpr uint8_t JSON_FEEDBACK = 100;
// acknowlegement time out
uint16_t lTimeOut = 2000;   // timeout in millis second after sending a message

// Arg receive, transmit
#ifdef DEBUG_HUB  //Debug info
SoftwareSerial softSerial(11, 10);
#endif

#define DHTPIN 12 // what digital pin we're connected to
#define DHTTYPE DHT22

DHT dht22(DHTPIN, DHTTYPE);
float air_temperature = 0, air_humidity = 0, soil_temperature = 0, soil_humidity = 0, bat_voltage = 0;

// the setup function runs once when you press reset or power the board
void setup() {
#ifdef DEBUG_HUB  //Debug info
	softSerial.begin(56000);
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
	StaticJsonDocument<JSON_MSG> jsonDoc;

	// prepare the json message
	jsonDoc["SensorID"] = mySensorID;
	jsonDoc["HubID"] = myHubID;
	jsonDoc["Location"] = location;
	jsonDoc["DeviceType"] = devicetype;
	char tempBuf[6];
	jsonDoc["AirTemp"] = dtostrf(air_temperature, 3, 1, tempBuf);
	jsonDoc["AirHum"] = dtostrf(air_humidity, 3, 1, tempBuf);
	jsonDoc["SoilTemp"] = dtostrf(soil_temperature, 3, 1, tempBuf);
	jsonDoc["SoilHum"] = dtostrf(soil_humidity, 3, 1, tempBuf);
	jsonDoc["BattVoltage"] = dtostrf(bat_voltage, 3, 2, tempBuf);
	jsonDoc["Status"] = sensorPreviousStatus;

	char crc32Buffer[MAX_CRC_BUFFER_SIZE];
	serializeJson(jsonDoc, crc32Buffer);
	uint32_t checksum = CRC32::calculate(crc32Buffer, strlen(crc32Buffer));
	jsonDoc["CRC32"] = checksum;

#ifdef DEBUG_HUB  //Debug info
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

//
bool SendATCommand(const char* atcmd)
{
#ifdef DEBUG_HUB  //Debug info
	softSerial.print("HC-12 command = ");
	softSerial.println(atcmd);
#endif
	// set the HC-12 command by entering AT command, followed by exiting the AT mode
	digitalWrite(HC12SETPIN, LOW);
	// a min of 50 ms are required for the HC-12 to change mode
	delay(400);
	Serial1.println(atcmd);

	// set a response time out
	uint32_t counterMillis = millis();

	// read the response from the HC-12
	String hc12Response = "";
	char hc12Receive;
	for (;;)
	{
		if (Serial1.available())
		{
			hc12Receive = Serial1.read();
			hc12Response += char(hc12Receive);
			if (hc12Receive == '\r') break;
		}
		if ((millis() - counterMillis) > 100)
		{
			break;
		}
	}
#ifdef DEBUG_HUB  //Debug info
	softSerial.print("HC-12 command response time =  ");
	softSerial.println(millis() - counterMillis);
	softSerial.print("HC-12 response = ");
	softSerial.println(hc12Response);
#endif
	digitalWrite(HC12SETPIN, HIGH);

	if (hc12Response != "") return 1;
	else return 0;
}

bool SetHC12ToSleep()
{
	SendATCommand("AT+SLEEP");
#ifdef DEBUG_HUB  //Debug info
	softSerial.print("HC12 current = ");
	softSerial.println(analogRead(HC12CUR) * 3);
#endif

	// test if the HC12 is sleeping by measuring the drain current
	if (analogRead(HC12CUR) == 0) return 1;
	else return 0;
}

void RestartSensor()
{
	char jsonErrorMsg[42];
	sprintf(jsonErrorMsg, "{\"SensorID\":%u,\"HubID\":%s,\"Status\":%u}", mySensorID, myHubID, sensorStatus::REBOOT);
	Serial1.println(jsonErrorMsg);
	delay(500);
	digitalWrite(RSTPIN, LOW);
}

// the loop function runs over and over again until power down or reset
void loop() {
	static uint32_t counterMillis = 0;
	static uint8_t measure_retry, msg_retry, send_retry, sendStatus, prevsendStatus;
	static int8_t command = 0, channel = 0;
	static uint16_t interval = 38;
	static uint8_t interval_byte = 0;
	static char msgBuffer[MSG_SIZE];
	static char recBuf[JSON_FEEDBACK];
	static uint8_t rec;
	static uint8_t recpos;
	static size_t msgLength;
	static bool startFeedback;
	static uint16_t hc12Current;

#ifdef DEBUG_HUB  //Debug info
	if ((!sendStatus) || (sendStatus != prevsendStatus))
	{
		softSerial.print("Status = ");
		softSerial.println(sendStatus);
		prevsendStatus = sendStatus;
	}
#endif

	if ((sendStatus == 0) && measure_retry < MAX_RETRY) {
		counterMillis = millis();
		// get the DHT22 sensor data, reading interval need to be bigger than 2 sec
		// Reading temperature or humidity takes about 250 milliseconds!
		air_humidity = dht22.readHumidity();
		// Read temperature as Celsius (the default)
		air_temperature = dht22.readTemperature();
		delay(50);
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
			sensorPreviousStatus = sensorStatus::SENSOR_ERROR;
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
		counterMillis = millis();
		// prepare the json message
		msgLength = formatJsonMessage(msgBuffer, MSG_SIZE);
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
	}

	// send the message
	if ((sendStatus == 2) && (send_retry < MAX_RETRY)) {
		// start the time counter
		counterMillis = millis();
		// post the message to the tranceiver
		Serial1.write(msgBuffer);
		Serial1.println();
		// reset status
		sensorPreviousStatus = sensorStatus::OK;
		// store the current used by the HC-12 to transmit
		hc12Current = analogRead(HC12CUR) * 3;
#ifdef DEBUG_HUB  //Debug info
		softSerial.print("HC12 current = ");
		softSerial.println(hc12Current);
#endif
		// test if the current used by the HC-12 has vary a lot, indicating that the module need a reset
		if (hc12Current < (lastHC12Current / 2))
		{
			sensorPreviousStatus = sensorStatus::HC12ERROR;
			digitalWrite(HC12RST, LOW);
			delay(200);
			digitalWrite(HC12RST, HIGH);
#ifdef DEBUG_HUB  //Debug info
			softSerial.print("HC12 current = ");
			softSerial.println(analogRead(HC12CUR) * 3);
#endif
		}
		// pending acknowledgment
		sendStatus = 4;
		startFeedback = false;

		// increase the retry counter
		send_retry++;
	}

	// once the message is send, wait for the acknowledgment or timeout
	if (sendStatus == 4)
	{
		if ((millis() - counterMillis) < lTimeOut) {
			while (Serial1.available())
			{
				// avoid to overrun the buffer size
				if (recpos < JSON_FEEDBACK)
				{
					// read and store in the buffer the serial character received on the serial port
					rec = Serial1.read();
					// eliminate everything in the received buffer before the json message starts
					if (!startFeedback)
					{
						recpos = 0;
						if (rec == '{')
						{
							startFeedback = true;
						}
					}
					if (startFeedback)
					{
						recBuf[recpos] = rec;
						recpos++;
					}
					// check if the last two characters are CRLF, indicating the end of a message
					if ((recpos) && (recBuf[recpos - 2] == '\r') && (recBuf[recpos - 1] == '\n'))
					{
						// replace the CRLN by 0
						recBuf[recpos - 2] = '\0'; recBuf[recpos - 1] = '\0';
						// reset the buffer
						startFeedback = false;
#ifdef DEBUG_HUB  //Debug info
						softSerial.print("Feedback message = ");
						softSerial.println(recBuf);
#endif
						// deserialize the json feedback message
						StaticJsonDocument<JSON_FEEDBACK> jsonFeedback;
						DeserializationError err = deserializeJson(jsonFeedback, recBuf);
						if (!err)
						{
							if (jsonFeedback["SensorID"] == mySensorID)
							{
#ifdef DEBUG_HUB  //Debug info
								softSerial.print("Elapse time since message sent = ");
								softSerial.println(millis() - counterMillis);
#endif
								// Ack received from hub
								if (jsonFeedback["Ack"] == 6)
								{
									sendStatus = 8;
									send_retry = 0;
									lastHC12Current = hc12Current;
								}
								// Nack received from hub
								if (jsonFeedback["Ack"] == 21)
								{
									sendStatus = 2;
									if (send_retry >= MAX_RETRY) sendStatus = 16;
								}
								// iterate through the properties
								JsonObject properties = jsonFeedback["Properties"];
								for (JsonPair kvProp : properties)
								{
#ifdef DEBUG_HUB  //Debug info
									softSerial.print("Property = ");
									softSerial.println(kvProp.key().c_str());
#endif
									if (kvProp.key() == "Sleep")
									{
										interval = kvProp.value().as<int>();
#ifdef DEBUG_HUB  //Debug info
										softSerial.print("Interval = ");
										softSerial.println(interval);
#endif
									}
									if (kvProp.key() == "Channel")
									{
										uint8_t desiredChannel = kvProp.value();
#ifdef DEBUG_HUB  //Debug info
										softSerial.print("Channel = ");
										softSerial.println(desiredChannel);
#endif
										if (channel != desiredChannel)
										{
											channel = desiredChannel;
											// set the HC-12 channel by entering AT command AT+Cxxx followed by exiting the AT mode
											char c[8];
											sprintf(c, "AT+C%03u", desiredChannel);
											SendATCommand(c);
										}
									}
									if (kvProp.key() == "Power")
									{
										char c[6];
										int power = kvProp.value();
										if ((power > 0) && (power < 9))
										{
											sprintf(c, "AT+P%u", power);
											SendATCommand(c);
										}
									}
									if (kvProp.key() == "Restart")
									{
										digitalWrite(RSTPIN, LOW);
									}
									if (kvProp.key() == "ResetHC12")
									{
										sensorPreviousStatus = sensorStatus::HC12ERROR;
										digitalWrite(HC12RST, LOW);
										delay(500);
										digitalWrite(HC12RST, HIGH);
									}
								}
							}
						}
						else
						{
#ifdef DEBUG_HUB  //Debug info
							softSerial.println(err.c_str());
#endif
						}
					}
				}
				else
				{
					recpos = 0;
				}
			}
		}
		// flag time out
		else
		{
			sendStatus = 2;
			lTimeOut += random(1000);
			if (send_retry >= MAX_RETRY)
			{
				sendStatus = 16;
				noResponse++;
				// restart the sensor after 24 consecutive time out
				if (noResponse == 24)
				{
					RestartSensor();
				}
				// set the status to bad transmission below 24 attempt
				sensorPreviousStatus = sensorStatus::BAD_TRANSMISSION;
			}
#ifdef DEBUG_HUB  //Debug info
			softSerial.print("ACK time out ");
			softSerial.println(lTimeOut);
#endif
		}
	}

	// get to sleep for saving energy if the message has been received or time out
	if (sendStatus > 4)
	{
		// set the HC-12 to sleep with a recovery safe
		if (!SetHC12ToSleep())
		{
			sensorPreviousStatus = sensorStatus::HC12ERROR;
			digitalWrite(HC12RST, LOW);
			delay(200);
			digitalWrite(HC12RST, HIGH);
			if (!SetHC12ToSleep())
			{
				RestartSensor();
			}
		}
#ifdef DEBUG_HUB  //Debug info
		softSerial.print("Sleeping for ");
		softSerial.print(interval * 8);
		softSerial.println(" sec");
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