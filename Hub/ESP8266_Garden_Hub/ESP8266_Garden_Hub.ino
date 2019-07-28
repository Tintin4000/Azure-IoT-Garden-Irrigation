/*
 Name:		ESP8266_Garden_Hub.ino
 Created:	2/19/2018
 Author:	Didier Coyman
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

#include <ESP8266WiFi.h>
#include "Token.h"
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <CRC32.h>
#include <time.h>
#include <AzureIoTHub.h>
#include <AzureIoTProtocol_MQTT.h>
#include <BlynkSimpleEsp8266.h>

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial1

#define DEBUG_HUB;

const char myHubID = '0';

const byte SUCCESS = 1;
const byte ERROR_BAD_CRC = 10;
const byte ERROR_BAD_JSON = 11;

// message reception time out in millisecond
const uint16_t MSG_TIMEOUT = 3000;
// message max size
const uint8 MAX_BUFFER_SIZE = 150;

// retry attempt to connect to WiFi
const int retryConnectWiFi = 25;

// NTP settings
WiFiUDP ntpUDP;

// Initialize the Azure IoT Hub Client object
static IOTHUB_CLIENT_LL_HANDLE iothubClientHandle;

// store the properties for the device retrieved from IoT Hub. First the device id, then the change flag, followed by the property values
char devProps[156];
bool deviceTwinUpdate = false;

// serial message buffer
char msgBuffer[156];

// the setup function runs once when you press reset or power the board
void setup() {
	// Get the Hardware Serial port started
	Serial.begin(9600);
	Serial1.begin(115200);
#ifdef DEBUG_HUB  //Debug info
	Serial1.setDebugOutput(true);
#endif
	// Set the pin HIGH for the HC-12 module to be in transmitter mode
	pinMode(D1, OUTPUT);
	digitalWrite(D1, HIGH);

	// counter
	uint8 counter = 0;

	// set WiFi mode to Station only
	WiFi.mode(WIFI_STA);

	// attempt to connect to the access point (AP), retry and if unsuccesfull print diagnostics
	if (WiFi.status() != WL_CONNECTED)
	{
		WiFi.begin(ssid, pass);
#ifdef DEBUG_HUB  //Debug info
		Serial1.print("Connecting to Wifi.");
		Serial1.println();
#endif	
	}

	while (WiFi.status() != WL_CONNECTED && counter <= retryConnectWiFi)
	{
#ifdef DEBUG_HUB  //Debug info
		Serial1.print(".");
#endif
		delay(1000);
		counter++;
	}
	if (counter > retryConnectWiFi)
	{
#ifdef DEBUG_HUB  //Debug info
		Serial1.println("Reseting...");
#endif
		ESP.reset();
	}
	// get the WiFi info
#ifdef DEBUG_HUB
	WiFi.printDiag(Serial1);
#endif

	// Blynk to connect to WiFi and the Blynk cloud server
	Blynk.begin(auth, ssid, pass);

	// use the Arduino time routine to get the time from ntp
	// this is a must to have the azureIot SDK sending message on ESP8266
	counter = 0;
	time_t epochTime = 0;
	configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
	while ((epochTime == 0 || epochTime < 10000000) && counter < 100) {
		counter++;
		delay(2000);
		epochTime = time(NULL);
	}
#ifdef DEBUG_HUB  //Debug info
	Serial1.printf("Epoch=%u\n", epochTime);
	char dateTime[25];
	Serial1.println(getFormattedTimeISO8601(dateTime, 25));
#endif
}

// BEGIN :: Adaptation of the Azure device-to-cloud SDK
static void IoTHubClientStart()
{
	// initialize the IoT Client using MQTT, syntax based on Microsoft Azure IoT Hub SDK
	iothubClientHandle = IoTHubClient_LL_CreateFromConnectionString(connectionstring, MQTT_Protocol);
	if (iothubClientHandle == NULL) {
#ifdef DEBUG_HUB  //Debug info
		Serial1.println("Fail to connect to Azure IoT Hub!");
#endif
	}
	else
	{
#ifdef DEBUG_HUB  //Debug info
		Serial1.println("Connected to Azure IoT Hub!");
#endif
		// set the callback function when receiving messages from the cloud
		if (IoTHubClient_LL_SetMessageCallback(iothubClientHandle, receiveMessageCallback, NULL) != IOTHUB_CLIENT_OK)
		{
#ifdef DEBUG_HUB  //Debug info
			Serial1.println("Azure IoTHub set message callback failed.");
#endif
		}
		else
		{
#ifdef DEBUG_HUB  //Debug info
			Serial1.println("Azure IoTHub set message callback succeeded.");
#endif
		}
		//set the callback function when receiving device twin update from the cloud
		if (IoTHubClient_LL_SetDeviceTwinCallback(iothubClientHandle, updateDeviceTwinCallback, NULL) != IOTHUB_CLIENT_OK)
		{
#ifdef DEBUG_HUB  //Debug info
			Serial1.println("Azure IoTHub device twin properties callback failed.");
#endif
		}
		else
		{
#ifdef DEBUG_HUB  //Debug info
			Serial1.println("Azure IoTHub device twin properties callback succeeded.");
#endif
		}
	}
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
	unsigned int messageTrackingId = (unsigned int)(uintptr_t)userContextCallback;
#ifdef DEBUG_HUB  //Debug info
	Serial1.printf("Message Id: %u Received.\r\n", messageTrackingId);
	Serial1.printf("Result Call Back Called! Result is: %s \r\n", ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));

	if (IOTHUB_CLIENT_CONFIRMATION_OK == result)
	{
		Serial1.println("Message send to Azure IOT Hub");
	}
	else
	{
		Serial1.print("Message failed with code:");
		Serial1.println(ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
	}
#endif
}

static void SendMessage(IOTHUB_CLIENT_LL_HANDLE iothubClientHandle, const char * msgbuffer)
{
	static unsigned int messageTrackingId;
	IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromByteArray((const unsigned char *)msgbuffer, strlen(msgbuffer));
	if (messageHandle == NULL) {
#ifdef DEBUG_HUB  //Debug info
		Serial1.println("Unable to create a IoT Message");
#endif
	}
	if (IoTHubClient_LL_SendEventAsync(iothubClientHandle, messageHandle, SendConfirmationCallback, (void*)(uintptr_t)messageTrackingId) != IOTHUB_CLIENT_OK)
	{
#ifdef DEBUG_HUB  //Debug info
		Serial1.println("Failed to send the message to the IoT Hub.");
#endif
	}
	else
	{
#ifdef DEBUG_HUB  //Debug info
		Serial1.println("IoT Hub accepted the delivery of the message.");
#endif
		IoTHubMessage_Destroy(messageHandle);
	}
	messageTrackingId++;
}
// END :: Adaptation of the Azure device-to-cloud SDK

// BEGIN :: Adaptation of the Azure cloud-to-device SDK
IOTHUBMESSAGE_DISPOSITION_RESULT receiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void *userContextCallback)
{
	IOTHUBMESSAGE_DISPOSITION_RESULT result;
	const unsigned char *buffer = NULL;
	size_t size;
	const char* messageId;
	const char* correlationId;

	// Message properties
	if ((messageId = IoTHubMessage_GetMessageId(message)) == NULL)
	{
		messageId = "<null>";
	}
#ifdef DEBUG_HUB  //Debug info
	Serial1.print("Message ID: ");
	Serial1.println(messageId);
#endif


	if ((correlationId = IoTHubMessage_GetCorrelationId(message)) == NULL)
	{
		correlationId = "<null>";
	}
#ifdef DEBUG_HUB  //Debug info
	Serial1.print("Correlation ID: ");
	Serial1.println(correlationId);
#endif

	// Message content
	if (IoTHubMessage_GetByteArray(message, &buffer, &size) != IOTHUB_MESSAGE_OK)
	{
#ifdef DEBUG_HUB  //Debug info
		Serial1.println("Unable to retrieve the message content.");
#endif
		result = IOTHUBMESSAGE_REJECTED;
	}
	else
	{
		// buffer is not zero terminated - portability to arduino
		char *temp = (char *)malloc(size + 1);
		(void)memcpy(temp, buffer, size);
		temp[size] = '\0';

		if (temp == NULL)
		{
#ifdef DEBUG_HUB  //Debug info
			Serial1.println("Message is NULL.");
#endif
			result = IOTHUBMESSAGE_ABANDONED;
		}
		else
		{
			processMessage(temp);
			free(temp);
		}
		result = IOTHUBMESSAGE_ACCEPTED;
	}
	return result;
}

void updateDeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, size_t size, void *userContextCallback)
{
	// buffer is not zero terminated - portability to arduino
	char *temp = (char *)malloc(size + 1);
	(void)memcpy(temp, payLoad, size);
	temp[size] = '\0';

#ifdef DEBUG_HUB  //Debug info
	Serial1.println(temp);
#endif

	// parse the properties
	DynamicJsonBuffer jsonBuffer;
	JsonObject &root = jsonBuffer.parseObject(temp);
	// iterate the JSON device TWIN desired properties to discover device id
	// depending on the situation the Json doc is not the same
	// full update
	if (root["desired"]["sensor"].success())
	{
		JsonObject &prop = root["desired"]["sensor"];
		prop.printTo(devProps);
		deviceTwinUpdate = true;
	}
	// partial update
	else if (root["sensor"].success())
	{
		JsonObject &prop = root["sensor"];
		prop.printTo(devProps);
		deviceTwinUpdate = true;
	}

#ifdef DEBUG_HUB  //Debug info
	Serial1.println(devProps);
#endif
	
	// release memory
	free(temp);
	jsonBuffer.clear();
}
// END :: Adaptation of the Azure cloud-to-device SDK

// BEGIN: Process message from the cloud
void processMessage(const char* messageContent)
{
#ifdef DEBUG_HUB  //Debug info
	Serial1.print("Message from the Cloud : ");
	Serial1.println(messageContent);
#endif
}
// END: Process message from the cloud

// send the message to the cloud
void sendSensorMessageToAzureIoTHub(JsonObject &jsonRoot) {
	// add some parameters to the message
	jsonRoot["location"] = "MainFrontGarden";
	char messageTime[25];
	jsonRoot["sensordatetime"] = getFormattedTimeISO8601(messageTime, 25);
	// send message to Azure
	char sensorDataJson[255];
	jsonRoot.printTo(sensorDataJson, 255);
	SendMessage(iothubClientHandle, sensorDataJson);
	
	IOTHUB_CLIENT_STATUS status;
	int count = 0;
	while ((IoTHubClient_LL_GetSendStatus(iothubClientHandle, &status) == IOTHUB_CLIENT_OK) && (status == IOTHUB_CLIENT_SEND_STATUS_BUSY) && (count < 200))
	{
		count++;
		IoTHubClient_LL_DoWork(iothubClientHandle);
		delay(10);
	}
#ifdef DEBUG_HUB  //Debug info
	Serial1.printf("Loop count to send & receive IoT Hub message : %u\n", count);
#endif
}

// return time formatted like `hh:mm:ss`, requires 9 bytes
char* getFormattedTime(char* buffer, uint8 buffsize)
{
	if (buffsize >= 9) {
		time_t rawtime;
		struct tm * timeinfo;

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer, buffsize, "%T", timeinfo);
	}
	return buffer;
}

// return time formatted like `dd-mm-yyyy hh:mm:ss`, requires 20 bytes
char*  getFullFormattedTime(char* buffer, int buffsize)
{
	if (buffsize >= 20) {
		time_t rawtime;
		struct tm * timeinfo;

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer, buffsize, "%d-%m-%Y %T", timeinfo);
	}
	return buffer;
}

//// return time formatted like `yyyy-mm-ddThh:mm:ssZ`, requires 25 bytes
char* getFormattedTimeISO8601(char* buffer, int buffsize)
{
	if (buffsize >= 25) {
		time_t rawtime;
		struct tm * timeinfo;

		time(&rawtime);
		timeinfo = gmtime(&rawtime);

		strftime(buffer, 25, "%FT%TZ", timeinfo);
	}
	return buffer;
}

// process message from sensors.  Return true if message is succesfully decoded.
byte processSerialMessage(const uint16 msglength, const char* payLoad)
{
	// Json message object buffer
	StaticJsonBuffer<250> jsonBuffer;
	// parse json message
	JsonObject &jsonRoot = jsonBuffer.parseObject(payLoad);

	// check the json structure
	if (jsonRoot.success())
	{
		// check the checksum
		uint32_t checksum = jsonRoot["CRC32"];
		if (checksum) 
		{
			jsonRoot.remove("CRC32");
			char crc32Buffer[128];
			jsonRoot.printTo(crc32Buffer);
			int bufLength = strlen(crc32Buffer);
			uint32_t calcchecksum = CRC32::calculate(crc32Buffer, bufLength);
			// flag CRC32 pass or fail
			if (checksum != calcchecksum)
			{
#ifdef DEBUG_HUB
				//Debug info
				Serial1.printf("JSON message length = %u\n", jsonRoot.measureLength());
				Serial1.printf("Buffer length = %u\n", bufLength);
				Serial1.printf("CRC32 incorrect. Checksum = %u, Calc checksum = %u\n%s\n", checksum, calcchecksum, payLoad);
#endif
				return ERROR_BAD_CRC;
			}
		}
	}
	else
	{
#ifdef DEBUG_HUB  //Debug info
		Serial1.printf("JSON message not correctly formated. => %s\n", payLoad);
#endif	
		return ERROR_BAD_JSON;
	}
	return SUCCESS;
}

void SendCommand(const char devID)
{
	DynamicJsonBuffer jsonBuffer;
	JsonObject &root = jsonBuffer.parseObject(devProps);

	for (auto kv : root) {
		if ((strlen(kv.key) == 1) && (kv.key[0] == devID))
		{
			if (root[kv.key]["Sleep"].success())
			{
				// send the command to change the interval
				Serial.write(devID);
				Serial.write(17);
				// send the interval
				Serial.print((const char*) root[kv.key]["Sleep"]);
				Serial.write(10);
#ifdef DEBUG_HUB  //Debug info
				Serial1.printf("Send command to device: %d, Sleep = %s\n", devID, (const char*) root[kv.key]["Sleep"]);
#endif	
			}
			if (root[kv.key]["Channel"].success())
			{
				// send the command to change the channel
				Serial.write(devID);
				Serial.write(18);
				// send the channel
				Serial.print((const char*) root[kv.key]["Channel"]);
				Serial.write(10);
#ifdef DEBUG_HUB  //Debug info
				Serial1.printf("Send command to device: %d, Channel = %s\n", devID, (const char*) root[kv.key]["Channel"]);
#endif	
			}
		}
	}
}

// ----------------------------------------------------------------------------------------------- LOOP ---------------------------------------------------------------------------------------------
// the loop function runs over and over again until power down or reset
void loop() {
	static char serialBuffer[MAX_BUFFER_SIZE];
	static uint8 buffpos = 0;
	static byte inMessage = false, msgReady = false, myMessage = false;
	static uint32_t msgStartTime;
	static uint16 msgLength = 0;
	static uint8_t devID, hubID;

	// message time out
	if (inMessage && ((millis() - msgStartTime) > MSG_TIMEOUT))
	{
		inMessage = false;
		// send a NACK to the sensor
		Serial.write(21);
#ifdef DEBUG_HUB  //Debug info
		Serial1.println();
		Serial1.println("NACK, message time out.");
		serialBuffer[buffpos] = 0;
		Serial1.println(serialBuffer);
#endif	
	}

	// process incoming serial bytes
	while (Serial.available())
	{
		char incoming = Serial.read();
		switch (incoming)
		{
		case 2:  // message start flag (ASCII 2)
			msgStartTime = millis();
			inMessage = true;
			myMessage = true;
			buffpos = 0;
			msgLength = 0;
#ifdef DEBUG_HUB  //Debug info
			Serial1.println();
			char dateTime[20];
			getFullFormattedTime(dateTime, 20);
			Serial1.println(dateTime);
#endif
			break;
		case 3:  // message header end flag (ASCII 3)
			if (inMessage)
			{
				msgLength = 0;
				byte i = 0;
				// get the message length
				while ((i < buffpos) && (serialBuffer[i] != ':')) {
					if (isDigit(serialBuffer[i]))
					{
						msgLength = (msgLength * 10) + (int(serialBuffer[i]) - 48);
					}
					i++;
				}i++;
				// get the device ID
				while ((i < buffpos) && (serialBuffer[i] != ':')) {
					devID = serialBuffer[i];
					i++;
				}i++;
				// get the hub ID
				while ((i < buffpos) && (serialBuffer[i] != ':')) {
					hubID = serialBuffer[i];
					i++;
				}
#ifdef DEBUG_HUB  //Debug info
				Serial1.printf("Message length is %d, devID is %d, hubID is %d.", msgLength, devID, hubID);
				Serial1.println();
#endif
				buffpos = 0;
				(hubID == myHubID) ? myMessage = true : myMessage = false;
			}
			break;
		case '\r':  //discard carriage return
			break;
		case '\n':  //message end
			if (inMessage && myMessage)
			{
#ifdef DEBUG_HUB  //Debug info
				Serial1.printf("Message response time: %d ms.\n", millis() - msgStartTime);
#endif
				serialBuffer[buffpos] = 0;
				buffpos = 0;
				inMessage = false;
				// send the update command if required
				if (deviceTwinUpdate) SendCommand(devID);
				// process the message
				if (msgLength > 0)
				{
#ifdef DEBUG_HUB  //Debug info
					Serial1.println(serialBuffer);
#endif	
					byte returnCode = processSerialMessage(msgLength, serialBuffer);
#ifdef DEBUG_HUB  //Debug info
					Serial1.printf("Process response time: %d ms.\n", millis() - msgStartTime);
#endif
					if (returnCode == SUCCESS)
					{
						// send an ACK to the sensor
						Serial.write(devID);
						Serial.write(6);
						Serial.write(10);
						msgReady = true;
#ifdef DEBUG_HUB  //Debug info
						Serial1.printf("Send ACK to %c\n", devID);
#endif	
					}
					else
					{
						// send a NACK to the sensor
						Serial.write(devID);
						Serial.write(21);
						Serial.write(10);
#ifdef DEBUG_HUB  //Debug info
						Serial1.printf("NACK, message not correctly formated. Error %d", returnCode);
						Serial1.println(serialBuffer);
#endif	
					}
					msgLength = 0;
				}
			}
			break;
		default:
			if (inMessage && myMessage)
			{
				// buffer overflow
				if (buffpos == (MAX_BUFFER_SIZE - 1))
				{
					inMessage = false;
					// send a NACK to the sensor
					Serial.write(devID);
					Serial.write(21);
					Serial.write(10);
#ifdef DEBUG_HUB  //Debug info
					Serial1.println();
					Serial1.println("NACK, buffer over run");
					serialBuffer[buffpos] = 0;
					Serial1.println(serialBuffer);
#endif	
					break;
				}
				serialBuffer[buffpos] = incoming;
				buffpos++;
			}
			break;
		}
	}

	// send the data from the sensor
	if (msgReady)
	{
		msgReady = false;
		// remove the checksum before sending to azure cloud
		StaticJsonBuffer<250> jsonBuffer;
		JsonObject &jsonRoot = jsonBuffer.parseObject(serialBuffer);
		jsonRoot.remove("CRC32");
		// send the message to the Azure IoT Hub
		unsigned long azureTimer = millis();
		IoTHubClientStart();
		sendSensorMessageToAzureIoTHub(jsonRoot);
		IoTHubClient_LL_Destroy(iothubClientHandle);
#ifdef DEBUG_HUB  //Debug info
		Serial1.printf("Azure send message elapse : %d msec\n", millis() - azureTimer);
#endif
		//send the temperature and humidity to Blynk
		float airTemp = atof(jsonRoot["AirTemp"]);
		float airHum = atof(jsonRoot["AirHum"]);
		Blynk.virtualWrite(V0, airTemp);
		Blynk.virtualWrite(V1, airHum);
	}

	// allow Blynk to operate
	Blynk.run();
}