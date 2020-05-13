/*
 Name:		ESP32_Garden_Hub.ino
 Created:	15 March 2020
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

#include <unordered_map>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <CRC32.h>
#include <AzureIoTHub.h>
#include "AzureIoTProtocol_MQTT.h"
#include "iothubtransportmqtt.h"
#include "Token.h"

#define DEBUG_HUB;

// Garden hub id used by the sensor to identified the receiver
constexpr char myHubID[] = "ESP32HUB1";

// maximum number of sensor supported
constexpr uint8_t MAX_SENSOR_ID = 5;

// maximum number of sensor properties
constexpr uint8_t MAX_SENSOR_PROPERTIES = 5;

// Sensor message max size
constexpr uint8_t MAX_BUFFER_SIZE = 250;
// Sensor message max size without CRC
constexpr uint8_t MAX_BUFFER_NOCRC_SIZE = MAX_BUFFER_SIZE - 18;
// JSON memory pool size, MUST be declared as a INT!
constexpr int MAX_JSON_SIZE = 400;
constexpr int MAX_JSON_TWIN_SIZE = 350;

constexpr byte SUCCESS = 1;
constexpr byte ERROR_BAD_CRC = 10;
constexpr byte ERROR_BAD_JSON = 11;

// NTP settings
WiFiUDP ntpUDP;

// structure for the sensor property and method
typedef struct {
    std::string property = "";
    uint16_t value = 0;
} sensorprop;

// structure for the sensor feedback message
typedef struct {
    uint8_t sensorID = 255;
    uint8_t acknowledge = 255;
    sensorprop property[5];
} sensorfeedback;

// sensor feedback storage array, using the array index as the sensor id.
sensorfeedback sensorFeedback[MAX_SENSOR_ID];

// the following map is used to store the command for the sensor and the corresponding code to send to the sensor
std::unordered_map<std::string, uint8_t> sensorCommand{ {"Sleep", 17}, {"Channel", 18}, {"Restart", 24}, {"ResetHC12", 22} };

// Queues used to pass information between tasks or events
static QueueHandle_t qHasIP;
static QueueHandle_t qUartBuffer;
static QueueHandle_t qJsonMessage;
static QueueHandle_t qJsonAzure;
static QueueHandle_t qFeedbackSensor;

// Tasks used to clearly identify enclosed activities and share the load between core
static TaskHandle_t tReceiveSerialSensorMsg = NULL;
static TaskHandle_t tProcessSensorMsg = NULL;
static TaskHandle_t tSendToAzure = NULL;
static TaskHandle_t tFeedbackSensor = NULL;

// Flag the end of an Azure IoT transmission exchange
volatile byte endOfAzureIoTEx = false;

// WiFi event
void WiFiEvent(WiFiEvent_t event)
{
#ifdef DEBUG_HUB  //Debug info
    Serial.printf("[WiFi-event] event: %d - Connected: %d\n", event, WiFi.isConnected());
#endif
    if (event == SYSTEM_EVENT_STA_GOT_IP)
    {
        uint8_t item = true;
        xQueueSendToBack(qHasIP, &item, 0);
#ifdef DEBUG_HUB  //Debug info
        Serial.print("IP address = ");
        Serial.println(WiFi.localIP());
#endif
    }
}

// Collect the information received from the sensor through serial communication separated by CRLF
void ReceiveSerialSensorMessageTask(void* pParam)
{
    uint8_t uartinputchar;
    bool cr = false;
    uint32_t lStartTime;
    char serialBuffer[MAX_BUFFER_SIZE];
    uint8_t buffPos = 0;

    const TickType_t xTicksToWait = pdMS_TO_TICKS(1);

#ifdef DEBUG_HUB  //Debug info
    Serial.printf("%s active on Core %d.\n", pcTaskGetTaskName(NULL), xPortGetCoreID());
#endif

    for (;;)
    {
        if (Serial1.available())
        {
#ifdef DEBUG_HUB  //Debug info
            if (buffPos == 0) lStartTime = millis();
#endif
            uartinputchar = Serial1.read();
            // avoid to overflow the buffer
            if (buffPos == (MAX_BUFFER_SIZE - 1))
            {
                buffPos = 0;
            }
            serialBuffer[buffPos] = uartinputchar;
            buffPos++;
            // store the message in the queue when a CRLF is detected
            if ((cr) && (uartinputchar == '\n'))
            {
                // terminate the message by a null character
                serialBuffer[buffPos] = '\0';
                // reset the queue when full
                if (uxQueueSpacesAvailable(qUartBuffer) == 0)
                {
                    xQueueReset(qUartBuffer);
#ifdef DEBUG_HUB  //Debug info
                    Serial.println("The UART buffer queue is full.");
#endif
                }
                xQueueSendToBack(qUartBuffer, serialBuffer, 0);

#ifdef DEBUG_HUB  //Debug info
                Serial.printf("Message length = %d\n", buffPos);
                Serial.printf("Serial response time: %d ms.\n", millis() - lStartTime);
                Serial.printf("Available heap memory = %d\n", ESP.getFreeHeap());
                Serial.printf("The minimum amount of remaining stack space is %u.\n", uxTaskGetStackHighWaterMark(NULL));
#endif
                buffPos = 0;
                xTaskNotifyGive(tProcessSensorMsg);
            }
            // reset the CR flag
            else
            {
                cr = false;
            }
            // set the CR flag
            if (uartinputchar == '\r') cr = true;
        }
        vTaskDelay(xTicksToWait);
    }
    vTaskDelete(NULL);
}

// Process the sensor JSON message
void ProcessSensorMessageTask(void* pParam)
{
    uint32_t ulNotification = 0;
    uint8_t charin;
    uint32_t lStartTime;
    char stringBuff[MAX_BUFFER_SIZE];

#ifdef DEBUG_HUB  //Debug info
    Serial.printf("%s active on Core %d.\n", pcTaskGetTaskName(NULL), xPortGetCoreID());
#endif


    for (;;)
    {
        // wait for the task to be activate upon reception of a message from a sensor
        ulNotification = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        lStartTime = millis();

        if (ulNotification > 0)
        {
            // loop until the queue is empty
            while (uxQueueMessagesWaiting(qUartBuffer))
            {
#ifdef DEBUG_HUB  //Debug info
                Serial.printf("Processing 1 sensor message out of %d.\n", uxQueueMessagesWaiting(qUartBuffer));
#endif
                xQueueReceive(qUartBuffer, stringBuff, 0);
                // Json message object buffer
                StaticJsonDocument<MAX_JSON_SIZE> jsonSensorDoc;
                // Parse Json message
                DeserializationError err = deserializeJson(jsonSensorDoc, stringBuff);
                // notify on incorrect formated Json
                if (err)
                {
#ifdef DEBUG_HUB  //Debug info
                    Serial.printf("Incorrectly formated Json (%s).\n%s\n", err.c_str(), stringBuff);
#endif
                }
                else
                {
                    // determine the sensor id used to store the feedback message
                    uint8_t sensorID = (uint8_t) jsonSensorDoc["deviceid"];

#ifdef DEBUG_HUB  //Debug info
                    Serial.printf("Correctly formated Json from sensor id %d.\n", sensorID);
#endif

                    if (sensorID > MAX_SENSOR_ID) {
#ifdef DEBUG_HUB  //Debug info
                        Serial.printf("Sensor ID %u is bigger than the Sensor MAX value.\n", sensorID);
#endif
                    }

                    // check the checksum
                    uint32_t checksum = jsonSensorDoc["CRC32"];
                    if (!checksum)
                    {
                        if (jsonSensorDoc["deviceid"])
                        {
                            if (sensorID < MAX_SENSOR_ID)
                            {
                                sensorFeedback[sensorID].sensorID = sensorID;
                                sensorFeedback[sensorID].acknowledge = 21;  // Ascii for NACK
#ifdef DEBUG_HUB  //Debug info
                                Serial.printf("No CRC32, sending NACK to %u.\n", sensorID);
#endif
                                xQueueSendToBack(qFeedbackSensor, &sensorFeedback[sensorID], 0);
                            }
                        }
                    }
                    else
                    {
                        jsonSensorDoc.remove("CRC32");
                        char crc32Buffer[MAX_BUFFER_NOCRC_SIZE];
                        serializeJson(jsonSensorDoc, crc32Buffer);
                        int bufLength = strlen(crc32Buffer);
                        uint32_t calcchecksum = CRC32::calculate(crc32Buffer, bufLength);
                        // flag CRC32 pass or fail
                        if (checksum != calcchecksum)
                        {
                            if (jsonSensorDoc["deviceid"])
                            {
                                if (sensorID < MAX_SENSOR_ID)
                                {
                                    sensorFeedback[sensorID].sensorID = sensorID;
                                    sensorFeedback[sensorID].acknowledge = 21;  // Ascii for NACK
#ifdef DEBUG_HUB  //Debug info
                                    Serial.printf("Incorrect CRC32, sending NACK to %u.\n", sensorID);
#endif
                                    xQueueSendToBack(qFeedbackSensor, &sensorFeedback[sensorID], 0);
                                }
                            }
                        }
                        else
                        {
                            // adding the necessary propoerties for the Azure message
                            char messageTime[25];
                            jsonSensorDoc["sensordatetime"] = getFormattedTimeISO8601(messageTime, 25);
                            jsonSensorDoc["HubID"] = myHubID;
                            if (jsonSensorDoc["deviceid"])
                            {
                                if (sensorID < MAX_SENSOR_ID)
                                {
                                    sensorFeedback[sensorID].sensorID = sensorID;
                                    sensorFeedback[sensorID].acknowledge = 6;  // Ascii for ACK
#ifdef DEBUG_HUB  //Debug info
                                    Serial.printf("Correct CRC32, sending ACK to %u.\n", sensorID);
#endif
                                    xQueueSendToBack(qFeedbackSensor, &sensorFeedback[sensorID], 0);
                                }
                            }
                            // insert the JSON sensor message in the Azure queue
                            char msgPayLoad[MAX_JSON_SIZE];
                            if (measureJson(jsonSensorDoc) < MAX_JSON_SIZE)
                            {
                                size_t lCount = 0;
                                lCount = serializeJson(jsonSensorDoc, msgPayLoad);
                                if (lCount) xQueueSendToBack(qJsonAzure, &msgPayLoad, 0);
                                else {
#ifdef DEBUG_HUB  //Debug info
                                    Serial.println("Serialized the JSON message failed!");
#endif
                                }
                            }
                            else {
#ifdef DEBUG_HUB  //Debug info
                                Serial.println("The buffer for the Azure JSON message is too small.");
#endif
                            }
                        }
                    }

                }
            }
#ifdef DEBUG_HUB  //Debug info
            Serial.printf("Message processing time: %d ms.\n", millis() - lStartTime);
            Serial.printf("Available heap memory = %d\n", ESP.getFreeHeap());
            Serial.printf("The minimum amount of remaining stack space for ProcessSensorMessageTask is %u.\n", uxTaskGetStackHighWaterMark(NULL));
#endif
        }
    }
    vTaskDelete(NULL);
}

// Send the acknowledgement to the sensor
void FeedbackSensorTask(void* pParam)
{
    const TickType_t xTicksToWait = pdMS_TO_TICKS(1);

#ifdef DEBUG_HUB  //Debug info
    Serial.printf("%s active on Core %d.\n", pcTaskGetTaskName(NULL), xPortGetCoreID());
#endif
    
    for (;;)
    {
        // inititate the structure return by the item in the queue
        sensorfeedback SensorFeedbackFromQueue;

        xQueueReceive(qFeedbackSensor, &SensorFeedbackFromQueue, portMAX_DELAY);
#ifdef DEBUG_HUB  //Debug info
        Serial.printf("Send Ack (%u) to Sensor (%u).\n", SensorFeedbackFromQueue.acknowledge, SensorFeedbackFromQueue.sensorID);
        Serial.printf("Available heap memory = %d\n", ESP.getFreeHeap());
        Serial.printf("The minimum amount of remaining stack space for FeedbackSensorTask is %u.\n", uxTaskGetStackHighWaterMark(NULL));
#endif
        // send the feedback to the sensor, starting by the sensor id, all the properties and the acknowledge status
        Serial1.write(SensorFeedbackFromQueue.sensorID);
        // loop through the properties
        for (uint8_t i = 0; i < MAX_SENSOR_PROPERTIES; i++)
        {
            // any of the property can be empty
            if (!SensorFeedbackFromQueue.property[i].property.empty())
            {
                // search the sensor command list for the equivalent command code
                if (sensorCommand.find(SensorFeedbackFromQueue.property[i].property) != sensorCommand.end())
                {
                    Serial1.printf("%u", sensorCommand[SensorFeedbackFromQueue.property[i].property]);
                }
            }
        }
        Serial1.write(SensorFeedbackFromQueue.acknowledge);
        //vTaskDelay(xTicksToWait);
    }
    vTaskDelete(NULL);
}

// Send the acknowledgement to the sensor
void SendToAzureTask(void* pParam)
{
    uint32_t lStartTime;
    IOTHUB_CLIENT_LL_HANDLE iothubClientHandle;
    const TickType_t xTicksToWait = pdMS_TO_TICKS(10);

#ifdef DEBUG_HUB  //Debug info
    Serial.printf("%s active on Core %d.\n", pcTaskGetTaskName(NULL), xPortGetCoreID());
#endif

    for (;;)
    {
        char msgPayLoad[MAX_JSON_SIZE];
        xQueueReceive(qJsonAzure, &msgPayLoad, portMAX_DELAY);
#ifdef DEBUG_HUB  //Debug info
        lStartTime = millis();
        Serial.println("Sending the JSON message to Azure IoT cloud.");
        Serial.println(msgPayLoad);
#endif
        // initiate the connection to the Azure IoT cloud
        iothubClientHandle = IoTHubClientStart();
        
        int count = 0;
        
        if (iothubClientHandle != NULL)
        {
            // send the message to the Azure IoT cloud
            SendMessage(iothubClientHandle, msgPayLoad);

            // check for status and wait
            IOTHUB_CLIENT_STATUS status;
            int storeStatus = 0;
            while (!endOfAzureIoTEx && count < 2000)
            {
                count++;
                IoTHubClient_LL_DoWork(iothubClientHandle);
                IoTHubClient_LL_GetSendStatus(iothubClientHandle, &status);
                if (storeStatus != status)
                {
                    Serial.printf("Status = %u\n", status);
                    storeStatus = status;
                }
                vTaskDelay(xTicksToWait);
            }

            // Clean up the iothub sdk handle
            IoTHubDeviceClient_LL_Destroy(iothubClientHandle);
            // Free all the sdk subsystem
            IoTHub_Deinit();
        }
#ifdef DEBUG_HUB  //Debug info
        Serial.printf("Azure processing time is %u ms, and DoWork count is %u.\n", millis() - lStartTime, count);
        Serial.printf("Available heap memory = %u\n", ESP.getFreeHeap());
        Serial.printf("The minimum amount of remaining stack space for SendToAzureTask is %u.\n", uxTaskGetStackHighWaterMark(NULL));
#endif
    }
    vTaskDelete(NULL);
}

// Task description
//void NameTask(void* pParam)
//{
//    const TickType_t xTicksToWait = pdMS_TO_TICKS(1);
//
//#ifdef DEBUG_HUB  //Debug info
//    Serial.printf("%s active on Core %d.\n", pcTaskGetTaskName(NULL), xPortGetCoreID());
//#endif
//
//    for (;;)
//    {
//        vTaskDelay(xTicksToWait);
//    }
//    vTaskDelete(NULL);
//}

// BEGIN :: Adaptation of the Azure device-to-cloud SDK
IOTHUB_CLIENT_LL_HANDLE IoTHubClientStart()
{
    IOTHUB_CLIENT_LL_HANDLE iothubClientHandle;

    // initialize IoTHub SDK subsystem
    (void)IoTHub_Init();
    // initialize the IoT Client using MQTT, syntax based on Microsoft Azure IoT Hub SDK
    iothubClientHandle = IoTHubClient_LL_CreateFromConnectionString(myConnectionString, MQTT_Protocol);
    if (iothubClientHandle == NULL) {
#ifdef DEBUG_HUB  //Debug info
        Serial.println("Fail to connect to Azure IoT Hub!");
#endif
        return NULL;
    }
    else
    {
#ifdef DEBUG_HUB  //Debug info
        Serial.println("Connected to Azure IoT Hub!");
#endif
        // Set any option that are neccessary.  For available options please see the iothub_sdk_options.md documentation in the main C SDK
        endOfAzureIoTEx = false;

        // turn off diagnostic sampling
        int diag_off = 0;
        IoTHubDeviceClient_LL_SetOption(iothubClientHandle, OPTION_DIAGNOSTIC_SAMPLING_PERCENTAGE, &diag_off);

#ifdef DEBUG_HUB  //Debug info
        // turn on tracing for troubleshooting
        bool traceOn = false;
        IoTHubDeviceClient_LL_SetOption(iothubClientHandle, OPTION_LOG_TRACE, &traceOn);
#endif
        // setting the Trusted Certificate.
        IoTHubDeviceClient_LL_SetOption(iothubClientHandle, OPTION_TRUSTED_CERT, certificates);

        // setting the auto URL Encoder (recommended for MQTT)
        bool urlEncodeOn = true;
        IoTHubDeviceClient_LL_SetOption(iothubClientHandle, OPTION_AUTO_URL_ENCODE_DECODE, &urlEncodeOn);

         // Setting connection status callback to get indication of connection to iothub
        if (IoTHubDeviceClient_LL_SetConnectionStatusCallback(iothubClientHandle, ConnectionStatusCallback, NULL) != IOTHUB_CLIENT_OK)
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.println("Azure IoTHub set connection status callback failed.");
#endif
        }
        else
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.println("Azure IoTHub set connection status callback succeeded.");
#endif
        }

        // set the callback function when receiving messages from the cloud
        if (IoTHubClient_LL_SetMessageCallback(iothubClientHandle, receiveMessageCallback, NULL) != IOTHUB_CLIENT_OK)
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.println("Azure IoTHub set message callback failed.");
#endif
        }
        else
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.println("Azure IoTHub set message callback succeeded.");
#endif
        }
        //set the callback function when receiving device twin update from the cloud
        if (IoTHubClient_LL_SetDeviceTwinCallback(iothubClientHandle, updateDeviceTwinCallback, NULL) != IOTHUB_CLIENT_OK)
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.println("Azure IoTHub device twin properties callback failed.");
#endif
        }
        else
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.println("Azure IoTHub device twin properties callback succeeded.");
#endif
        }

        
        if (IoTHubDeviceClient_LL_SetDeviceMethodCallback(iothubClientHandle, deviceMethodCallback, NULL) != IOTHUB_CLIENT_OK)
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.println("Azure IoTHub set Device Method callback failed.");
#endif
        }
        else
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.println("Azure IoTHub set Device Method callback succeeded.");
#endif
        }
         //(void)IoTHubDeviceClient_LL_SendReportedState(iotHubClientHandle, (const unsigned char*)reportedProperties, strlen(reportedProperties), reportedStateCallback, NULL);
        return iothubClientHandle;
   }
}

// Callback method which executes on receipt of a connection status message from the IoT Hub in the cloud.
void ConnectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
    (void)reason;
    (void)user_context;

#ifdef DEBUG_HUB  //Debug info
    if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
    {
        Serial.println("The device client is connected to iothub");
    }
    else
    {
        Serial.println("The device client has been disconnected.");
    }
#endif
}

// Callback method which executes upon confirmation that a message originating from this device has been received by the IoT Hub in the cloud.
void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
    unsigned int messageTrackingId = (unsigned int)(uintptr_t)userContextCallback;
#ifdef DEBUG_HUB  //Debug info
    Serial.printf("Message Id: %u Received.\n", messageTrackingId);
    Serial.printf("Result is: %s\n", MU_ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));

    if (IOTHUB_CLIENT_CONFIRMATION_OK == result)
    {
        Serial.println("Message send to Azure IOT Hub");
    }
    else
    {
        Serial.print("Message failed with code:");
        Serial.println(MU_ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
    }
#endif
}

// Callback method which executes upon receipt of a message originating from the IoT Hub in the cloud.
IOTHUBMESSAGE_DISPOSITION_RESULT receiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback)
{
    IOTHUBMESSAGE_DISPOSITION_RESULT result;
    const unsigned char* buffer = NULL;
    size_t size;
    const char* messageId;
    const char* correlationId;

    // Message properties
    if ((messageId = IoTHubMessage_GetMessageId(message)) == NULL)
    {
        messageId = "<null>";
    }
#ifdef DEBUG_HUB  //Debug info
    Serial.printf("Message ID: %s\n", messageId);
#endif


    if ((correlationId = IoTHubMessage_GetCorrelationId(message)) == NULL)
    {
        correlationId = "<null>";
    }
#ifdef DEBUG_HUB  //Debug info
    Serial.printf("Correlation ID: %s\n", correlationId);
#endif

    // Message content
    if (IoTHubMessage_GetByteArray(message, &buffer, &size) != IOTHUB_MESSAGE_OK)
    {
#ifdef DEBUG_HUB  //Debug info
        Serial.println("Unable to retrieve the message content.");
#endif
        result = IOTHUBMESSAGE_REJECTED;
    }
    else
    {
        // buffer is not zero terminated - portability to arduino
        char* temp = (char*)malloc(size + 1);
        (void)memcpy(temp, buffer, size);
        temp[size] = '\0';

        if (temp == NULL)
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.println("Message is NULL.");
#endif
            result = IOTHUBMESSAGE_ABANDONED;
        }
        else
        {
#ifdef DEBUG_HUB  //Debug info
            Serial.printf("Message from the Cloud : %s\n", temp);
#endif
            free(temp);
        }
        result = IOTHUBMESSAGE_ACCEPTED;
    }
    return result;
}

void SendMessage(IOTHUB_CLIENT_LL_HANDLE iothubClientHandle, const char* msgbuffer)
{
    static unsigned int messageTrackingId;
    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromByteArray((const unsigned char*)msgbuffer, strlen(msgbuffer));
    if (messageHandle == NULL) {
#ifdef DEBUG_HUB  //Debug info
        Serial.println("Unable to create an IoT Message");
#endif
    }
    if (IoTHubClient_LL_SendEventAsync(iothubClientHandle, messageHandle, SendConfirmationCallback, (void*)(uintptr_t)messageTrackingId) != IOTHUB_CLIENT_OK)
    {
#ifdef DEBUG_HUB  //Debug info
        Serial.println("Failed to send the message to the IoT Hub.");
#endif
    }
    else
    {
#ifdef DEBUG_HUB  //Debug info
        Serial.println("IoT Hub accepted the delivery of the message.");
#endif
        IoTHubMessage_Destroy(messageHandle);
    }
    messageTrackingId++;
}

void reportedStateCallback(int status_code, void* userContextCallback)
{
    (void)userContextCallback;
    Serial.printf("Device Twin reported properties update completed with result: %d\r\n", status_code);
}

int deviceMethodCallback(const char* method_name, const unsigned char* payload, size_t size, unsigned char** response, size_t* response_size, void* userContextCallback)
{
    (void)userContextCallback;
    (void)payload;
    (void)size;

    printf("Method Name: %s\n", method_name);
}

void updateDeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char* payLoad, size_t size, void* userContextCallback)
{
    // buffer is not zero terminated - portability to arduino
    char* temp = (char*)malloc(size + 1);
    (void)memcpy(temp, payLoad, size);
    temp[size] = '\0';

#ifdef DEBUG_HUB  //Debug info
    Serial.printf("TWIN Update is %s.\n", temp);
#endif
    // parse the properties
    StaticJsonDocument<MAX_JSON_TWIN_SIZE> jsonTwinDoc;
    DeserializationError err = deserializeJson(jsonTwinDoc, temp);

#ifdef DEBUG_HUB  //Debug info
    Serial.printf("JSON property message status => %s\n", err.c_str());
#endif	

    // iterate the JSON device TWIN desired properties to add device properties to the storage
    // example : {"desired":{"sensor":{"1":{"Channel":1,"Sleep":425}},"$version":4},"reported":{"$version":1}}
    JsonObject jsonTwinRoot = jsonTwinDoc.as<JsonObject>();
    if (jsonTwinRoot["desired"]["sensor"] != nullptr)
    {
        // locate the sensor section
        JsonObject jsonSensors = jsonTwinRoot["desired"]["sensor"];
        if (jsonSensors != NULL)
        {
            for (JsonPair kvSensor : jsonSensors)
            {
                // the TWIn device json message value is a text and need to be converted safely
                uint16_t ulNode = std::strtoul(kvSensor.key().c_str(), NULL, 10);
                if (ulNode && ulNode < MAX_SENSOR_ID)
                {
                    JsonObject jsonSensor = jsonSensors[kvSensor.key().c_str()];
                    uint8_t i = 0;
                    for (JsonPair kvSensorProp : jsonSensor)
                    {
                        // max char array size for property is 10 and max number of property is 5
                        if ((strlen(kvSensorProp.key().c_str()) < 10) && (i < 5))
                        {
                            //strcpy(sensorFeedback[ulNode].property[i].property, kvSensorProp.key().c_str());
                            std::string keyProp(kvSensorProp.key().c_str());
                            
                            sensorFeedback[ulNode].property[i].property = keyProp;
                            sensorFeedback[ulNode].property[i].value = kvSensorProp.value().as<short>();
                            i++;
                        }
                    }
                }
            }
        }
    }

    // flag the transmission exchange to be complete
    endOfAzureIoTEx = true;

    // release memory
    free(temp);
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

// return time formatted like `hh:mm:ss`, requires 9 bytes
char* getFormattedTime(char* buffer, uint8_t buffsize)
{
    if (buffsize >= 9) {
        time_t rawtime;
        struct tm* timeinfo;

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, buffsize, "%T", timeinfo);
    }
    return buffer;
}

// return time formatted like `dd-mm-yyyy hh:mm:ss`, requires 20 bytes
char* getFullFormattedTime(char* buffer, int buffsize)
{
    if (buffsize >= 20) {
        time_t rawtime;
        struct tm* timeinfo;

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, buffsize, "%d-%m-%Y %T", timeinfo);
    }
    return buffer;
}

// return time formatted like `yyyy-mm-ddThh:mm:ssZ`, requires 25 bytes
char* getFormattedTimeISO8601(char* buffer, int buffsize)
{
    if (buffsize >= 25) {
        time_t rawtime;
        struct tm* timeinfo;

        time(&rawtime);
        timeinfo = gmtime(&rawtime);

        strftime(buffer, 25, "%FT%TZ", timeinfo);
    }
    return buffer;
}

// the setup function runs once when you press reset or power the board
void setup() 
{
    Serial.begin(115200);
    Serial1.begin(9600);
    Serial1.println("Ready!");
    pinMode(BUILTIN_LED, OUTPUT);

#ifdef DEBUG_HUB  //Debug info
    WiFi.onEvent(WiFiEvent);
    Serial.printf("\nInitializing on core %d\n", xPortGetCoreID());
    Serial.printf("Total heap memory = %d\n", ESP.getHeapSize());
    Serial.printf("Available heap memory = %d\n", ESP.getFreeHeap());
#endif

    // set WiFi mode to Station only and enable event
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    WiFi.begin(mySSID, myPassword);

    uint8_t item = true;
    qHasIP = xQueueCreate(1, 1);
    xQueueReceive(qHasIP, &item, (TickType_t)(10000 / portTICK_PERIOD_MS));

    if (WiFi.isConnected())
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

        // create the queue to hold the data from the sensor serial transmission
        qUartBuffer = xQueueCreate(4, 250);
        // create the task to queue serial sensor data
        xTaskCreatePinnedToCore(ReceiveSerialSensorMessageTask, "RecSerialSensorMsgTask", 2500, NULL, 5, &tReceiveSerialSensorMsg, 0);

        // create the queue to hold the JSON message from the sensor
        qJsonMessage = xQueueCreate(4, 250);
        // create the task to process the sensor message
        xTaskCreatePinnedToCore(ProcessSensorMessageTask, "ProcSensorMsgTask", 4000, NULL, 2, &tProcessSensorMsg, 0);

        // create the queue to hold the acknowledgment to the sensor
        qFeedbackSensor = xQueueCreate(4, sizeof(sensorfeedback));
        // create the task that send the acknowledgment to the sensor
        xTaskCreatePinnedToCore(FeedbackSensorTask, "FeedbackSensorTask", 2500, NULL, 3, &tFeedbackSensor, 0);

        // create the queue to hold the JSON message to send to the Azure Cloud
        qJsonAzure = xQueueCreate(5, MAX_JSON_SIZE);
        // create the task to send the JSON message to the Azure Cloud
        xTaskCreatePinnedToCore(SendToAzureTask, "SendToAzureTask", 6000, NULL, 1, &tSendToAzure, 1);

#ifdef DEBUG_HUB  //Debug info
        Serial.printf("Epoch = %u, counter = %u\n", epochTime, counter);
        char dateTime[25];
        Serial.printf("GMT   = %s\n", getFormattedTimeISO8601(dateTime, 25));
        Serial.printf("Local = %s\n", getFormattedTime(dateTime, 25));
#endif

    }
}

// the loop function runs over and over again until power down or reset
void loop()
{
    // std::map<int, int> test;

}
