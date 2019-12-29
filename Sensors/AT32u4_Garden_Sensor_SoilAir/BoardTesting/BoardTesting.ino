/*
 Name:		  BoardTesting.ino
 Created:	  27/12/2019
 Last update: 28/09/2019
 Version: 0.1
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

//#define DEBUG_HUB;

//#include <SoftwareSerial.h>
//#include <LowPower.h>
#include <DHT.h>
#include <DFRobot_SHT20.h>

#define VBATPIN A9
#define RSTPIN A0
#define HC12RST A3
#define HC12CUR A1
#define HC12SETPIN A2

DFRobot_SHT20 sht20;

const char atSleep[] = { 'O','K','+','S','L','E','E','P','\r','\n' };

// Arg receive, transmit
//SoftwareSerial softSerial(11, 10);

#define DHTPIN 12 // what digital pin we're connected to
#define DHTTYPE DHT22

DHT dht22(DHTPIN, DHTTYPE);
float air_temperature = 0, air_humidity = 0, soil_temperature = 0, soil_humidity = 0, bat_voltage = 0;

// the setup function runs once when you press reset or power the board
void setup() {
    Serial1.begin(9600);
    Serial.begin(19200);
    Serial.println("** Setup **");

    // Set the HC-12 in transmitter mode and prepare all other input/output
    digitalWrite(RSTPIN, HIGH);
    digitalWrite(HC12RST, HIGH);
    pinMode(RSTPIN, OUTPUT);
    pinMode(HC12RST, OUTPUT);
    pinMode(HC12SETPIN, OUTPUT);
    pinMode(HC12CUR, INPUT);
    digitalWrite(HC12SETPIN, HIGH);
    pinMode(13, OUTPUT);
 
    // measure the voltage of the battery, add a pre read and a delay to stabilize the reading
    analogRead(VBATPIN);
    delay(10);
    bat_voltage = analogRead(VBATPIN);
    bat_voltage *= 2;    // divided by 2 on the board, so multiply back
    bat_voltage *= 3.3;  // Multiply by 3.3V, the reference voltage
    bat_voltage /= 1024; // convert to voltage
    Serial.print("Voltage: ");
    Serial.print(bat_voltage);
    Serial.println();

    // measure the current used by the HC12
    Serial.print("HC12 current: ");
    Serial.println(analogRead(HC12CUR) * 3);
    Serial.println();

    // Swith the HC12 off
    digitalWrite(HC12RST, LOW);

    // measure the current used by the HC12
    Serial.print("HC12 current: ");
    Serial.println(analogRead(HC12CUR) * 3);
    Serial.println();

    delay(500);

    // Swith the HC12 off
    digitalWrite(HC12RST, HIGH);

    delay(250);

    // measure the current used by the HC12
    Serial.print("HC12 current: ");
    Serial.println(analogRead(HC12CUR) * 3);
    Serial.println();

    // start the sensors
    dht22.begin();

    // Init SHT20 Sensor
    sht20.initSHT20();

    delay(2000);

    // Reading temperature or humidity takes about 250 milliseconds!
    air_humidity = dht22.readHumidity();
    // Read temperature as Celsius (the default)
    air_temperature = dht22.readTemperature();
    // get the SHT20 sensor data
    soil_humidity = sht20.readHumidity();
    soil_temperature = sht20.readTemperature();

    Serial.print("DHT22 - Humidity: ");
    Serial.println(air_humidity);
    Serial.print("DHT22 - Temp: ");
    Serial.println(air_temperature);
    Serial.print("SHT20 - Humidity: ");
    Serial.println(soil_humidity);
    Serial.print("SHT20 - Temp: ");
    Serial.println(soil_temperature);

    SetHC12ToSleep();
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

    Serial.print("HC-12 response 200 = ");
    Serial.println(hc12Response);
    Serial.print("HC12 current = ");
    Serial.println(analogRead(HC12CUR) * 3);

    // test if the HC12 is sleeping by measuring the drain current
    if (analogRead(HC12CUR) == 0) return 1;
    else return 0;
}

// the loop function runs over and over again until power down or reset
void loop() {

}
