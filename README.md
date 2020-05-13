# Azure-IoT-Garden-Irrigation

BACKGROUND

Develop a multi-tier micro-controller (SoC) concept containing sensors, a sensor hub and controllers. The project challenges are the development of autonomous sensors with very low consumption, a hub capable of communicating with various sensors within medium range distance (100m) and feeding the Microsoft Azure cloud, as well as controllers that can be activated remotely by manual actions or through a machine learning service.

TECHNOLOGY

The choice of technology has been based on knowledge and experience, along with the project pre-requisites. The sensors, due to the low consumption nature of the project, are built around Arduino Pro Mini and 32u4, the original hub has been developed with ESP8266 for its Wi-Fi capability as well its 16-bit architecture and its large memory capacity, the second revision use a ESP32 and multi-core capabilities. The controllers are construct around the Arduino MKR1000 for their large IO and Wi-Fi connectivity as well as its capability of handling battery redundancy.

SENSOR – AIR & SOIL TEMPERATURE & HUMIDITY

The sensor core are build around the Arduino Pro Mini equipped with an Atmel 368P 3.3V 8MHz and the 32u4 along with a 433MHz HC-12 Wireless transmitter.  The sensors are a DHT22 temperature and humidity sensor for the outide environment and an industrial soil sensor SENSIRION SHT20.
