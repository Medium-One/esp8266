/***************************************************************************
  This is an example of program for connected the Adafruit Huzzah and BMP280
  to the Medium One Prototyping Sandbox.  Visit www.medium.one for more information.
  Author: Medium One
  Last Revision Date: May 1, 2018

  The program includes a library and portions of sample code from Adafruit
  with their description below:
  
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout 
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
 
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bmp; // I2C

// ongoing timer counter for heartbeat
static int heartbeat_timer = 0;

// ongoing timer counter for sensor
static int sensor_timer = 0;

// set heartbeat period in milliseconds
static int heartbeat_period = 60000;

// set sensor transmit period in milliseconds
static int sensor_period = 5000;

// track time when last connection error occurs
long lastReconnectAttempt = 0;

// set pin for LED
const int LED_PIN = 2;

// wifi client with security
WiFiClientSecure wifiClient; 

// MQTT Connection info
//char server[] = "mqtt.mediumone.com";
//int port = 61620;
//char pub_topic[]="0/<Project MQTT ID>/<User MQTT>/esp8266/";
//char sub_topic[]="1/<Project MQTT ID>/<User MQTT>/esp8266/event";
//char mqtt_username[]="<Project MQTT ID>/<User MQTT>";
//char mqtt_password[]="<API Key>/<User Password>";

char server[] = "mqtt.mediumone.com";
int port = 61620;
char pub_topic[]="0/rHG62eujmak/HifeH9TftgA/device/";
char sub_topic[]="1/rHG62eujmak/HifeH9TftgA/device/event";
char mqtt_username[]="rHG62eujmak/HifeH9TftgA";
char mqtt_password[]="3DPALTEM4WLUJ2NRVSPN67ZQGVQWIMRVMVSWGYRQGQ4DAMBQ/cloudJam1!";

void setup() {
  
  // init uart
  Serial.begin(9600);
  while(!Serial){}

  // wifi setup
  WiFi.mode(WIFI_STA);

  // WiFi.begin("<WIFI SSID>", "<WIFI_PASSWORD>");
  WiFi.begin("mediumone", "cccccccc");

  //not sure this is needed
  delay(5000);
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Failed to connect, resetting"));
    ESP.reset();
  }

  // optinally
  //while (WiFi.status() != WL_CONNECTED) {}
  
  // if you get here you have connected to the WiFi
  Serial.println(F("Connected to Wifi!"));

  Serial.println(F("Init hardware LED"));
  pinMode(LED_PIN, OUTPUT);
 
  // connect to MQTT broker to send board reset msg
  connectMQTT();

  // connect to BMP sensor board
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  Serial.println("Complete Setup");
}

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  int i = 0;
  char message_buff[length + 1];
  for(i=0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  
  Serial.print(F("Received some data: "));
  Serial.println(String(message_buff));
  
  // Process message to turn LED on and off
  if (String(message_buff[0]) == "0") { 
    // Turn off LED
    digitalWrite(LED_PIN, HIGH);
  } else if (String(message_buff[0]) == "1") { 
    // Turn on LED
    digitalWrite(LED_PIN, LOW);
  }
}

PubSubClient client(server, port, callback, wifiClient);

boolean connectMQTT()
{    
  // Important Note: MQTT requires a unique id (UUID), we are using the mqtt_username as the unique ID
  // Besure to create a new device ID if you are deploying multiple devices.
  // Learn more about Medium One's self regisration option on docs.mediumone.com
  if (client.connect((char*) mqtt_username,(char*) mqtt_username, (char*) mqtt_password)) {
    Serial.println(F("Connected to MQTT broker"));

    // send a connect message
    if (client.publish((char*) pub_topic, "{\"event_data\":{\"mqtt_connected\":true}}")) {
      Serial.println("Publish connected message ok");
    } else {
      Serial.print(F("Publish connected message failed: "));
      Serial.println(String(client.state()));
    }

    // subscrive to MQTT topic
    if (client.subscribe((char *)sub_topic,1)){
      Serial.println(F("Successfully subscribed"));
    } else {
      Serial.print(F("Subscribed failed: "));
      Serial.println(String(client.state()));
    }
  } else {
    Serial.println(F("MQTT connect failed"));
    Serial.println(F("Will reset and try again..."));
    abort();
  }
  return client.connected();
}

void loop() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 1000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (connectMQTT()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    client.loop();
  }
  heartbeat_loop();
  bmp_loop();
}

void bmp_loop() {
  if ((millis()- sensor_timer) > sensor_period) {
    sensor_timer = millis();

    String payload = "{\"event_data\":{\"temperature\":";
    payload += bmp.readTemperature();
    payload += ",\"pressure\":";
    payload += bmp.readPressure();
    payload += ",\"altitude\":";
    payload += bmp.readAltitude(1013.25);
    payload += "}}";
    
    if (client.loop()){
      Serial.print(F("Sending sensor: "));
      Serial.println(payload);
  
      if (client.publish((char *) pub_topic, (char*) payload.c_str()) ) {
        Serial.println("Publish ok");
      } else {
        Serial.print(F("Failed to publish sensor data: "));
        Serial.println(String(client.state()));
      }
    }
    
    Serial.println();
    delay(2000);
  }
}

void heartbeat_loop() {
  if ((millis()- heartbeat_timer) > heartbeat_period) {
    heartbeat_timer = millis();
    String payload = "{\"event_data\":{\"millis\":";
    payload += millis();
    payload += ",\"heartbeat\":true}}";
    
    if (client.loop()){
      Serial.print(F("Sending heartbeat: "));
      Serial.println(payload);
  
      if (client.publish((char *) pub_topic, (char*) payload.c_str()) ) {
        Serial.println(F("Publish ok"));
      } else {
        Serial.print(F("Failed to publish heartbeat: "));
        Serial.println(String(client.state()));
      }
    }
  }
}

