/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-mqtt-publish-dht11-dht22-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include <Wire.h>
#include "SHTSensor.h"
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

// WIFI Settings s
#define WIFI_SSID "mejsebo1"
#define WIFI_PASSWORD "byvejen55"

// MQTT Broker Settings
#define MQTT_HOST "broker.hivemq.com"
#define MQTT_PORT 1883
// Topics
#define MQTT_PUB_TEMP "c3698581-d0a0-4c7a-8041-b50f2e80fe53_T"
#define MQTT_PUB_HUM "c3698581-d0a0-4c7a-8041-b50f2e80fe53_H"
#define MQTT_PUB_VAL_G "c3698581-d0a0-4c7a-8041-b50f2e80fe53_VAL_G"
#define MQTT_PUB_MQTT "c3698581-d0a0-4c7a-8041-b50f2e80fe53_MQTT"
#define MQTT_PUB_SET "c3698581-d0a0-4c7a-8041-b50f2e80fe53_SET"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

SHTSensor sht;
// To use a specific sensor instead of probing the bus use this command:
// SHTSensor sht(SHTSensor::SHT3X);

// Variables to hold sensor readings
float temp;
float hum;
float DewPoint;
float deltaT;
float FanOn = 2;
float FanOff = 3;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 30000;        // Interval at which to publish sensor readings

// rounds a number to 2 decimal places
// example: round(3.14159) -> 3.14
double roundX(double value, int X) {
   return (int)(value * (10 * X) + 0.5) / (10.0 * X + 0.0);
}


// Calculate DewPoint **************************************************************************************
void dewP(double temp, double hum)
{
  double A0 = 373.15 / (273.15 + (double)temp);
  double SUM = -7.90298 * (A0 - 1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1);
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1);
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * (double)hum;
  double Td = log(VP / 0.61078); // temp var
  Td = (241.88 * Td) / (17.558 - Td);
  DewPoint = roundX(Td, 2);
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void measureandsend()
{
  // Save the last time a new reading was published
  
  if (sht.readSample()) {
      Serial.print("SHT:\n");
      Serial.print("  RH: ");
      Serial.print(sht.getHumidity(), 2);
      Serial.print("\n");
      Serial.print("  T:  ");
      Serial.print(sht.getTemperature(), 2);
      Serial.print("\n");
  } else {
      Serial.print("Error in readSample()\n");
  }
  
  hum = sht.getHumidity();
  Serial.printf("Humidity: %.2f \n", hum);
  temp = sht.getTemperature();
  Serial.printf("Temperature: %.2f \n", temp);
  dewP((double)temp, (double)hum);
  deltaT = temp - DewPoint;

  double t_temp = roundX(temp, 2);
  double h_temp = roundX(hum, 2);
  double dp_temp = roundX(DewPoint, 2);
  double dt_temp = roundX(deltaT, 2);
  

  String OutputValues;
  StaticJsonDocument<200> doc;
  doc["temp"] = t_temp;   // String(temp, 2);
  doc["hum"] = h_temp;    // String(hum, 2);
  doc["dP"] = dp_temp;    // String(DewPoint, 2);
  doc["dT"] = dt_temp;    // String(deltaT, 2);
  // doc["state"] = "OFF"; // String(digitalRead(Fan1), 0);
  // doc["F-on"] = String(FanOn, 2);
  // doc["F-off"] = String(FanOff, 2);
  serializeJson(doc, OutputValues);
  
  Serial.println();
  Serial.println(OutputValues);
  Serial.println();
  Serial.printf("Message: %.2f degC \n", temp);

  /*
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 2, true, String(temp).c_str());
  Serial.printf("Publishing on topic %s at QoS 2, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
*/
    // Publish an MQTT message on topic esp32/dht/temperature
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
  Serial.printf("Message: %.2f \n", temp);

  // Publish an MQTT message on topic esp32/dht/humidity
  uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
  Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
  Serial.printf("Message: %.2f \n", hum);

  uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_VAL_G, 2, true, OutputValues.c_str());
  Serial.printf("Publishing on topic %s at QoS 2, packetId: %i ", MQTT_PUB_VAL_G, packetIdPub3);
  Serial.printf("Message: %s degC \n", OutputValues.c_str());
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(500); // let serial console settle
  Serial.println();

  if (sht.init()) {
      Serial.print("init(): success\n");
  } else {
      Serial.print("init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x

  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    measureandsend();
  }
}
