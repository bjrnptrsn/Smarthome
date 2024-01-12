#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SmarthomeHelper.h>
#include "config.h"

#define ZC_GPIO 4
#define PWM_GPIO 5

WiFiClient net;
MQTTClient mqtt(1024);
MQTTClient* HAEntity::_mqtt = &mqtt;

DimmerEntity dimmer(ZC_GPIO, PWM_GPIO);
HAEntity reboot;

enum CONFIG
{
  CONFIG_READ,
  CONFIG_PUBLISH
};

unsigned long timer;

void onMqttMessage(String &topic, String &payload)
{
  if (topic.equals(reboot.commandTopic()) && payload == "PRESS")
    ESP.restart();

  if (topic.equals(dimmer.commandTopic()))
    dimmer.parse(payload);
}

void initEntity(int config = CONFIG_READ)
{
  if (config == CONFIG_READ)
  {
    dimmer.readConfig("/dimmer.json", "light");
    reboot.readConfig("/reboot.json", "button");
  }
  else if (config == CONFIG_PUBLISH)
  {
    dimmer.publishConfig();
    reboot.publishConfig();
  }
}

void initConnect()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setHostname(MQTT_CLIENT);
  WiFi.persistent(true);
  WiFi.begin(SSID, PASS);

  mqtt.begin(MQTT_BROKER, net);
  mqtt.onMessage(onMqttMessage);
}

bool connect()
{
  Serial.println("\nchecking WiFi...");
  unsigned long wifiDisconnectTimer_ms = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(10);
    if (millis() - wifiDisconnectTimer_ms > 8000)
      ESP.restart();
  }
  wifiDisconnectTimer_ms = 0;

  Serial.println("connecting MQTT...");
  mqtt.setWill(dimmer.availabilityTopic().c_str(), "offline", true, 0);
  while (!mqtt.connect(MQTT_CLIENT))
    delay(10);
  
  mqtt.publish(dimmer.availabilityTopic(), "online", true, 0);
  mqtt.subscribe(dimmer.commandTopic());
  mqtt.subscribe(reboot.commandTopic());

  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println("\n---------------------------");
  Serial.println(" This is ac_dimmer_bath");
  Serial.println("---------------------------");

  LittleFS.begin();

  initConnect();
  initEntity();

  if (connect())
  {
    initEntity(CONFIG_PUBLISH);
    delay(500);
    dimmer.sendState();
  }

  ArduinoOTA.setHostname(MQTT_CLIENT);
  ArduinoOTA.onStart([]() {});

  ArduinoOTA.begin();
}

void process()
{
  // heartbeat
  if (millis() - timer > 5 * 60 * 1000)
  {
    dimmer.sendState();

    timer = millis();
  }
}

void loop()
{
  ArduinoOTA.handle();

  dimmer.run();

  mqtt.loop();

  if (!mqtt.connected())
    connect();
  else
    process();
}
