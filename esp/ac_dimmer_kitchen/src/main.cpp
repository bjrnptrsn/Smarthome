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
MQTTClient *HAEntity::_mqtt = &mqtt;

DimmerEntity dimmer(ZC_GPIO, PWM_GPIO);

const char* dimmer_json = "{\"availability_topic\":\"home/kitchen/dimmer\",\"device\":{\"identifiers\":[\"kitchen_ac_dimmer_8266\"],\"manufacturer\":\"bjrnptrsn\",\"model\":\"ac_dimmer(esp8266)\",\"name\":\"kitchen.dimmer\",\"sw_version\":\"ac_dimmer_v1\"},\"icon\":\"mdi:ceiling-light\",\"name\":\"kitchen.dimmer\",\"schema\":\"json\",\"state_topic\":\"home/kitchen/dimmer/lamp\",\"command_topic\":\"home/kitchen/dimmer/lamp/set\",\"brightness\":true,\"brightness_scale\":100,\"unique_id\":\"kitchen_dimmer_light\"}";

enum CONFIG
{
  CONFIG_READ,
  CONFIG_PUBLISH
};

unsigned long availabilityTimer;

void heartbeat(unsigned long delay_seconds = 60)
{
  if (millis() - availabilityTimer > delay_seconds * 1000)
  {
    dimmer.sendState();

    availabilityTimer = millis();
  }
}

void onMqttMessage(String &topic, String &payload)
{
  if (topic.equals(dimmer.commandTopic()))
    dimmer.parse(payload);
}

void setupEntity(int config = CONFIG_READ)
{
  if (config != CONFIG_PUBLISH)
    dimmer.readConfig("/dimmer.json", "light");
    // dimmer.readConfig(dimmer_json, "light");
  else
    dimmer.publishConfig();
}

void setupConnect()
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

  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n---------------------------");
  Serial.println(" This is kitchen_ac_dimmer");
  Serial.println("---------------------------");

  LittleFS.begin();

  setupConnect();
  setupEntity();

  if (connect())
  {
    setupEntity(CONFIG_PUBLISH);
    heartbeat(1);
  }

  ArduinoOTA.setHostname(MQTT_CLIENT);
  ArduinoOTA.onStart([]() {});

  ArduinoOTA.begin();
}

void loop()
{
  ArduinoOTA.handle();

  if (!mqtt.connected())
    connect();
  else
    heartbeat();

  mqtt.loop();

  dimmer.run();
}
