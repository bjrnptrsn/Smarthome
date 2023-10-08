#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SmarthomeHelper.h>
#include <OneButton.h>
#include "config.h"

#define BUTTON_GPIO 5   // D1

WiFiClient net;
MQTTClient mqtt(1024);
MQTTClient* HAEntity::_mqtt = &mqtt;

OneButton pushbutton(BUTTON_GPIO);

ButtonEntity button;

enum CONFIG
{
  CONFIG_READ,
  CONFIG_PUBLISH
};

unsigned long availabilityTimer;

void onMqttMessage(String &topic, String &payload)
{
  Serial.println("\nmsg incoming:");
  Serial.println(payload);

  StaticJsonDocument<64> jsonDoc;
  deserializeJson(jsonDoc, payload);
}

void heartbeat(unsigned long delay_seconds = 60)
{
  if (millis() - availabilityTimer > delay_seconds * 1000)
  {
    if (pushbutton.isIdle())
      button.sendButtonState(ButtonEntity::BUTTON_IDLE);

    availabilityTimer = millis();
  }
}

void setupButton()
{
  pushbutton.attachIdle([]() { button.sendButtonState(ButtonEntity::BUTTON_IDLE); });
  pushbutton.attachClick([]() { button.sendButtonState(ButtonEntity::BUTTON_SINGLE); });
  pushbutton.attachDoubleClick([]() { button.sendButtonState(ButtonEntity::BUTTON_DOUBLE); });
  pushbutton.attachMultiClick([]() { button.sendButtonState(ButtonEntity::BUTTON_MULTI, pushbutton.getNumberClicks()); });
  pushbutton.attachLongPressStart([]() { button.sendButtonState(ButtonEntity::BUTTON_LONG_START); });
  pushbutton.attachLongPressStop([]() { button.sendButtonState(ButtonEntity::BUTTON_LONG_STOP); });
  // pushbutton.attachDuringLongPress([](){ button.sendButtonState(ButtonEntity::BUTTON_LONG_DURING); });
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
  mqtt.setWill(button.availabilityTopic().c_str(), "offline", true, 0);
  while (!mqtt.connect(MQTT_CLIENT))
    delay(10);
  
  mqtt.publish(button.availabilityTopic(), "online", true ,0);

  return true;
}

void setupEntity(int config = CONFIG_READ)
{
  if (config != CONFIG_PUBLISH)
    button.readConfig("/button.json", "sensor");
  else
    button.publishConfig();
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("\n------------------------"));
  Serial.println(F(" This is bedroom_button"));
  Serial.println(F("------------------------"));

  LittleFS.begin();

  setupConnect();

  setupEntity();

  if (connect())
  {
    setupEntity(CONFIG_PUBLISH);
    heartbeat(1);
  }

  ArduinoOTA.setHostname(MQTT_CLIENT);
  ArduinoOTA.begin();

  setupButton();
}

void loop()
{
  ArduinoOTA.handle();

  mqtt.loop();

  if (!mqtt.connected())
    connect();
  else
  {
    pushbutton.tick();
    heartbeat();
  }
}
