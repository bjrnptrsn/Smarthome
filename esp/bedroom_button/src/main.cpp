#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SmarthomeHelper.h>
#include <OneButton.h>
#include "config.h"

#define BUTTON_GPIO 0

WiFiClient net;
MQTTClient mqtt(1024);
MQTTClient* HAEntity::_mqtt = &mqtt;

OneButton pushbutton(BUTTON_GPIO);

ButtonEntity button;
HAEntity reboot;
HAEntity cputemp;

enum CONFIG
{
  CONFIG_READ,
  CONFIG_PUBLISH
};

unsigned long timer = 0;

void onMqttMessage(String &topic, String &payload)
{
  if (topic == reboot.commandTopic() && payload == "PRESS")
    ESP.restart();
}

void sendMeasurements()
{
  StaticJsonDocument<128> doc;

  doc["cpu_temp"] = std::ceil(temperatureRead() * 100) / 100.0;

  char out[128];
  serializeJson(doc, out);

  mqtt.publish(cputemp.stateTopic(), out);
}

void initButton()
{
  pushbutton.attachIdle([]() { button.sendButtonState(ButtonEntity::BUTTON_IDLE); });
  pushbutton.attachClick([]() { button.sendButtonState(ButtonEntity::BUTTON_SINGLE); });
  pushbutton.attachDoubleClick([]() { button.sendButtonState(ButtonEntity::BUTTON_DOUBLE); });
  pushbutton.attachMultiClick([]() { button.sendButtonState(ButtonEntity::BUTTON_MULTI, pushbutton.getNumberClicks()); });
  pushbutton.attachLongPressStart([]() { button.sendButtonState(ButtonEntity::BUTTON_LONG_START); });
  pushbutton.attachLongPressStop([]() { button.sendButtonState(ButtonEntity::BUTTON_LONG_STOP); });
  // pushbutton.attachDuringLongPress([](){ button.sendButtonState(ButtonEntity::BUTTON_LONG_DURING); });
}

void initEntity(int config = CONFIG_READ)
{
  if (config == CONFIG_READ)
  {
    button.readConfig("/button.json", "sensor");
    reboot.readConfig("/reboot.json", "button");
    cputemp.readConfig("/cputemp.json", "sensor");
  }
  else if (config == CONFIG_PUBLISH)
  {
    button.publishConfig();
    reboot.publishConfig();
    cputemp.publishConfig();
  }
}

void initConnect()
{
  WiFi.disconnect(true);
  WiFi.setSleep(false);
  WiFi.setHostname(MQTT_CLIENT);
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
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
  mqtt.setWill(cputemp.availabilityTopic().c_str(), "offline", true, 0);
  while (!mqtt.connect(MQTT_CLIENT))
    delay(10);
  
  mqtt.publish(cputemp.availabilityTopic(), "online", true ,0);
  mqtt.subscribe(reboot.commandTopic());

  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("\n------------------------"));
  Serial.println(F(" This is bedroom_button"));
  Serial.println(F("------------------------"));

  LittleFS.begin();

  initButton();
  initEntity();
  initConnect();

  ArduinoOTA.setHostname(MQTT_CLIENT);
  ArduinoOTA.begin();

  if (connect())
  {
    initEntity(CONFIG_PUBLISH);
    delay(500);
    sendMeasurements();
    button.sendButtonState(ButtonEntity::BUTTON_IDLE);
  }
}

void process()
{
  // heartbeat
  if (millis() - timer > 60 * 1000)
  {
    sendMeasurements();
    if (pushbutton.isIdle()) button.sendButtonState(ButtonEntity::BUTTON_IDLE);

    timer = millis();
  }
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
    process();
  }
}
