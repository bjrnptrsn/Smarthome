#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SmarthomeHelper.h>
#include <OneButton.h>
#include "config.h"

static constexpr int BUTTON_GPIO = 0;

static constexpr unsigned long SECOND_MS          = 1000;
static constexpr unsigned long REBOOT_BLOCKING_MS = 5  * SECOND_MS;
static constexpr unsigned long CONNECT_TIMEOUT_MS = 8  * SECOND_MS; 
static constexpr unsigned long MINUTE_MS          = 60 * SECOND_MS;

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

unsigned long lastConnectEventTime = 0;
unsigned long processTimer = 0;

void onMqttMessage(String &topic, String &payload)
{
  if (topic == reboot.commandTopic() && payload == "PRESS" && millis() - lastConnectEventTime >= REBOOT_BLOCKING_MS)
    ESP.restart();
}

void sendMeasurements()
{
  JsonDocument doc;

  doc["cpu_temp"] = std::round(temperatureRead() * 100) / 100.0f;

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
    button.readConfig("/button.json", "event");
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
  mqtt.setWill(cputemp.availabilityTopic().c_str(), "offline", true, 0);
  mqtt.onMessage(onMqttMessage);
}

void connect()
{
  lastConnectEventTime = millis();

  Serial.print("\nChecking WiFi and MQTT connection...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (millis() - lastConnectEventTime >= CONNECT_TIMEOUT_MS)
      ESP.restart();
  }
  Serial.println("\nWiFi connected.");

  mqtt.connect(MQTT_CLIENT);
  delay(100);

  if (mqtt.connected())
  {
    lastConnectEventTime = millis();
    mqtt.publish(cputemp.availabilityTopic(), "online", true, 0);
    mqtt.subscribe(reboot.commandTopic());
    Serial.println("MQTT connected.");
  }
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

  connect();

  ArduinoOTA.setHostname(MQTT_CLIENT);
  ArduinoOTA.onStart([](){});
  ArduinoOTA.begin();

  initEntity(CONFIG_PUBLISH);
  delay(100);

  // sending initial states
  sendMeasurements();
  button.sendButtonState(ButtonEntity::BUTTON_IDLE);
}

void process()
{
  // heartbeat
  if (millis() - processTimer >= MINUTE_MS )
  {
    sendMeasurements();
    processTimer = millis();
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
