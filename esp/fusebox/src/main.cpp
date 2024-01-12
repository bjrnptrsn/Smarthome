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

#define BUTTON_1_GPIO 13  // D7
#define BUTTON_2_GPIO 14  // D5
#define BUTTON_3_GPIO 12  // D6
#define BUTTON_4_GPIO 5   // D1
#define BUZZER_GPIO   4   // D2

WiFiClient net;
MQTTClient mqtt(1024);
MQTTClient* HAEntity::_mqtt = &mqtt;

OneButton pushbutton1(BUTTON_1_GPIO);
OneButton pushbutton2(BUTTON_2_GPIO);
OneButton pushbutton3(BUTTON_3_GPIO);
OneButton pushbutton4(BUTTON_4_GPIO);

ButtonEntity button1;
ButtonEntity button2;
ButtonEntity button3;
ButtonEntity button4;
HAEntity buzzer;
HAEntity reboot;

enum CONFIG
{
  CONFIG_READ,
  CONFIG_PUBLISH
};

unsigned long minuteTimer = 0;
struct {
  unsigned long timer = 0;
  bool state = false;
  bool set = false;
} buzz;

void onMqttMessage(String &topic, String &payload)
{
  if (topic == reboot.commandTopic() && payload == "PRESS")
    ESP.restart();

  if (topic == buzzer.commandTopic() && payload == "PRESS")
    buzz.set = true;
}

void initButton()
{
  pushbutton1.attachIdle([]() { button1.sendButtonState(ButtonEntity::BUTTON_IDLE); });
  pushbutton1.attachClick([]() { button1.sendButtonState(ButtonEntity::BUTTON_SINGLE); });
  pushbutton1.attachDoubleClick([]() { button1.sendButtonState(ButtonEntity::BUTTON_DOUBLE); });
  pushbutton1.attachMultiClick([]() { button1.sendButtonState(ButtonEntity::BUTTON_MULTI, pushbutton1.getNumberClicks()); });
  pushbutton1.attachLongPressStart([]() { button1.sendButtonState(ButtonEntity::BUTTON_LONG_START); });
  pushbutton1.attachLongPressStop([]() { button1.sendButtonState(ButtonEntity::BUTTON_LONG_STOP); });
  // pushbutton1.attachDuringLongPress([](){ button1.sendButtonState(ButtonEntity::BUTTON_LONG_DURING); });
  
  pushbutton2.attachIdle([]() { button2.sendButtonState(ButtonEntity::BUTTON_IDLE); });
  pushbutton2.attachClick([]() { button2.sendButtonState(ButtonEntity::BUTTON_SINGLE); });
  pushbutton2.attachDoubleClick([]() { button2.sendButtonState(ButtonEntity::BUTTON_DOUBLE); });
  pushbutton2.attachMultiClick([]() { button2.sendButtonState(ButtonEntity::BUTTON_MULTI, pushbutton2.getNumberClicks()); });
  pushbutton2.attachLongPressStart([]() { button2.sendButtonState(ButtonEntity::BUTTON_LONG_START); });
  pushbutton2.attachLongPressStop([]() { button2.sendButtonState(ButtonEntity::BUTTON_LONG_STOP); });
  // pushbutton2.attachDuringLongPress([](){ button2.sendButtonState(ButtonEntity::BUTTON_LONG_DURING); });

  pushbutton3.attachIdle([]() { button3.sendButtonState(ButtonEntity::BUTTON_IDLE); });
  pushbutton3.attachClick([]() { button3.sendButtonState(ButtonEntity::BUTTON_SINGLE); });
  pushbutton3.attachDoubleClick([]() { button3.sendButtonState(ButtonEntity::BUTTON_DOUBLE); });
  pushbutton3.attachMultiClick([]() { button3.sendButtonState(ButtonEntity::BUTTON_MULTI, pushbutton3.getNumberClicks()); });
  pushbutton3.attachLongPressStart([]() { button3.sendButtonState(ButtonEntity::BUTTON_LONG_START); });
  pushbutton3.attachLongPressStop([]() { button3.sendButtonState(ButtonEntity::BUTTON_LONG_STOP); });
  // pushbutton3.attachDuringLongPress([](){ button3.sendButtonState(ButtonEntity::BUTTON_LONG_DURING); });

  pushbutton4.attachIdle([]() { button4.sendButtonState(ButtonEntity::BUTTON_IDLE); });
  pushbutton4.attachClick([]() { button4.sendButtonState(ButtonEntity::BUTTON_SINGLE); });
  pushbutton4.attachDoubleClick([]() { button4.sendButtonState(ButtonEntity::BUTTON_DOUBLE); });
  pushbutton4.attachMultiClick([]() { button4.sendButtonState(ButtonEntity::BUTTON_MULTI, pushbutton4.getNumberClicks()); });
  pushbutton4.attachLongPressStart([]() { button4.sendButtonState(ButtonEntity::BUTTON_LONG_START); });
  pushbutton4.attachLongPressStop([]() { button4.sendButtonState(ButtonEntity::BUTTON_LONG_STOP); });
  // pushbutton4.attachDuringLongPress([](){ button4.sendButtonState(ButtonEntity::BUTTON_LONG_DURING); });
}

void initEntity(int config = CONFIG_READ)
{
  if (config == CONFIG_READ)
  {
    button1.readConfig("/button1.json", "sensor");
    button2.readConfig("/button2.json", "sensor");
    button3.readConfig("/button3.json", "sensor");
    button4.readConfig("/button4.json", "sensor");
    buzzer.readConfig("/relay.json", "button");
    reboot.readConfig("/reboot.json", "button");
  }
  else if (config == CONFIG_PUBLISH)
  {
    button1.publishConfig();
    button2.publishConfig();
    button3.publishConfig();
    button4.publishConfig();
    buzzer.publishConfig();
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
  mqtt.setWill(button1.availabilityTopic().c_str(), "offline", true, 0);
  while (!mqtt.connect(MQTT_CLIENT))
    delay(10);

  mqtt.publish(button1.availabilityTopic(), "online", true, 0);
  mqtt.subscribe(buzzer.commandTopic());
  mqtt.subscribe(reboot.commandTopic());

  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("\n-----------------"));
  Serial.println(F(" This is fusebox"));
  Serial.println(F("-----------------"));

  pinMode(BUZZER_GPIO, OUTPUT);
  digitalWrite(BUZZER_GPIO, HIGH);

  LittleFS.begin();

  ArduinoOTA.setHostname(MQTT_CLIENT);
  ArduinoOTA.onStart([]()
                     { digitalWrite(BUZZER_GPIO, HIGH); });
  ArduinoOTA.begin();

  initButton();
  initEntity();
  initConnect();

  if (connect())
  {
    initEntity(CONFIG_PUBLISH);
    delay(500);
    button1.sendButtonState(ButtonEntity::BUTTON_IDLE);
    button2.sendButtonState(ButtonEntity::BUTTON_IDLE);
    button3.sendButtonState(ButtonEntity::BUTTON_IDLE);
    button4.sendButtonState(ButtonEntity::BUTTON_IDLE);
  }
}

void process()
{
  if (buzz.set)
  {
    if (!buzz.state)
    {
      digitalWrite(BUZZER_GPIO, LOW);
      buzz.state = true;
      buzz.timer = millis();
    }
    else if (millis() - buzz.timer > 1500)
    {
      digitalWrite(BUZZER_GPIO, HIGH);
      buzz.state = false;
      buzz.set = false;
    }
  }

  // every minute
  if (millis() - minuteTimer > 60 * 1000)
  {
    if (pushbutton1.isIdle())
      button1.sendButtonState(ButtonEntity::BUTTON_IDLE);
      
    if (pushbutton2.isIdle())
      button2.sendButtonState(ButtonEntity::BUTTON_IDLE);

    if (pushbutton3.isIdle())
      button3.sendButtonState(ButtonEntity::BUTTON_IDLE);

    if (pushbutton4.isIdle())
      button4.sendButtonState(ButtonEntity::BUTTON_IDLE);

    minuteTimer = millis();
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
    pushbutton1.tick();
    pushbutton2.tick();
    pushbutton3.tick();
    pushbutton4.tick();
    process();
  }
}
