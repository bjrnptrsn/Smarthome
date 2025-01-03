#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SmarthomeHelper.h>
#include <OneButton.h>
#include "config.h"

#define BUTTON_1_GPIO   3
#define BUTTON_2_GPIO   2
#define BUTTON_3_GPIO   1
#define BUTTON_4_GPIO   0
#define BUZZER_GPIO     4

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
HAEntity cputemp;

enum CONFIG
{
  CONFIG_READ,
  CONFIG_PUBLISH
};

unsigned long connectionTimer = 0;
unsigned long processTimer = 0;

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

void sendMeasurements()
{
  JsonDocument doc;

  doc["cpu_temp"] = std::round(temperatureRead() * 100) / 100.0; // shorten to 2 decimals places

  char out[128];
  serializeJson(doc, out);

  mqtt.publish(cputemp.stateTopic(), out);
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
    button1.readConfig("/button1.json", "event");
    button2.readConfig("/button2.json", "event");
    button3.readConfig("/button3.json", "event");
    button4.readConfig("/button4.json", "event");
    buzzer.readConfig("/relay.json", "button");
    reboot.readConfig("/reboot.json", "button");
    cputemp.readConfig("/cputemp.json", "sensor");
  }
  else if (config == CONFIG_PUBLISH)
  {
    button1.publishConfig();
    button2.publishConfig();
    button3.publishConfig();
    button4.publishConfig();
    buzzer.publishConfig();
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
  connectionTimer = millis();

  Serial.print("\nChecking WiFi and MQTT connection...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (millis() - connectionTimer > 8000)
      ESP.restart();
  }
  Serial.println("\nWiFi connected.");

  mqtt.connect(MQTT_CLIENT);
  delay(100);

  if (mqtt.connected())
  {
    mqtt.publish(cputemp.availabilityTopic(), "online", true, 0);
    mqtt.subscribe(buzzer.commandTopic());
    mqtt.subscribe(reboot.commandTopic());
    Serial.println("MQTT connected.");
  }
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
  digitalWrite(BUZZER_GPIO, LOW);

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
  button1.sendButtonState(ButtonEntity::BUTTON_IDLE);
  button2.sendButtonState(ButtonEntity::BUTTON_IDLE);
  button3.sendButtonState(ButtonEntity::BUTTON_IDLE);
  button4.sendButtonState(ButtonEntity::BUTTON_IDLE);
  sendMeasurements();
}

void process()
{
  if (buzz.set)
  {
    if (!buzz.state)
    {
      digitalWrite(BUZZER_GPIO, HIGH);
      buzz.state = true;
      buzz.timer = millis();
    }
    else if (millis() - buzz.timer > 1500)
    {
      digitalWrite(BUZZER_GPIO, LOW);
      buzz.state = false;
      buzz.set = false;
    }
  }

  // every minute
  if (millis() - processTimer > 60 * 1000)
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
    pushbutton1.tick();
    pushbutton2.tick();
    pushbutton3.tick();
    pushbutton4.tick();
    process();
  }
}
