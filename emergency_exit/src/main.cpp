#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SmarthomeHelper.h>
#include <OneButton.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <SPI.h>
#include <ChainableLED.h>
#include "config.h"

#define BME280_SCL_GPIO 5 // D1
#define BME280_SDA_GPIO 4 // D2
#define P9813_CIN_GPIO 13 // D7
#define P9813_DIN_GPIO 15 // D8
#define MOTION_GPIO 14    // D5
#define BUTTON_GPIO 12    // D6

WiFiClient net;
MQTTClient mqtt(1024);
MQTTClient *HAEntity::_mqtt = &mqtt;

OneButton pushbutton(BUTTON_GPIO);

ChainableLED led(P9813_CIN_GPIO, P9813_DIN_GPIO, 1);

BME280I2C bme280;

LightEntity light;
ButtonEntity button;
HAEntity motionSensor;
HAEntity temperatureSensor;
HAEntity humiditySensor;
HAEntity pressureSensor;
HAEntity illuminanceSensor;

enum CONFIG
{
  CONFIG_READ,
  CONFIG_PUBLISH
};

unsigned long availabilityTimer;
unsigned long measureTimer;

bool lastMotionState = true;

void checkMotion()
{
  bool motionState = digitalRead(MOTION_GPIO);

  if (motionState != lastMotionState)
  {
    if (motionState)
      mqtt.publish(motionSensor.stateTopic(), "{\"motion\":\"ON\"}");
    else
      mqtt.publish(motionSensor.stateTopic(), "{\"motion\":\"OFF\"}");

    lastMotionState = motionState;
  }
}

void onMqttMessage(String &topic, String &payload)
{
  light.parse(payload);
}

void ambientMeasurements(unsigned long delay_seconds = 300)
{
  if (millis() - measureTimer > delay_seconds * 1000)
  {
    float temperature, humidity, pressure;

    bme280.read(pressure, temperature, humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);
    int illuminance = analogRead(A0);

    StaticJsonDocument<128> doc;
    doc["temperature"] = std::ceil(temperature * 100) / 100.0; // shorten to 2 decimals places
    doc["humidity"] = std::ceil(humidity * 100) / 100.0;
    doc["pressure"] = std::ceil(pressure * 100) / 100.0;
    // doc["pressure"] = int(pressure);
    doc["illuminance"] = illuminance * 5;

    char out[128];
    serializeJson(doc, out);

    mqtt.publish(temperatureSensor.stateTopic(), out);

    measureTimer = millis();
  }
}

void heartbeat(unsigned long delay_seconds = 60)
{
  if (millis() - availabilityTimer > delay_seconds * 1000)
  {
    light.sendState();

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
  // pushbutton.attachDuringLongPress([](){ buttonEntity.sendButtonState(ButtonEntity::BUTTON_LONG_DURING); });
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
  mqtt.setWill(light.availabilityTopic().c_str(), "offline", true, 0);
  while (!mqtt.connect(MQTT_CLIENT))
    delay(10);

  mqtt.publish(light.availabilityTopic(), "online", true, 0);
  mqtt.subscribe(light.commandTopic());

  return true;
}

void setupEntity(int config = CONFIG_READ)
{
  if (config != CONFIG_PUBLISH)
  {
    light.readConfig("/light.json", "light");
    motionSensor.readConfig("/motion.json", "binary_sensor");
    button.readConfig("/button.json", "sensor");
    temperatureSensor.readConfig("/temperature.json", "sensor");
    humiditySensor.readConfig("/humidity.json", "sensor");
    pressureSensor.readConfig("/pressure.json", "sensor");
    illuminanceSensor.readConfig("/illuminance.json", "sensor");
  }
  else
  {
    light.publishConfig();
    motionSensor.publishConfig();
    button.publishConfig();
    temperatureSensor.publishConfig();
    humiditySensor.publishConfig();
    pressureSensor.publishConfig();
    illuminanceSensor.publishConfig();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("\n------------------------"));
  Serial.println(F(" This is emergency_exit"));
  Serial.println(F("------------------------"));

  pinMode(MOTION_GPIO, INPUT);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);

  LittleFS.begin();

  setupConnect();

  setupEntity();

  setupButton();

  if (connect())
  {
    setupEntity(CONFIG_PUBLISH);
    heartbeat(1);
  }

  led.setColorRGB(0, 0, 0, 0);

  ArduinoOTA.setHostname(MQTT_CLIENT);
  ArduinoOTA.onStart([]()
                     { led.setColorRGB(0, 0, 0, 0); });

  ArduinoOTA.begin();

  Wire.begin(BME280_SDA_GPIO, BME280_SCL_GPIO);
  bme280.begin();
}

void loop()
{
  ArduinoOTA.handle();

  mqtt.loop();

  if (!mqtt.connected())
    connect();
  else
  {
    heartbeat();
    ambientMeasurements(15);
  }

  light.run();
  led.setColorRGB(0, light.brightness(), 0, 0);

  pushbutton.tick();

  checkMotion();
}
