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

bool lastMotionState = false;
int counter = 0;
unsigned long timer = 0;

void onMqttMessage(String &topic, String &payload)
{
  light.parse(payload);
}

void sendMotion(bool state = false)
{
  StaticJsonDocument<128> doc;

  if (state)
    doc["motion"] = "ON";
  else
    doc["motion"] = "OFF";
    
  int illuminance = analogRead(A0);
  doc["illuminance"] = illuminance * 5;
  
  char out[128];
  serializeJson(doc, out);
  
  mqtt.publish(motionSensor.stateTopic(), out);
}
  
void sendMeasurements()
{
  StaticJsonDocument<128> doc;

  float temperature, humidity, pressure;
  bme280.read(pressure, temperature, humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);
   
  doc["temperature"] = std::ceil(temperature * 100) / 100.0; // shorten to 2 decimals places
  doc["humidity"] = std::ceil(humidity * 100) / 100.0;
  doc["pressure"] = std::ceil(pressure * 100) / 100.0;

  char out[128];
  serializeJson(doc, out);

  mqtt.publish(temperatureSensor.stateTopic(), out);
}

void process()
{
  bool motionState = digitalRead(MOTION_GPIO);

  if (motionState != lastMotionState)
  {
    sendMotion(motionState);
    lastMotionState = motionState;
  }

  // every minute
  else if (millis() - timer > 60 * 1000)
  {
    sendMeasurements();

    // heartbeat
    counter++;
    if (counter == 5)
    {
      light.sendState();
      if (pushbutton.isIdle()) button.sendButtonState(ButtonEntity::BUTTON_IDLE);
      counter = 0;
    }

    timer = millis();
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
  delay(500);

  Serial.println();
  Serial.println(F("\n------------------------"));
  Serial.println(F(" This is emergency_exit"));
  Serial.println(F("------------------------"));

  led.setColorRGB(0, 0, 0, 0);

  pinMode(MOTION_GPIO, INPUT);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);

  Wire.begin(BME280_SDA_GPIO, BME280_SCL_GPIO);
  bme280.begin();

  LittleFS.begin();

  ArduinoOTA.setHostname(MQTT_CLIENT);
  ArduinoOTA.onStart([]()
                     { led.setColorRGB(0, 0, 0, 0); });
  ArduinoOTA.begin();

  setupConnect();
  setupEntity();
  setupButton();

  if (connect())
  {
    setupEntity(CONFIG_PUBLISH);
    delay(500);
    sendMotion();
    sendMeasurements();
    button.sendButtonState(ButtonEntity::BUTTON_IDLE);
  }
}

void loop()
{
  led.setColorRGB(0, light.brightness(), 0, 0);
  light.run();

  pushbutton.tick();

  ArduinoOTA.handle();

  mqtt.loop();

  if (!mqtt.connected())
    connect();
  else
    process();
}
