#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SmarthomeHelper.h>
#include <OneButton.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>
#include <Wire.h>
#include <SPI.h>
#include "config.h"

#define TEMT6000_TOP_GPIO     0
#define TEMT6000_BOTTOM_GPIO  1
#define BME280_SDA_GPIO       2
#define BME280_SCL_GPIO       3
#define BUTTON_GPIO           5
#define MOTION_GPIO           6

WiFiClient net;
MQTTClient mqtt(1024);
MQTTClient *HAEntity::_mqtt = &mqtt;

OneButton pushbutton(BUTTON_GPIO);

BME280I2C bme280;

ButtonEntity button;
HAEntity motionSensor;
HAEntity temperatureSensor;
HAEntity humiditySensor;
HAEntity pressureSensor;
HAEntity dewpointSensor;
HAEntity illuminanceSensorTop;
HAEntity illuminanceSensorBottom;
HAEntity reboot;
HAEntity cputemp;

enum CONFIG
{
  CONFIG_READ,
  CONFIG_PUBLISH
};

bool lastMotionState = false;
unsigned long processTimer = 0;
unsigned long connectionTimer = 0;

void onMqttMessage(String &topic, String &payload)
{
  if (topic == reboot.commandTopic() && payload == "PRESS")
    ESP.restart();
}

void sendMotion(bool state = false)
{
  JsonDocument doc;

  if (state)
    doc["motion"] = "ON";
  else
    doc["motion"] = "OFF";

  analogReadResolution(12); // 12 bits

  float volts_top = analogRead(TEMT6000_TOP_GPIO) * 3.3 / 4096.0; // 3.3 V
  float volts_bottom = analogRead(TEMT6000_BOTTOM_GPIO) * 3.3 / 4096.0; // 3.3 V

  int lux_top = volts_top * 200; // 1 µA = 2 lx
  int lux_bottom = volts_bottom * 200; // 1 µA = 2 lx
  doc["illuminance_top"] = lux_top;
  doc["illuminance_bottom"] = lux_bottom;

  // float lux_top = volts_top * 200; // 1 µA = 2 lx
  // float lux_bottom = volts_bottom * 200; // 1 µA = 2 lx
  // doc["illuminance_top"] = std::ceil(lux_top * 100) / 100.0;
  // doc["illuminance_bottom"] = std::ceil(lux_bottom * 100) / 100.0;
  
  char out[128];
  serializeJson(doc, out);
  
  mqtt.publish(motionSensor.stateTopic(), out);
}
  
void sendMeasurements()
{
  JsonDocument doc;

  float temperature, humidity, pressure;
  bme280.read(pressure, temperature, humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);
  
  doc["temperature"] = std::round(temperature * 100) / 100.0; // shorten to 2 decimals places
  doc["humidity"] = std::round(humidity * 100) / 100.0;
  doc["pressure"] = std::round(pressure * 100) / 100.0;
  doc["dewpoint"] = std::round(EnvironmentCalculations::DewPoint(temperature, humidity) * 100) / 100.0;
  doc["cpu_temp"] = std::round(temperatureRead() * 100) / 100.0;

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
  // pushbutton.attachDuringLongPress([](){ buttonEntity.sendButtonState(ButtonEntity::BUTTON_LONG_DURING); });
}

void initEntity(int config = CONFIG_READ)
{
  if (config == CONFIG_READ)
  {
    motionSensor.readConfig("/motion.json", "binary_sensor");
    button.readConfig("/button.json", "event");
    temperatureSensor.readConfig("/temperature.json", "sensor");
    humiditySensor.readConfig("/humidity.json", "sensor");
    pressureSensor.readConfig("/pressure.json", "sensor");
    dewpointSensor.readConfig("/dewpoint.json", "sensor");
    illuminanceSensorTop.readConfig("/illuminanceTop.json", "sensor");
    illuminanceSensorBottom.readConfig("/illuminanceBottom.json", "sensor");
    reboot.readConfig("/reboot.json", "button");
    cputemp.readConfig("/cputemp.json", "sensor");
  }
  else if (config == CONFIG_PUBLISH)
  {
    motionSensor.publishConfig();
    button.publishConfig();
    temperatureSensor.publishConfig();
    humiditySensor.publishConfig();
    pressureSensor.publishConfig();
    dewpointSensor.publishConfig();
    illuminanceSensorTop.publishConfig();
    illuminanceSensorBottom.publishConfig();
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
  Serial.println(F(" This is emergency_exit"));
  Serial.println(F("------------------------"));

  Wire.begin(BME280_SDA_GPIO, BME280_SCL_GPIO);
  bme280.begin();

  pinMode(MOTION_GPIO, INPUT);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);

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
  sendMotion();
  sendMeasurements();
  button.sendButtonState(ButtonEntity::BUTTON_IDLE);
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
  else if (millis() - processTimer > 60 * 1000)
  {
    sendMeasurements();
    if (!motionState) sendMotion();
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
