#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <SmarthomeHelper.h>
#include <OneButton.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <SPI.h>
#include "config.h"

#define BME280_SDA_GPIO       0
#define BME280_SCL_GPIO       1
#define TEMT6000_TOP_GPIO     2
#define TEMT6000_BOTTOM_GPIO  3
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
int counter = 0;
unsigned long minuteTimer = 0;

void onMqttMessage(String &topic, String &payload)
{
  if (topic == reboot.commandTopic() && payload == "PRESS")
    ESP.restart();
}

void sendMotion(bool state = false)
{
  StaticJsonDocument<128> doc;

  if (state)
    doc["motion"] = "ON";
  else
    doc["motion"] = "OFF";

  analogReadResolution(12); // 12 bits

  float volts_top = analogRead(TEMT6000_TOP_GPIO) * 3.3 / 4096.0; // 3.3 V
  float volts_bottom = analogRead(TEMT6000_BOTTOM_GPIO) * 3.3 / 4096.0; // 3.3 V

  // float lux_top = volts_top * 200; // 1 µA = 2 lx
  // float lux_bottom = volts_bottom * 200; // 1 µA = 2 lx
  int lux_top = volts_top * 200; // 1 µA = 2 lx
  int lux_bottom = volts_bottom * 200; // 1 µA = 2 lx

  // doc["illuminance_top"] = std::ceil(lux_top * 100) / 100.0;
  // doc["illuminance_bottom"] = std::ceil(lux_bottom * 100) / 100.0;
  doc["illuminance_top"] = lux_top;
  doc["illuminance_bottom"] = lux_bottom;
  
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
  // pushbutton.attachDuringLongPress([](){ buttonEntity.sendButtonState(ButtonEntity::BUTTON_LONG_DURING); });
}

void initEntity(int config = CONFIG_READ)
{
  if (config == CONFIG_READ)
  {
    motionSensor.readConfig("/motion.json", "binary_sensor");
    button.readConfig("/button.json", "sensor");
    temperatureSensor.readConfig("/temperature.json", "sensor");
    humiditySensor.readConfig("/humidity.json", "sensor");
    pressureSensor.readConfig("/pressure.json", "sensor");
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
  mqtt.onMessage(onMqttMessage);
}

bool connect()
{
  Serial.println("\nChecking WiFi...");
  unsigned long wifiDisconnectTimer_ms = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(10);
    if (millis() - wifiDisconnectTimer_ms > 8000)
      ESP.restart();
  }
  wifiDisconnectTimer_ms = 0;

  Serial.println("Connecting MQTT...");
  mqtt.setWill(cputemp.availabilityTopic().c_str(), "offline", true, 0);
  while (!mqtt.connect(MQTT_CLIENT))
    delay(10);

  mqtt.publish(cputemp.availabilityTopic(), "online", true, 0);
  mqtt.subscribe(reboot.commandTopic());

  return true;
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

  pinMode(TEMT6000_TOP_GPIO, INPUT);
  pinMode(TEMT6000_BOTTOM_GPIO, INPUT);
  pinMode(MOTION_GPIO, INPUT);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);

  LittleFS.begin();

  initButton();
  initEntity();
  initConnect();

  ArduinoOTA.setHostname(MQTT_CLIENT);
  ArduinoOTA.onStart([](){});
  ArduinoOTA.begin();

  if (connect())
  {
    initEntity(CONFIG_PUBLISH);
    delay(500);
    sendMotion();
    sendMeasurements();
    button.sendButtonState(ButtonEntity::BUTTON_IDLE);
  }
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
  else if (millis() - minuteTimer > 60 * 1000)
  {
    sendMeasurements();
    if (!motionState) sendMotion();
    if (pushbutton.isIdle()) button.sendButtonState(ButtonEntity::BUTTON_IDLE);
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
    pushbutton.tick();
    process();
  }
}
