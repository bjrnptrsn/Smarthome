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

static constexpr int TEMT6000_TOP_GPIO    = 0;
static constexpr int TEMT6000_BOTTOM_GPIO = 1;
static constexpr int BME280_SDA_GPIO      = 2;
static constexpr int BME280_SCL_GPIO      = 3;
static constexpr int BUTTON_GPIO          = 5;
static constexpr int MOTION_GPIO          = 6;

static constexpr unsigned long SECOND_MS          = 1000;
static constexpr unsigned long REBOOT_BLOCKING_MS = 5  * SECOND_MS;
static constexpr unsigned long CONNECT_TIMEOUT_MS = 8  * SECOND_MS; 
static constexpr unsigned long MINUTE_MS          = 60 * SECOND_MS;

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
unsigned long lastConnectEventTime = 0;

int readLux(int pin)
{
  static constexpr float ADC_REF_VOLTAGE    = 3.3f;     // Reference voltage of the ADC (typically 3.3V on ESP32)
  static constexpr int   ADC_MAX_READING    = 4095;     // Maximum value for 12-bit ADC (0–4095)
  static constexpr float VOLT_TO_LUX_FACTOR = 200.0f;   // Approximate conversion: lux = volts * 200

  float volts = (analogRead(pin) * ADC_REF_VOLTAGE) / ADC_MAX_READING;
  return int(volts * VOLT_TO_LUX_FACTOR);
}

void onMqttMessage(String &topic, String &payload)
{
  if (topic == reboot.commandTopic() && payload == "PRESS" && millis() - lastConnectEventTime >= REBOOT_BLOCKING_MS)
    ESP.restart();
}

void sendMotion(bool state = false)
{
  JsonDocument doc;

  if (state)
    doc["motion"] = "ON";
  else
    doc["motion"] = "OFF";

  doc["illuminance_top"] = readLux(TEMT6000_TOP_GPIO);
  doc["illuminance_bottom"] = readLux(TEMT6000_BOTTOM_GPIO);

  char out[128];
  serializeJson(doc, out);
  
  mqtt.publish(motionSensor.stateTopic(), out);
}
  
void sendMeasurements()
{
  JsonDocument doc;

  float temperature, humidity, pressure;
  bme280.read(pressure, temperature, humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);
  
  doc["temperature"] = std::round(temperature * 100) / 100.0f; // shorten to 2 decimals places
  doc["humidity"] = std::round(humidity * 100) / 100.0f;
  doc["pressure"] = std::round(pressure * 100) / 100.0f;
  doc["dewpoint"] = std::round(EnvironmentCalculations::DewPoint(temperature, humidity) * 100) / 100.0f;
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
  Serial.println(F(" This is emergency_exit"));
  Serial.println(F("------------------------"));

  Wire.begin(BME280_SDA_GPIO, BME280_SCL_GPIO);
  bme280.begin();

  pinMode(MOTION_GPIO, INPUT);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);

  analogReadResolution(12); // 12 bits

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
  else if (millis() - processTimer >= MINUTE_MS)
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
