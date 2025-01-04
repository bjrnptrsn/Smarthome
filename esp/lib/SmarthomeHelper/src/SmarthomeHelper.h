#ifndef SmarthomeHelper_h
#define SmarthomeHelper_h

#include <MQTT.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// class LightEngine
#define MIREDS (1000000 / _kelvin)

class HAEntity
{

public:
    static MQTTClient *_mqtt;

protected:
    String _availability_topic;
    String _state_topic;
    String _command_topic;
    String _discovery_topic;

private:
    JsonDocument _doc;

public:
    HAEntity() {}

    ~HAEntity() {}

    bool readConfig(const char *filename, const char *componentName)
    {
        // DEBUG
        // const char *file = filename;
        // Serial.println(file);

        File file = LittleFS.open(filename, "r");
        DeserializationError error = deserializeJson(_doc, file);
        file.close();

        if (error)
        {
            Serial.println("deserializeJson() failed: ");
            Serial.println(error.f_str());
            return false;
        }

        _availability_topic = _doc["availability_topic"].as<String>();

        String component = componentName;
        // relay has no 'state_topic'
        // 'command_topic' only needed for light and button
        if (!component.equals("event") or !component.equals("button"))
            _state_topic = _doc["state_topic"].as<String>();

        if (component.equals("light") || component.equals("button"))
            _command_topic = _doc["command_topic"].as<String>();

        // create homeassistant discovery topic
        _discovery_topic = _doc["unique_id"].as<String>();
        _discovery_topic.setCharAt(_discovery_topic.lastIndexOf("_"), char('/'));
        _discovery_topic = String("homeassistant/") + component + "/" + _discovery_topic + "/config";

        return true;
    }

    bool publishConfig()
    {
        char out[1024];
        serializeJson(_doc, out);
        // Serial.println(out);
        // Serial.println(_discovery_topic);

        // publish homeassistant config for device discovery
        return _mqtt->publish(_discovery_topic, out, true, 0);
    }

    String stateTopic()
    {
        return _state_topic;
    }

    String commandTopic()
    {
        return _command_topic;
    }

    String availabilityTopic()
    {
        return _availability_topic;
    }

private:
    // class HAEntity
};

class LightEngine : public HAEntity
{

    // variables
public:
protected:
    int _brightness = 0;
    int _brightnessScale;
    int _lastBrightness = BRIGHTNESS_SCALE;

    bool _transitionState = false;
    int _set_transition_ms;
    int _last_transition_ms;
    int _transitionBegin;
    int _transitionEnd;
    double _transitionSlope_ms;
    unsigned long _transitionStart_ms;

    // CCT
    bool _cctLight;
    int _kelvin = 3450;
    int _minKelvin;
    int _maxKelvin;
    u_int8_t _byteWW;
    u_int8_t _byteCW;

    // methods
public:
    enum LIGHTENGINE
    {
        BRIGHTNESS_SCALE = 100,
        TRANSITION_MIN = 100,
        TRANSITION_MAX = 10000,
        TRANSITION_DEFAULT = 700,
        MIN_KELVIN = 2700,
        MAX_KELVIN = 6500,
        MIN_MIREDS = 153,
        MAX_MIREDS = 350
    };

    LightEngine()
    {
    }

    bool setBrightness(int setBrightness, int setTransition_ms = -1)
    {
        if (setTransition_ms == -1) setTransition_ms = _set_transition_ms;
        else if (setTransition_ms < TRANSITION_MIN) setTransition_ms = TRANSITION_MIN;
        else if (setTransition_ms > TRANSITION_MAX) setTransition_ms = TRANSITION_MAX;
        _last_transition_ms = setTransition_ms;

        if ((0 <= setBrightness and setBrightness <= BRIGHTNESS_SCALE) and (setBrightness != _brightness))
        {
            _transitionBegin = _brightness;
            _transitionEnd = setBrightness;

            _transitionSlope_ms = abs(_transitionEnd - _transitionBegin) / double(setTransition_ms);
            // _transitionSlope_ms = BRIGHTNESS_SCALE / double(setTransition_ms);
            if (setBrightness - _transitionBegin < 0)
                _transitionSlope_ms = -(_transitionSlope_ms);

            _transitionState = true;
            // if transition starts at 0%, subtract 1% of the transition duration to be 1% ahead
            _transitionStart_ms = millis() - (_transitionBegin == 0 ? (setTransition_ms / 10) : 0);

            return true;
        }
        else
            return false;
    }

    bool setColorTemp(int value)
    {
        // value means kelvin
        if (_minKelvin <= value and value <= _maxKelvin and value != _kelvin)
        {
            _kelvin = value;
            cct();
            return true;
        }

        // value means mireds  ( DEPRECATED: latest Home Assistant finally moves to Kelvin )
        else if (MIN_MIREDS <= value and value <= MAX_MIREDS and value != MIREDS)
        {
            // Temperature in K = 10^6 / mireds
            _kelvin = 1000000 / value;
            cct();
            return true;
        }
        else
            return false;
    }

    void setTransition_ms(int transition_ms)
    {
        if (transition_ms < 100 or transition_ms > 10000)
            _set_transition_ms = TRANSITION_DEFAULT;
        else
            _set_transition_ms = transition_ms;

        sendState();
    }

    u_int8_t brightness()
    {
        return _brightness;
    }

    u_int8_t byteWW()
    {
        return _byteWW;
    }

    u_int8_t byteCW()
    {
        return _byteCW;
    }

    bool parseJson(String &payload)
    {
        JsonDocument doc;
        deserializeJson(doc, payload);

        int docBrightness = -1;

        if (doc["state"].is<String>())
        {
            String docState = doc["state"];

            if ((docState.equals("ON") or docState.equals("on")) and _brightness == 0)
                docBrightness = _lastBrightness;
            else if ((docState.equals("OFF") or docState.equals("off")) and _brightness > 0)
                docBrightness = 0;
        }

        if (doc["set_transition"].is<int>() or doc["set_transition"].is<float>())
        {
            if (doc["set_transition"].is<int>())
                setTransition_ms(doc["set_transition"]);
            else if (doc["set_transition"].is<float>())
                setTransition_ms(int(float(doc["set_transition"]) * 1000));
        }

        int docTransition_ms = _set_transition_ms;

        if (doc["transition"].is<int>() or doc["transition"].is<float>())
        {
            if (doc["transition"].is<int>())
                docTransition_ms = doc["transition"];
            else if (doc["transition"].is<float>())
                docTransition_ms = int(float(doc["transition"]) * 1000);

        }

        if (docBrightness != 0)
        {
            if (doc["brightness"].is<int>())
            {
                docBrightness = doc["brightness"];
                if (docBrightness == _brightness)
                    docBrightness = -1;
                else if (docBrightness > BRIGHTNESS_SCALE)
                    docBrightness = BRIGHTNESS_SCALE;
            }
        }

        if (_cctLight and doc["color_temp"].is<int>())
        {
            int docColorTemp = doc["color_temp"];
            if (docColorTemp != _kelvin)
                if (setColorTemp(docColorTemp))
                    return true;
        }

        if (docBrightness >= 0)
        {
            // on lamp setting to OFF, save last brightness
            if (docBrightness > 0)
                _lastBrightness = docBrightness;

            if (setBrightness(docBrightness, docTransition_ms))
                return true;
        }

        return false;
    }

    String composeJson()
    {

        JsonDocument doc;

        if (_transitionEnd > 0)
            doc["state"] = "ON";
        else
            doc["state"] = "OFF";

        doc["brightness"] = _transitionEnd;

        if (_cctLight)
        {
            doc["color_mode"] = "color_temp";
            doc["color_temp"] = MIREDS;
        }
        doc["transition"] = std::ceil(_last_transition_ms/100) / 10.0;
        doc["set_transition"] = std::ceil(_set_transition_ms/100) / 10.0;

        String out;
        serializeJson(doc, out);

        return out;
    }

    void sendState()
    {
        _mqtt->publish(_state_topic, composeJson());
    }

    bool run()
    {
        return transitionEngine();
    }

private:
    bool transitionEngine()
    {
        if (_transitionState)
        {

            _brightness = _transitionBegin + ((millis() - _transitionStart_ms) * _transitionSlope_ms);
            cct();

            if (abs(_brightness - _transitionBegin) > abs(_transitionEnd - _transitionBegin))
            {
                _brightness = _transitionEnd;
                cct();
                _transitionState = false;
            }
        }

        return _transitionState;
    }

    void cct()
    {
        u_int8_t pctWW = float((MAX_KELVIN - _kelvin) / 38);
        u_int8_t pctCW = 100 - pctWW;

        _byteWW = ((float(_brightness) / 100) * (float(pctWW) / 100) * 255);
        _byteCW = ((float(_brightness) / 100) * (float(pctCW) / 100) * 255);
    }

    // class LightEngine
};

class LightEntity : public LightEngine
{
public:
    LightEntity(bool cctLight = false,
                int brightnessScale = LightEngine::BRIGHTNESS_SCALE,
                int transition_ms = LightEngine::TRANSITION_DEFAULT,
                int min_kelvin = LightEngine::MIN_KELVIN,
                int max_kelvin = LightEngine::MAX_KELVIN)
    {
        _cctLight = cctLight;
        _brightnessScale = brightnessScale;
        _set_transition_ms = transition_ms;
        _minKelvin = min_kelvin;
        _maxKelvin = max_kelvin;
    }

    ~LightEntity() {}

    bool parse(String &payload)
    {
        if (parseJson(payload))
            sendState();
        else
            return false;

        return true;
    }

    bool run()
    {
        return LightEngine::run();
    }
};

class ButtonEntity : public HAEntity
{

public:
    ButtonEntity() {}

    ~ButtonEntity() {}

    enum BUTTON
    {
        BUTTON_IDLE,
        BUTTON_SINGLE,
        BUTTON_DOUBLE,
        BUTTON_MULTI,
        BUTTON_LONG_START,
        BUTTON_LONG_DURING,
        BUTTON_LONG_STOP
    };

private:
    String _previous_state = "idle";

public:
    void sendButtonState(int action, int clicks = 0)
    {
        String state;
        switch (action)
        {
        case BUTTON_IDLE:
            state = "idle";
            break;
        case BUTTON_SINGLE:
            state = "singleclick";
            break;
        case BUTTON_DOUBLE:
            state = "doubleclick";
            break;
        case BUTTON_MULTI:
            state = String(clicks) + " clicks";
            break;
        case BUTTON_LONG_START:
            state = "longpress";
            break;
        case BUTTON_LONG_DURING:
            state = "longpress_during";
            break;
        case BUTTON_LONG_STOP:
            state = "idle";
            break;
        default:
            state = "idle";
            break;
        }

        JsonDocument doc;
        doc["state"] = state;
        doc["previous_state"] = _previous_state;

        if (!state.equals("idle"))
            _previous_state = state;

        char out[64];
        serializeJson(doc, out);
        _mqtt->publish(_state_topic, out);
    }
};

#endif