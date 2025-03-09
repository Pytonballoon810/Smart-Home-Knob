#if SK_MQTT
#include "mqtt_task.h"

#include "motor_task.h"
#include "secrets.h"
#include "interface_task.h"
#include <ArduinoJson.h>

#define HA_DISCOVERY_PREFIX "homeassistant"
#define DEVICE_NAME "SmartKnob"
#define HA_DISCOVERY_TOPIC HA_DISCOVERY_PREFIX "/number/" HOSTNAME "/config"
#define MQTT_STATE_TOPIC HOSTNAME "/state"
#define MQTT_CONFIG_TOPIC HOSTNAME "/config"

MQTTTask::MQTTTask(const uint8_t task_core, MotorTask &motor_task, Logger &logger) : Task("MQTT", 4096, 1, task_core),
                                                                                     motor_task_(motor_task),
                                                                                     logger_(logger),
                                                                                     wifi_client_(),
                                                                                     mqtt_client_(wifi_client_)
{
    knob_state_queue_ = xQueueCreate(1, sizeof(PB_SmartKnobState));
    assert(knob_state_queue_ != NULL);
    auto callback = [this](char *topic, byte *payload, unsigned int length)
    { mqttCallback(topic, payload, length); };
    mqtt_client_.setCallback(callback);
}

void MQTTTask::connectWifi()
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        logger_.log("Establishing connection to WiFi..");
    }

    char buf[256];
    snprintf(buf, sizeof(buf), "Connected to network %s", WIFI_SSID);
    logger_.log(buf);
}

void MQTTTask::mqttCallback(char *topic, byte *payload, unsigned int length)
{
    char buf[256];
    snprintf(buf, sizeof(buf), "Received mqtt callback for topic %s, length %u", topic, length);
    logger_.log(buf);

    if (strcmp(topic, MQTT_CONFIG_TOPIC) == 0)
    {
        handleConfigMessage(payload, length);
    }
}

void MQTTTask::handleConfigMessage(byte *payload, unsigned int length)
{
    if (!InterfaceTask::configs_loaded_)
    {
        // Use dynamic allocation for large JSON document
        DynamicJsonDocument doc(16384); // Increase buffer size
        DeserializationError error = deserializeJson(doc, payload, length);

        if (error)
        {
            char buf[64];
            snprintf(buf, sizeof(buf), "JSON parse failed: %s", error.c_str());
            logger_.log(buf);
            return;
        }

        if (!doc.is<JsonArray>())
        {
            logger_.log("Config must be an array");
            return;
        }

        JsonArray array = doc.as<JsonArray>();
        size_t i = 0;
        
        // Initialize all configs with safe defaults first
        for (i = 0; i < InterfaceTask::MAX_CONFIGS; i++) {
            PB_SmartKnobConfig &cfg = InterfaceTask::configs_[i];
            memset(&cfg, 0, sizeof(PB_SmartKnobConfig));
            cfg.max_position = -1;
            cfg.position_width_radians = 10 * PI / 180;
            cfg.endstop_strength_unit = 1;
            cfg.snap_point = 1.1;
            strlcpy(cfg.text, "Default", sizeof(cfg.text));
            cfg.led_hue = 200;
        }

        // Now process the JSON configs
        i = 0;
        for (JsonObject config : array) {
            if (i >= InterfaceTask::MAX_CONFIGS) break;

            PB_SmartKnobConfig &cfg = InterfaceTask::configs_[i];
            
            // Safely copy text with length check
            const char* text = config["text"] | "Unnamed";
            strlcpy(cfg.text, text, sizeof(cfg.text)-1);
            cfg.text[sizeof(cfg.text)-1] = '\0';

            // Get numeric values with bounds checking
            cfg.position = constrain(config["position"] | 0, -32768, 32767);
            cfg.min_position = constrain(config["min_position"] | 0, -32768, 32767);
            cfg.max_position = constrain(config["max_position"] | -1, -32768, 32767);
            cfg.position_width_radians = constrain(config["width_radians"] | (10 * PI / 180), 0.0f, PI);
            cfg.detent_strength_unit = constrain(config["detent_strength"] | 0.0f, 0.0f, 10.0f);
            cfg.endstop_strength_unit = constrain(config["endstop_strength"] | 1.0f, 0.0f, 10.0f);
            cfg.snap_point = constrain(config["snap_point"] | 1.1f, 0.0f, 10.0f);
            cfg.led_hue = constrain(config["led_hue"] | 200, 0, 255);

            i++;
        }

        InterfaceTask::num_configs_ = i > 0 ? i : 1; // Ensure at least 1 config
        InterfaceTask::configs_loaded_ = true;

        char buf[64];
        snprintf(buf, sizeof(buf), "Loaded %u configs from MQTT", i);
        logger_.log(buf);
    }
}

void MQTTTask::connectMQTT()
{
    char buf[512];
    mqtt_client_.setServer(MQTT_SERVER, 1883);
    logger_.log("Attempting MQTT connection...");
    if (mqtt_client_.connect(HOSTNAME "-" MQTT_USER, MQTT_USER, MQTT_PASSWORD))
    {
        logger_.log("MQTT connected");

        // Publish Home Assistant discovery configuration
        const char *discovery_json = R"({
"name": "%s",
"unique_id": "%s_position",
"device": {
    "identifiers": ["%s"],
    "name": "%s",
    "model": "SmartKnob",
    "manufacturer": "SmartKnob"
},
"state_topic": "%s",
"value_template": "{{ value_json.position }}",
"command_topic": "%s",
"min": -32768,
"max": 32767})";

        snprintf(buf, sizeof(buf), discovery_json,
                 DEVICE_NAME, HOSTNAME, HOSTNAME, DEVICE_NAME,
                 MQTT_STATE_TOPIC, MQTT_COMMAND_TOPIC);

        printf("HA Discovery: %s\n", buf);
        printf("HA Discovery Topic: %s\n", HA_DISCOVERY_TOPIC);
        if (!mqtt_client_.setBufferSize(512))
        {
            logger_.log("Failed to set buffer size");
        }
        if (!mqtt_client_.publish(HA_DISCOVERY_TOPIC, buf, true))
        {
            logger_.log("Failed to publish HA discovery");
        }
        mqtt_client_.subscribe(MQTT_CONFIG_TOPIC);
        mqtt_client_.subscribe(MQTT_COMMAND_TOPIC);
    }
    else
    {
        snprintf(buf, sizeof(buf), "MQTT failed rc=%d will try again in 5 seconds", mqtt_client_.state());
        logger_.log(buf);
    }
}

void MQTTTask::run()
{
    connectWifi();
    connectMQTT();

    PB_SmartKnobState state;
    while (1)
    {
        long now = millis();
        if (!mqtt_client_.connected() && (now - mqtt_last_connect_time_) > 5000)
        {
            logger_.log("Reconnecting MQTT");
            mqtt_last_connect_time_ = now;
            connectMQTT();
        }
        mqtt_client_.loop();

        if (xQueueReceive(knob_state_queue_, &state, 0) == pdTRUE)
        {
            // Only publish if values changed
            if (state.current_position != last_published_state_.current_position ||
                state.config.min_position != last_published_state_.config.min_position ||
                state.config.max_position != last_published_state_.config.max_position)
            {

                char buf[256];
                snprintf(buf, sizeof(buf),
                         "{\"position\":%d,\"min\":%d,\"max\":%d}",
                         state.current_position,
                         state.config.min_position,
                         state.config.max_position);
                mqtt_client_.publish(MQTT_STATE_TOPIC, buf);

                // Store the published state
                last_published_state_ = state;
            }
        }

        delay(1);
    }
}

QueueHandle_t MQTTTask::getKnobStateQueue()
{
    return knob_state_queue_;
}
#endif
