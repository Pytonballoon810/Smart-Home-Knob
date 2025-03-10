#if SK_MQTT
#include "mqtt_task.h"

#include "motor_task.h"
#include "secrets.h"
#include "interface_task.h"
#include <ArduinoJson.h>

#define HA_DISCOVERY_PREFIX "homeassistant"
#define DEVICE_NAME "SmartKnob"
#define HA_DISCOVERY HA_DISCOVERY_PREFIX "/sensor/" HOSTNAME
#define HA_DISCOVERY_TOPIC_VALUE HA_DISCOVERY "/" HOSTNAME "-value/config"
#define HA_DISCOVERY_TOPIC_ENTITY_ID HA_DISCOVERY "/" HOSTNAME "-entity_id/config"
#define MQTT_VALUE_STATE_TOPIC HOSTNAME "/value/state"
#define MQTT_ENTITY_ID_STATE_TOPIC HOSTNAME "/entity_id/state"
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
        // First try with heap allocation
        DynamicJsonDocument *doc = nullptr;
        try
        {
            doc = new DynamicJsonDocument(16384);
            if (!doc)
            {
                logger_.log("Failed to allocate JSON document");
                return;
            }

            DeserializationError error = deserializeJson(*doc, payload, length);
            if (error)
            {
                logger_.log("Failed to parse config JSON");
                delete doc;
                return;
            }

            if (!doc->is<JsonArray>())
            {
                logger_.log("Config must be an array");
                delete doc;
                return;
            }

            JsonArray array = doc->as<JsonArray>();
            size_t i = 0;

            // First validate all entries before modifying configs
            bool valid = true;
            for (JsonObject config : array)
            {
                if (!config.containsKey("position") ||
                    !config.containsKey("min_position") ||
                    !config.containsKey("max_position"))
                {
                    valid = false;
                    break;
                }
            }

            if (!valid)
            {
                logger_.log("Invalid config format");
                delete doc;
                return;
            }

            // Now safe to modify configs
            for (JsonObject config : array)
            {
                if (i >= InterfaceTask::MAX_CONFIGS)
                    break;

                PB_SmartKnobConfig &cfg = InterfaceTask::configs_[i];
                memset(&cfg, 0, sizeof(PB_SmartKnobConfig)); // Clear config first

                cfg.position = config["position"] | 0;
                cfg.min_position = config["min_position"] | 0;
                cfg.max_position = config["max_position"] | -1;
                cfg.position_width_radians = config["width_radians"] | (10 * PI / 180);
                cfg.detent_strength_unit = config["detent_strength"] | 0;
                cfg.endstop_strength_unit = config["endstop_strength"] | 1;
                cfg.snap_point = config["snap_point"] | 1.1;

                const char *text = config["text"] | "Unnamed";
                strncpy(cfg.text, text, sizeof(cfg.text) - 1);
                cfg.text[sizeof(cfg.text) - 1] = '\0';

                cfg.led_hue = config["led_hue"] | 200;

                const char *entity_id = config["entity_id"] | "";
                strncpy(cfg.entity_id, entity_id, sizeof(cfg.entity_id) - 1);
                cfg.entity_id[sizeof(cfg.entity_id) - 1] = '\0';

                i++;
            }

            InterfaceTask::num_configs_ = i > 0 ? i : 1;
            InterfaceTask::configs_loaded_ = true;

            char buf[64];
            snprintf(buf, sizeof(buf), "Loaded %u configs from MQTT", i);
            logger_.log(buf);

            delete doc;
        }
        catch (...)
        {
            logger_.log("Exception during config parsing");
            if (doc)
                delete doc;
            return;
        }
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

        // Request configs immediately after connection
        mqtt_client_.publish(MQTT_CONFIG_TOPIC "/get", "1", false);

        // Publish Home Assistant discovery configuration
        const char *discovery_json = R"({
"name": "position",
"unique_id": "position",
"device": {
    "identifiers": ["%s"],
    "name": "%s",
    "model": "SmartKnob",
    "manufacturer": "SmartKnob"
},
"state_topic": "%s",
"value_template": "{{ value_json.position | string }}"
})";

        snprintf(buf, sizeof(buf), discovery_json,
                DEVICE_NAME, DEVICE_NAME,
                MQTT_VALUE_STATE_TOPIC);

        printf("HA Discovery: %s\n", buf);
        printf("HA Discovery Topic: %s\n", HA_DISCOVERY_TOPIC_VALUE);
        if (!mqtt_client_.setBufferSize(4096))
        {
            logger_.log("Failed to set buffer size");
        }
        if (!mqtt_client_.publish(HA_DISCOVERY_TOPIC_VALUE, buf, true))
        {
            logger_.log("Failed to publish HA discovery");
        }
        discovery_json = R"({
"name": "entity_id",
"unique_id": "entity_id",
"device": {
    "identifiers": ["%s"]
},
"state_topic": "%s",
"value_template": "{{ value_json.entity_id | string }}"
})";

        snprintf(buf, sizeof(buf), discovery_json,
                DEVICE_NAME,
                MQTT_ENTITY_ID_STATE_TOPIC);

        printf("HA Discovery: %s\n", buf);
        printf("HA Discovery Topic: %s\n", HA_DISCOVERY_TOPIC_ENTITY_ID);
        if (!mqtt_client_.setBufferSize(4096))
        {
            logger_.log("Failed to set buffer size");
        }
        if (!mqtt_client_.publish(HA_DISCOVERY_TOPIC_ENTITY_ID, buf, true))
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
            // Only publish if values changed and 50ms have passed
            if ((now - mqtt_last_publish_time_ >= 100) &&
                (state.current_position != last_published_state_.current_position ||
                 state.config.min_position != last_published_state_.config.min_position ||
                 state.config.max_position != last_published_state_.config.max_position))
            {
                char buf[256];
                snprintf(buf, sizeof(buf),
                         "{\"position\":\"%d\"}",
                         state.current_position);
                mqtt_client_.publish(MQTT_VALUE_STATE_TOPIC, buf);

                snprintf(buf, sizeof(buf),
                         "{\"entity_id\":\"%s\"}",
                         state.config.entity_id);
                mqtt_client_.publish(MQTT_ENTITY_ID_STATE_TOPIC, buf);

                // Store the published state and time
                last_published_state_ = state;
                mqtt_last_publish_time_ = now;
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
