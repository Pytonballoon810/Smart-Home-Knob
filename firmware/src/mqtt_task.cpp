#if SK_MQTT
#include "mqtt_task.h"

#include "motor_task.h"
#include "secrets.h"

#define HA_DISCOVERY_PREFIX "homeassistant"
#define DEVICE_NAME "SmartKnob"
#define HA_DISCOVERY_TOPIC HA_DISCOVERY_PREFIX "/number/" HOSTNAME "/config"
#define MQTT_STATE_TOPIC HOSTNAME "/state"

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
        if (!mqtt_client_.setBufferSize(128))
        {
            logger_.log("Failed to set buffer size Back");
        }
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
