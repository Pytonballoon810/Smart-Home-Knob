#include "hass_client.h"

WiFiClient HassClient::client;

HassClient::HassClient(const uint8_t task_core, InterfaceTask &interface, Logger &logger)
    : Task("HassClient", 4096, 1, task_core),
        logger_(logger),
        interface_(interface)
{
    knob_state_queue_ = xQueueCreate(1, sizeof(PB_SmartKnobState));
    assert(knob_state_queue_ != NULL);
    client.setTimeout(5000);
}

int HassClient::getStateValue(const char *entityId, int maxValue)
{
    HTTPClient http;
    String url = String(HASS_URL) + "/api/states/" + entityId;

    http.begin(client, url);
    http.addHeader("Authorization", "Bearer " + String(HASS_TOKEN));

    int value = 0;
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK)
    {
        // Use streaming parser to reduce memory usage
        StaticJsonDocument<128> filter;
        filter["state"] = true;
        filter["attributes"]["brightness"] = true;

        StaticJsonDocument<128> doc;
        DeserializationError error = deserializeJson(doc, http.getString(), DeserializationOption::Filter(filter));

        if (!error)
        {
            if (maxValue == 1)
            {
                value = (doc["state"] == "on") ? 1 : 0;
            }
            else if (doc["state"] == "on")
            {
                if (doc["attributes"]["brightness"].is<int>())
                {
                    value = map(doc["attributes"]["brightness"].as<int>(), 0, 255, 0, maxValue);
                }
                else
                {
                    value = maxValue;
                }
            }
            interface_.log("Successfully got state from Home Assistant");
        }
        else
        {
            interface_.log("Failed to parse Home Assistant response");
        }
    }
    else
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "HTTP request failed with code: %d", httpCode);
        interface_.log(buf);
    }

    http.end();
    return value;
}

void HassClient::setStateValue(const char *entityId, int value, int maxValue)
{
    HTTPClient http;
    String url = String(HASS_URL) + "/api/services/light/turn_";

    if (value == 0)
    {
        url += "off";
    }
    else
    {
        url += "on";
    }

    http.begin(client, url);
    http.addHeader("Authorization", "Bearer " + String(HASS_TOKEN));
    http.addHeader("Content-Type", "application/json");

    StaticJsonDocument<200> doc;
    doc["entity_id"] = entityId;

    if (value > 0 && maxValue > 1)
    {
        doc["brightness"] = map(value, 0, maxValue, 0, 255);
    }

    String requestBody;
    serializeJson(doc, requestBody);

    int httpCode = http.POST(requestBody);
    if (httpCode != HTTP_CODE_OK)
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "State update failed with code: %d", httpCode);
        interface_.log(buf);
    }

    http.end();
}

void HassClient::run()
{
    while (true)
    {
        long now = millis();
        PB_SmartKnobState state;
        if (xQueueReceive(knob_state_queue_, &state, pdMS_TO_TICKS(100)) == pdTRUE)
        {

            // Only publish if values changed and 50ms have passed
            if ((now - last_publish_time_ >= 100) &&
                (state.current_position != last_published_state_.current_position ||
                 state.config.min_position != last_published_state_.config.min_position ||
                 state.config.max_position != last_published_state_.config.max_position))
            {
                
                setStateValue(state.config.entity_id, state.current_position, state.config.max_position);
                // Store the published state and time
                last_published_state_ = state;
                last_publish_time_ = now;
            }


            
                
            
        }
        delay(1);
    }
}

QueueHandle_t HassClient::getKnobStateQueue()
{
    return knob_state_queue_;
}