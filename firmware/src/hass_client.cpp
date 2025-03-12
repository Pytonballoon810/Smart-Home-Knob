#include "hass_client.h"
#include "interface_task.h" // Include hier statt im Header
#include <ArduinoJson.h>    // Explicit include for JsonDocument

WiFiClient HassClient::client;

HassClient::HassClient(const uint8_t task_core)
    : Task("HassClient", 4096, 1, task_core) // was 0
{
    knob_state_queue_ = xQueueCreate(1, sizeof(PB_SmartKnobState));
    assert(knob_state_queue_ != NULL);
    client.setTimeout(TIMEOUT);
}

void HassClient::setInterfaceTask(InterfaceTask *interface)
{
    logger_ = interface;
    interface_ = interface;
}

bool HassClient::isWiFiReady()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        log("WiFi not connected");
        return false;
    }
    return true;
}

int HassClient::getStateValue(const char *entityId, int maxValue, bool hue)
{
    if (!isWiFiReady())
    {
        log("Error: WiFi not ready!");
        return 0;
    }

    HTTPClient http;
    String url = String(HASS_URL) + "/api/states/" + entityId;

    http.begin(client, url);
    http.addHeader("Authorization", "Bearer " + String(HASS_TOKEN));
    http.setTimeout(TIMEOUT);

    int value = 0;
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK)
    {
        if (!client.connected())
        {
            log("Client connection lost");
            http.end();
            return 0;
        }

        WiFiClient *stream = http.getStreamPtr();
        if (!stream)
        {
            log("Failed to get stream pointer");
            http.end();
            return 0;
        }

        JsonDocument filter;
        filter["state"] = true;
        filter["attributes"]["brightness"] = true;
        if (hue)
        {
            filter["attributes"]["hs_color"] = true;
        }

        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));

        if (!error)
        {
            const char *state = doc["state"];
            if (state != nullptr)
            {
                if (hue && strcmp(state, "on") == 0)
                {
                    JsonVariant hsColor = doc["attributes"]["hs_color"];
                    if (hsColor.is<JsonArray>())
                    {
                        value = round(hsColor[0].as<float>());
                    }
                }
                else if (maxValue == 1)
                {
                    value = (strcmp(state, "on") == 0) ? 1 : 0;
                }
                else if (strcmp(state, "on") == 0)
                {
                    JsonVariant brightness = doc["attributes"]["brightness"];
                    if (brightness.is<int>())
                    {
                        value = map(brightness.as<int>(), 0, 255, 0, maxValue);
                    }
                    else
                    {
                        value = maxValue;
                    }
                }
                else if (strcmp(state, "off") == 0)
                {
                    value = 0;
                }
            }
            log("Successfully got state from Home Assistant");
        }
        else
        {
            log("Failed to parse Home Assistant response");
        }
    }
    else
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "HTTP request failed with code: %d", httpCode);
        log(buf);
    }

    http.end();
    return value;
}

void HassClient::setStateValue(const char *entityId, int value, int maxValue, bool hue)
{
    if (!isWiFiReady())
    {
        log("Error: WiFi not ready!");
        return;
    }

    HTTPClient http;
    String url = String(HASS_URL) + "/api/services/homeassistant/turn_";

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
    http.setTimeout(TIMEOUT);

    JsonDocument doc;
    doc["entity_id"] = entityId;

    if (hue)
    {
        JsonArray color = doc["hs_color"].to<JsonArray>();
        color.add(value);
        color.add(100);
    }
    else if (value > 0 && maxValue > 1)
    {
        doc["brightness"] = map(value, 0, maxValue, 0, 255);
    }

    String requestBody;
    serializeJson(doc, requestBody);

    int httpCode = http.POST(requestBody);

    if (httpCode == HTTP_CODE_OK)
    {
        log("State update successful");
    }
    else
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "State update failed with code: %d", httpCode);
        log(buf);
    }

    http.end();
}

void HassClient::run()
{
    while (true)
    {
        long now = millis();
        PB_SmartKnobState state;
        if (xQueueReceive(knob_state_queue_, &state, 0) == pdTRUE)
        {

            // Only publish if values changed and 50ms have passed
            if ((now - last_publish_time_ >= HTTP_AND_MQTT_SENDINTERVAL) &&
                (state.current_position != last_published_state_.current_position ||
                 state.config.min_position != last_published_state_.config.min_position ||
                 state.config.max_position != last_published_state_.config.max_position))
            {

                setStateValue(state.config.entity_id, state.current_position, state.config.max_position, state.config.hue);
                // Store the published state and time
                last_published_state_ = state;
                last_publish_time_ = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

QueueHandle_t HassClient::getKnobStateQueue()
{
    return knob_state_queue_;
}

void HassClient::log(const char *msg)
{
    if (logger_ != nullptr)
    {
        logger_->log(msg);
    }
}