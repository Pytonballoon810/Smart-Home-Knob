#include "hass_client.h"

WiFiClient HassClient::client;

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
        }
    }

    http.end();
    return value;
}
