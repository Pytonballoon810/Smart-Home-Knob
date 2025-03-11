#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

class HassClient
{
public:
    static int getStateValue(const char *entityId, int maxValue);

private:
    static WiFiClient client;
};
