#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "logger.h"
#include "task.h"
#include "pb_decode.h"
#include "proto_gen/smartknob.pb.h"

// Forward declaration
class InterfaceTask;

class HassClient : public Task<HassClient>
{
    friend class Task<HassClient>; // Allow base Task to invoke protected run()

public:
    HassClient(const uint8_t task_core);             // Konstruktor ohne InterfaceTask
    void setInterfaceTask(InterfaceTask *interface); // Neue Methode
    int getStateValue(const char *entityId, int maxValue);
    void setStateValue(const char *entityId, int value, int maxValue);
    QueueHandle_t getKnobStateQueue();

protected:
    void run();

private:
    Logger *logger_;
    static WiFiClient client;
    InterfaceTask *interface_ = nullptr; // Pointer statt Referenz
    QueueHandle_t knob_state_queue_;

    long last_publish_time_ = 0;
    PB_SmartKnobState last_published_state_;

    void log(const char *msg); // Private log Methode
    bool isWiFiReady();
};
