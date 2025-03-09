#pragma once

#if SK_MQTT

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "logger.h"
#include "motor_task.h"
#include "task.h"


class MQTTTask : public Task<MQTTTask> {
    friend class Task<MQTTTask>; // Allow base Task to invoke protected run()

    public:
        MQTTTask(const uint8_t task_core, MotorTask& motor_task, Logger& logger);
        QueueHandle_t getKnobStateQueue();

    protected:
        void run();

    private:
        QueueHandle_t knob_state_queue_;
        MotorTask& motor_task_;
        Logger& logger_;
        WiFiClient wifi_client_;
        PubSubClient mqtt_client_;
        int mqtt_last_connect_time_ = 0;
        PB_SmartKnobState last_published_state_;

        void connectWifi();
        void connectMQTT();
        void mqttCallback(char *topic, byte *payload, unsigned int length);
};

#endif
