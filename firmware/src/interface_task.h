#pragma once

#include <AceButton.h>
#include <Arduino.h>

#include "configuration.h"
#include "display_task.h"
#include "logger.h"
#include "motor_task.h"
#include "serial/serial_protocol_plaintext.h"
#include "serial/serial_protocol_protobuf.h"
#include "serial/uart_stream.h"
#include "task.h"

#ifndef SK_FORCE_UART_STREAM
#define SK_FORCE_UART_STREAM 0
#endif

// Forward declaration
class HassClient;

class InterfaceTask : public Task<InterfaceTask>, public Logger
{
    friend class Task<InterfaceTask>; // Allow base Task to invoke protected run()

public:
    InterfaceTask(const uint8_t task_core, MotorTask &motor_task, DisplayTask *display_task, HassClient &hass_client);
    virtual ~InterfaceTask();

    static constexpr size_t MAX_CONFIGS = 20;
    static PB_SmartKnobConfig configs_[MAX_CONFIGS];
    static size_t num_configs_;
    static bool configs_loaded_;

    void log(const char *msg) override;
    void setConfiguration(Configuration *configuration);

protected:
    void run();

private:
#if defined(CONFIG_IDF_TARGET_ESP32S3) && !SK_FORCE_UART_STREAM
    HWCDC stream_;
#else
    UartStream stream_;
#endif
    MotorTask &motor_task_;
    DisplayTask *display_task_;
    HassClient &hass_client_;
    char buf_[128];

    SemaphoreHandle_t mutex_;
    Configuration *configuration_ = nullptr; // protected by mutex_

    PB_PersistentConfiguration configuration_value_;
    bool configuration_loaded_ = false;

    uint8_t strain_calibration_step_ = 0;
    int32_t strain_reading_ = 0;

    SerialProtocol *current_protocol_ = nullptr;
    bool remote_controlled_ = false;
    int current_config_ = 0;
    uint8_t press_count_ = 1;

    PB_SmartKnobState latest_state_ = {};
    PB_SmartKnobConfig latest_config_ = {};

    QueueHandle_t log_queue_;
    QueueHandle_t knob_state_queue_;
    SerialProtocolPlaintext plaintext_protocol_;
    SerialProtocolProtobuf proto_protocol_;

#if SK_ALS
    bool als_initialized_ = false;
    uint32_t als_retry_time_ = 0;
    static constexpr uint32_t ALS_RETRY_INTERVAL_MS = 5000;
#endif

    void changeConfig(bool next);
    void updateHardware();
    void publishState();
    void applyConfig(PB_SmartKnobConfig &config, bool from_remote);
};
