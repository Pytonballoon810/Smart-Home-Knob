#pragma once

#include <FFat.h>
#include <PacketSerial.h>

#include "proto_gen/smartknob.pb.h"

#include "logger.h"

const uint32_t PERSISTENT_CONFIGURATION_VERSION = 1;

class Configuration
{
public:
    Configuration();
    ~Configuration();

    void setLogger(Logger *logger);
    bool loadFromDisk();
    bool saveToDisk();
    PB_PersistentConfiguration get();
    bool setMotorCalibrationAndSave(PB_MotorCalibration &motor_calibration);
    bool setStrainCalibrationAndSave(PB_StrainCalibration &strain_calibration);

private:
    SemaphoreHandle_t mutex_;

    Logger *logger_ = nullptr;
    bool loaded_ = false;
    PB_PersistentConfiguration pb_buffer_ = {};

    uint8_t buffer_[PB_PersistentConfiguration_size];

    void log(const char *msg);
};
class FatGuard
{
public:
    FatGuard(Logger *logger) : logger_(logger)
    {
        const int max_retries = 3;
        for (int i = 0; i < max_retries; i++)
        {
            if (tryMountFFat())
            {
                return;
            }
            if (i == 0)
            {
                // Try formatting on first failure
                if (formatFFat())
                {
                    if (tryMountFFat())
                    {
                        return;
                    }
                }
            }
            if (logger_ != nullptr)
            {
                char buf[64];
                snprintf(buf, sizeof(buf), "Failed to mount FFat, retry %d/%d", i + 1, max_retries);
                logger_->log(buf);
            }
            delay(500); // Increased delay between retries
        }
        if (logger_ != nullptr)
        {
            logger_->log("Failed all attempts to mount FFat");
        }
    }
    ~FatGuard()
    {
        if (mounted_)
        {
            FFat.end();
            if (logger_ != nullptr)
            {
                logger_->log("Unmounted FFat");
            }
        }
    }
    FatGuard(FatGuard const &) = delete;
    FatGuard &operator=(FatGuard const &) = delete;

    bool mounted_ = false;

private:
    Logger *logger_;

    bool tryMountFFat()
    {
        if (FFat.begin(false))
        {
            if (logger_ != nullptr)
            {
                logger_->log("Mounted FFat");
            }
            mounted_ = true;
            return true;
        }
        return false;
    }

    bool formatFFat()
    {
        if (logger_ != nullptr)
        {
            logger_->log("Formatting FFat partition...");
        }
        if (!FFat.format())
        {
            if (logger_ != nullptr)
            {
                logger_->log("Format failed!");
            }
            return false;
        }
        if (logger_ != nullptr)
        {
            logger_->log("Format successful");
        }
        return true;
    }
};
