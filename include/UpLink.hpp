#pragma once

#include <utility>
#include <functional>
#include <STM32FreeRTOS.h>
#include <timers.h>
#include <ulog.h>
#include <Arduino.h>

#include "IMU.h"

class UpLink
{
public:
    using onUpLinkCommandCB = std::function<void(const float targetLinear, const float targetAngular)>;
    // Current linear speed, current angular speed, IMU data
    using getStatusFunc = std::function<std::tuple<float, float, IMU::IMUData>()>;

private:
    HardwareSerial *linkSerial;

    onUpLinkCommandCB onCmdCB;
    getStatusFunc getStatus;
    callback_function_t onLinkLostCallback;
    callback_function_t onConnectCallback;

    uint8_t retryCount = 0;
    bool linkLost = false;

    static constexpr uint8_t MAX_RETRY = 10;

    static constexpr uint8_t SEND_STATUS_PERIOD = 50; // ms
    static constexpr uint8_t CHECK_CMD_PERIOD = 20;   // ms

    struct __attribute__((packed)) UpLinkCommand
    {
        uint8_t header;      // Should be 0x7b
        float targetLinear;  // Linear speed in m/s
        float targetAngular; // Angular speed in rad/s
        uint8_t checksum;
    };

    static void sendStat(TimerHandle_t handle)
    {
        auto uplink = reinterpret_cast<UpLink *>(pvTimerGetTimerID(handle));

        if (!uplink->getStatus)
        {
            ULOG_WARNING("Get status function not set, will not send status");
            return;
        }

        auto [currLinear, currAngular, imuData] = uplink->getStatus();

        struct __attribute__((packed))
        {
            uint8_t header = 0x69;
            float currentLinear;     // Linear speed in m/s
            float currentAngular;    // Angular speed in rad/s
            uint8_t powerPercentage; // Battery percentage
            // Accel in m/s^2
            IMU::IMUData imu;
            uint8_t checksum;
        } status{
            .currentLinear = currLinear,
            .currentAngular = currAngular,
            .imu = imuData,
        };
        auto bytes = reinterpret_cast<uint8_t *>(&status);
        status.checksum = calcSum(bytes, sizeof(status) - 1);

        uplink->linkSerial->write(bytes, sizeof(status));
    }

    static uint8_t calcSum(const uint8_t *buf, size_t len)
    {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < len; i++)
        {
            sum ^= buf[i];
        }
        return sum;
    }

    static void checkCmd(TimerHandle_t handle)
    {
        auto uplink = reinterpret_cast<UpLink *>(pvTimerGetTimerID(handle));

        if ((!uplink->linkLost) && (uplink->retryCount++ == uplink->MAX_RETRY))
        {
            if (uplink->onLinkLostCallback)
                uplink->onLinkLostCallback();
            else
                ULOG_WARNING("LoraLink: No on-link-lost callback set");

            uplink->linkLost = true;
        }

        while (uplink->linkSerial->available() && (uplink->linkSerial->peek() != 0x7b)) // The header byte
        {
            uplink->linkSerial->read();
        }

        if (!uplink->linkSerial->available())
            return;

        char buf[16];
        uplink->linkSerial->readBytes(buf, sizeof(UpLinkCommand));
        auto parsed = reinterpret_cast<const UpLinkCommand *>(buf);
        auto sum = calcSum(reinterpret_cast<const uint8_t *>(buf), sizeof(UpLinkCommand) - 1);

        if (sum != parsed->checksum)
        {
            ULOG_ERROR("UpLink command checksum error: %x", sum);
            return;
        }

        if (uplink->onCmdCB)
            uplink->onCmdCB(parsed->targetLinear, parsed->targetAngular);
        else
            ULOG_WARNING("No callback set for UpLink command");

        // Clear the buffer
        while (uplink->linkSerial->available())
            uplink->linkSerial->read();

        if (uplink->linkLost)
        {
            if (uplink->onConnectCallback)
                uplink->onConnectCallback();
            else
                ULOG_WARNING("LoraLink: No on-connect callback set");
        }
        uplink->retryCount = 0;
        uplink->linkLost = false;
    }

public:
    UpLink(HardwareSerial &ser, uint32_t tx_pin, uint32_t rx_pin, uint32_t baudrate) : linkSerial(&ser)
    {
        linkSerial->setTx(tx_pin);
        linkSerial->setRx(rx_pin);
        linkSerial->setTimeout(20);
        linkSerial->begin(baudrate);
        auto sendStatTimer = xTimerCreate("Send status", pdMS_TO_TICKS(SEND_STATUS_PERIOD),
                                          true, reinterpret_cast<void *>(this), sendStat);
        auto checkCmdTimer = xTimerCreate("Read command", pdMS_TO_TICKS(CHECK_CMD_PERIOD),
                                          true, reinterpret_cast<void *>(this), checkCmd);

        xTimerStart(sendStatTimer, 0);
        xTimerStart(checkCmdTimer, 0);
    }

    inline void setOnCmdCallback(onUpLinkCommandCB cb) { onCmdCB = cb; }

    inline void setGetStatusFunc(getStatusFunc cb) { getStatus = cb; }

    inline void setOnLinkLostCallback(callback_function_t cb) { onLinkLostCallback = cb; }

    inline void setOnConnectCallback(callback_function_t cb) { onConnectCallback = cb; }
};