#pragma once

#include <Arduino.h>

namespace IMU
{
    struct __attribute__((packed)) IMUData
    {
        float accelX;
        float accelY;
        float accelZ;
        float gyroX;
        float gyroY;
        float gyroZ;
    };

    bool begin();
    void getData(IMUData *data);

}