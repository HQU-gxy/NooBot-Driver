#pragma once

#include <Arduino.h>

/**
 * @brief SensorCollector
 *
 * This namespace provides a way to collect data from multiple sensors
 *
 */

namespace SensorCollector
{
    void begin();

    void registerSensorCb(String name, callback_function_t readFunc);
} // namespace SensorCollector
