#include <BMI088.h>
#include <vector>
#include <CMSIS_DSP.h>
#include <STM32FreeRTOS.h>
#include <timers.h>
#include <ulog.h>

#include "IMU.h"
#include "config.h"
#include "SensorCollector.h"

namespace IMU
{
    static Bmi088Accel accel(SPI, ACCEL_CS_PIN);
    static Bmi088Gyro gyro(SPI, GYRO_CS_PIN);

    constexpr uint8_t AVR_SAMPLES_COUNT = 8;
    IMUData collectedData[AVR_SAMPLES_COUNT]{IMUData{0}};
    uint8_t readIndex = 0;

    void readSensors()
    {
        if (!accel.getDrdyStatus())
            return;

        accel.readSensor();
        gyro.readSensor();

        collectedData[readIndex++] = IMUData{accel.getAccelX_mss(),
                                             accel.getAccelY_mss(),
                                             accel.getAccelZ_mss(),
                                             gyro.getGyroX_rads(),
                                             gyro.getGyroY_rads(),
                                             gyro.getGyroZ_rads()};
        if (readIndex >= AVR_SAMPLES_COUNT)
            readIndex = 0;
    }

    bool begin()
    {
        if (accel.begin() < 0)
        {
            ULOG_ERROR("Accel initialization failed");
            return false;
        }
        accel.setRange(Bmi088Accel::RANGE_6G);
        accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ);
        ULOG_INFO("Accel initialized");

        if (gyro.begin() < 0)
        {
            ULOG_ERROR("Gyro initialization failed");
            return false;
        }
        gyro.setRange(Bmi088Gyro::RANGE_500DPS);
        gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ);
        ULOG_INFO("Gyro initialized");

        SensorCollector::registerSensorCb("IMU", readSensors);
        return true;
    }

    void getData(IMUData *data)
    {
        float32_t temp[6]{0};
        for (auto d : collectedData)
        {
            temp[0] += d.accelX;
            temp[1] += d.accelY;
            temp[2] += d.accelZ;
            temp[3] += d.gyroX;
            temp[4] += d.gyroY;
            temp[5] += d.gyroZ;
        }

        arm_scale_f32(temp, 1.0f / AVR_SAMPLES_COUNT, temp, 6);

        data->accelX = temp[0];
        data->accelY = temp[1];
        data->accelZ = temp[2];
        data->gyroX = temp[3];
        data->gyroY = temp[4];
        data->gyroZ = temp[5];
    }
} // namespace IMU