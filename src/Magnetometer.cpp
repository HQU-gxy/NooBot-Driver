#include <EEPROM.h>
#include <STM32FreeRTOS.h>
#include <ulog.h>

#include "CMSIS_DSP.h"
#include "Magnetometer.h"
#include "config.h"
#include "SensorCollector.h"

namespace Magneto
{
    static TwoWire i2c1(I2C1_SDA_PIN, I2C1_SCL_PIN);
    static Calibration calibration;

    constexpr uint8_t QMC5883_ADDR = 0x0D;
    constexpr uint8_t QMC5883_REG_X_LSB = 0x00;
    constexpr uint8_t QMC5883_REG_STATUS = 0x06;
    constexpr uint8_t QMC5883_REG_CONFIG1 = 0x09;
    constexpr uint8_t QMC5883_REG_CONFIG2 = 0x0A;
    constexpr uint8_t QMC5883_REG_SET_RESET_PERIOD = 0x0B;
    constexpr uint8_t QMC5883_REG_CHIP_ID = 0x0D;

    constexpr uint8_t QMC5883_INIT_TIMEOUT = 100;
    constexpr uint8_t QMC5883_READ_TIMEOUT = 100;

    constexpr uint8_t QMC5883_CAL_ADDR = 0x69;

    constexpr uint8_t AVR_SAMPLES_COUNT = 8;
    static MagData collectedData[AVR_SAMPLES_COUNT]{MagData{0}}; // Raw data buffer
    uint8_t readIndex = 0;                                       // Read index for the buffer

    void writeRegister(uint8_t reg, uint8_t *val, uint8_t len = 1)
    {
        i2c1.beginTransmission(QMC5883_ADDR);
        i2c1.write(reg);
        i2c1.write(val, len);
        i2c1.endTransmission();
    }

    bool readRegister(uint8_t reg, uint8_t *data, uint8_t len = 1)
    {
        i2c1.flush();
        i2c1.beginTransmission(QMC5883_ADDR);
        i2c1.write(reg);
        i2c1.endTransmission();
        i2c1.requestFrom(QMC5883_ADDR, len);

        auto startTime = millis();
        while (millis() - startTime < QMC5883_READ_TIMEOUT)
        {
            if (i2c1.available() < len) // Wait for required bytes
                continue;

            for (uint8_t i = 0; i < len; i++) // Read data
            {
                data[i] = i2c1.read();
            }
            return true;
        }
        ULOG_ERROR("QMC5883 read timeout");
        return false;
    }

    inline bool dataReady()
    {
        uint8_t status;
        readRegister(QMC5883_REG_STATUS, &status);
        return status & 0x01;
    }

    void readSensor()
    {
        if (!dataReady())
        {
            ULOG_WARNING("QMC5883 data not ready");
            return;
        }

        uint8_t data[6];
        if (!readRegister(QMC5883_REG_X_LSB, data, 6))
        {
            ULOG_ERROR("QMC5883 read failed");
            return;
        }

        // Combine the bytes into 16-bit signed integers
        collectedData[readIndex].x = (data[1] << 8) | data[0];
        collectedData[readIndex].y = (data[3] << 8) | data[2];
        collectedData[readIndex].z = (data[5] << 8) | data[4];
        if (++readIndex >= AVR_SAMPLES_COUNT)
            readIndex = 0;
    }

    void getAvrRawData(MagData *data)
    {
        float32_t temp[3]{0};
        for (auto d : collectedData)
        {
            temp[0] += d.x;
            temp[1] += d.y;
            temp[2] += d.z;
        }

        arm_scale_f32(temp, 1.0f / AVR_SAMPLES_COUNT, temp, 3);
        data->x = temp[0];
        data->y = temp[1];
        data->z = temp[2];
    }

    bool getAvrRawDataWithCal(MagData *xyz)
    {
        int16_t data[3];
        getAvrRawData(reinterpret_cast<MagData *>(data));

        arm_sub_q15(data, calibration.offset, data, 3);
        float32_t temp[3]{
            data[0],
            data[1],
            data[2],
        };
        arm_mult_f32(temp, calibration.gain, temp, 3);
        xyz->x = temp[0];
        xyz->y = temp[1];
        xyz->z = temp[2];

        return true;
    }

    void setCalibration(Calibration *cal)
    {
        eeprom_buffer_fill();
        for (uint8_t i = 0; i < sizeof(Calibration); i++)
        {
            eeprom_buffered_write_byte(QMC5883_CAL_ADDR + i, reinterpret_cast<uint8_t *>(cal)[i]);
        }

        eeprom_buffer_flush();                  // Write the data to EEPROM
        memcpy(&calibration, cal, sizeof(cal)); // Apply the calibration data
    }

    void getCalibration(Calibration *cal)
    {
        eeprom_buffer_fill();
        // cal->offsetX = eeprom_buffered_read_byte(QMC5883_CAL_ADDR) | (eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 1) << 8);
        // cal->gainX = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 2);

        // cal->offsetY = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 3) | (eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 4) << 8);
        // cal->gainY = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 5);

        // cal->offsetZ = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 6) | (eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 7) << 8);
        // cal->gainZ = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + 8);

        for (uint8_t i = 0; i < sizeof(Calibration); i++)
        {
            reinterpret_cast<uint8_t *>(cal)[i] = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + i);
        }
    }

    void runCalibration(void *params)
    {
        MagData temp;
        MagData minData, maxData;

        ULOG_INFO("Compas calibration started");
        auto tft = static_cast<TFT_eSPI *>(params);

        while (1)
        {
            getAvrRawData(&temp);
            if (temp.x > maxData.x)
                maxData.x = temp.x;
            if (temp.y > maxData.y)
                maxData.y = temp.y;
            if (temp.z > maxData.z)
                maxData.z = temp.z;

            if (temp.x < minData.x)
                minData.x = temp.x;
            if (temp.y < minData.y)
                minData.y = temp.y;
            if (temp.z < minData.z)
                minData.z = temp.z;

            if (tft)
            {
                tft->fillRect(0, 40, 160, 40, TFT_BLACK);
                tft->drawString("Max X: " + String(maxData.x) + " Min X: " + String(minData.x), 8, 40);
                tft->drawString("Max Y: " + String(maxData.y) + " Min Y: " + String(minData.y), 8, 50);
                tft->drawString("Max Z: " + String(maxData.z) + " Min Z: " + String(minData.z), 8, 60);
            }
            if (Serial2.available() && Serial2.read() == 'e')
                break;
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        Calibration cal;
        cal.offset[0] = (maxData.x + minData.x) / 2;
        cal.offset[1] = (maxData.y + minData.y) / 2;
        cal.offset[2] = (maxData.z + minData.z) / 2;

        cal.gain[0] = 1.0f;
        cal.gain[1] = static_cast<float>(maxData.y - minData.y) / static_cast<float>(maxData.x - minData.x);
        cal.gain[2] = static_cast<float>(maxData.z - minData.z) / static_cast<float>(maxData.x - minData.x);
        setCalibration(&cal);
        ULOG_INFO("Compas calibration done");
    }

    bool getGauss(float *xyz)
    {
        MagData data;
        getAvrRawData(&data);

        constexpr float QMC5883_16BIT_SENSITIVITY = 12000.0f;

        xyz[0] = data.x;
        xyz[1] = data.y;
        xyz[2] = data.z;

        arm_scale_f32(xyz, 1.0f / QMC5883_16BIT_SENSITIVITY, xyz, 3);

        return true;
    }

    float getHeading()
    {
        MagData data;
        getAvrRawDataWithCal(&data);

        float32_t heading;
        heading = atan2(data.y, data.x); // This shitty version of CMSIS-DSP doesn't have atan2f
        if (heading < 0)
            heading += 2 * PI;

        arm_scale_f32(&heading, RAD_TO_DEG, &heading, 1);
        return heading;
    }

    bool begin(Mode m, DataRate odr, Range rng, OverSample osr)
    {
        i2c1.begin();
        uint8_t chipID;
        readRegister(QMC5883_REG_CHIP_ID, &chipID);
        if (chipID != 0xff)
        { // It's always 0xff by normal
            ULOG_ERROR("QMC5883 not found");
            return false;
        }

        uint8_t cfgData = (osr << 6) | (rng << 4) | (odr << 2) | m;
        writeRegister(QMC5883_REG_CONFIG1, &cfgData);

        getCalibration(&calibration); // Load calibration data

        auto startTime = millis();
        uint8_t cfgRead;
        while (millis() - startTime < QMC5883_INIT_TIMEOUT)
        {
            // Check if the configuration is written
            readRegister(QMC5883_REG_CONFIG1, &cfgRead);
            if (cfgRead == cfgData)
            {
                ULOG_INFO("QMC5883 initialized");
                SensorCollector::registerSensorCb("Magneto", readSensor);
                return true;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        ULOG_ERROR("QMC5883 configure timeout");
        return false;
    }

} // namespace Magneto