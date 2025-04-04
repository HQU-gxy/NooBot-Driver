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

    /**
     * @brief Write data to the sensor register
     *
     * @param reg Register address
     * @param data Pointer to the data
     * @param len Length of the data to write, default is 1 byte
     */
    void writeRegister(uint8_t reg, uint8_t *data, uint8_t len = 1)
    {
        i2c1.beginTransmission(QMC5883_ADDR);
        i2c1.write(reg);
        i2c1.write(data, len);
        i2c1.endTransmission();
    }

    /**
     * @brief Read data from the sensor register
     *
     * @param reg Register address
     * @param data Pointer to the data buffer
     * @param len Length of the data to read, default is 1 byte
     *
     * @return true if the read is successful
     */
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

    /**
     * @brief Check if the data is ready to be read
     *
     * @return true if the data is ready
     */
    inline bool dataReady()
    {
        uint8_t status;
        readRegister(QMC5883_REG_STATUS, &status);
        return status & 0x01;
    }

    /**
     * @brief Read the sensor data and store it in the buffer
     *
     * @note This function is called in the sensor collector timer
     */
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

    /**
     * @brief Get the average raw data from the buffer
     *
     * @param data Pointer to the MagData structure to store the average data
     */
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

    /**
     * @brief Get the average raw data with calibration applied
     *
     * @param data Pointer to the MagData structure to store the average data
     * @return true
     * @return false
     */
    void getAvrRawDataWithCal(MagData *data)
    {
        int16_t rawData[3];
        getAvrRawData(reinterpret_cast<MagData *>(rawData));

        arm_sub_q15(rawData, calibration.offset, rawData, 3);
        float32_t temp[3]{
            rawData[0],
            rawData[1],
            rawData[2],
        };
        arm_mult_f32(temp, calibration.gain, temp, 3);
        data->x = temp[0];
        data->y = temp[1];
        data->z = temp[2];
    }

    /**
     * @brief Write the calibration data to the EEPROM and apply it
     *
     * @param cal The calibration data to write
     */
    void setCalibration(const Calibration &cal)
    {
        eeprom_buffer_fill();
        for (uint8_t i = 0; i < sizeof(Calibration); i++)
        {
            eeprom_buffered_write_byte(QMC5883_CAL_ADDR + i, reinterpret_cast<const uint8_t *>(&cal)[i]);
        }

        eeprom_buffer_flush();                   // Write the data to EEPROM
        memcpy(&calibration, &cal, sizeof(cal)); // Apply the calibration data
    }

    /**
     * @brief Get the calibration data from the EEPROM
     *
     * @param cal Pointer to the Calibration structure to store the data
     */
    void getCalibration(Calibration *cal)
    {
        eeprom_buffer_fill();

        for (uint8_t i = 0; i < sizeof(Calibration); i++)
        {
            reinterpret_cast<uint8_t *>(cal)[i] = eeprom_buffered_read_byte(QMC5883_CAL_ADDR + i);
        }
    }

    /**
     * @brief A freaking naive calibration function
     *
     * @param params Pointer to the TFT_eSPI object to display the calibration data
     * @note This function is called as a separate task, it's a very shitty way to do it, could be modified later
     */
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
        setCalibration(cal);
        ULOG_INFO("Compas calibration done");
    }

    /**
     * @brief Get the heading angle in degrees
     *
     * @return float The heading in degrees
     */
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

    /**
     * @brief Initialize the QMC5883 sensor
     *
     * @param m The mode of the sensor, default is continuous
     * @param odr The output data rate, default is 200Hz
     * @param rng The range of the sensor, default is 2GA
     * @param osr The oversampling rate, default is 512
     * @return true if the initialization is successful
     */
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