#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>

namespace Magneto
{
    enum Mode
    {
        MODE_STANDBY = 0,
        MODE_CONTINUOUS
    };

    enum DataRate
    {
        DATARATE_10HZ = 0,
        DATARATE_50HZ,
        DATARATE_100HZ,
        DATARATE_200HZ
    };

    enum Range
    {
        RANGE_2GA = 0,
        RANGE_8GA
    };
    enum OverSample
    {
        OVERSAMPLE_512 = 0,
        OVERSAMPLE_256,
        OVERSAMPLE_128,
        OVERSAMPLE_64,
    };

    struct MagData
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    struct Calibration
    {
        int16_t offset[3]; // Offset for each axis
        float gain[3];     // Gain for each axis
    };

    bool begin(const Mode m = MODE_CONTINUOUS, const DataRate odr = DATARATE_200HZ, const Range rng = RANGE_2GA, const OverSample osr = OVERSAMPLE_512);
    void setCalibration(const Calibration &calibration);
    void getCalibration(Calibration *calibration);
    // void runCalibration(Calibration *calibration);
    void runCalibration(void *params); // Show calibration data on TFT
    inline String getCalibrationDataString(const Calibration &calibration)
    {
        return "OffsetX, Y, Z: " + String(calibration.offset[0]) + ", " + String(calibration.offset[1]) + ", " + String(calibration.offset[2]) + "\n" +
               "GainX, Y, Z: " + String(calibration.gain[0]) + ", " + String(calibration.gain[1]) + ", " + String(calibration.gain[2]);
    }

    float getHeading();

} // namespace Magneto