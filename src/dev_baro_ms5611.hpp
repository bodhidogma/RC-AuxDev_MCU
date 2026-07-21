/**
 * 
 */

#ifndef _MS5611_HPP
#define _MS5611_HPP

#include "mymain.h"
#include "stm_console.hpp"

class DevMS5611 {
public:
    // OSR (Over Sampling Ratio) constants
    enum class OSR : uint8_t {
        OSR_256  = 0x00,
        OSR_512  = 0x02,
        OSR_1024 = 0x04,
        OSR_2048 = 0x06,
        OSR_4096 = 0x08
    };

    DevMS5611(I2C_HandleTypeDef* hi2c, uint8_t address = 0x77 << 1);
    
    bool begin();
    bool read();

    float getPressure() const { return pressure; }       // In mbar / hPa
    float getTemperature() const { return temperature; } // In Celsius

private:
    I2C_HandleTypeDef* _hi2c;
    uint8_t _address;
    
    uint16_t fc[6]; // Calibration coefficients
    float pressure;
    float temperature;

    bool reset();
    bool readProm();
    uint32_t readRawADC(uint8_t cmd);
};

#endif // _MS5611_HPP
