/**
 * 
 */

 #include "dev_baro_ms5611.hpp"

 #include <stdio.h>
 #include <string.h>

 #include "stm_hal_shims.hpp"

// Commands
#define CMD_RESET       0x1E
#define CMD_ADC_READ    0x00
#define CMD_ADC_CONV    0x40
#define CMD_PROM_READ   0xA0

DevMS5611::DevMS5611(I2C_HandleTypeDef* hi2c, uint8_t address) 
    : _hi2c(hi2c), _address(address), pressure(0.0f), temperature(0.0f) {
    for(int i = 0; i < 6; i++) fc[i] = 0;
}

bool DevMS5611::begin() {
    if (!reset()) return false;
    HAL_Delay(10);
    return readProm();
}

bool DevMS5611::reset() {
    uint8_t cmd = CMD_RESET;
    return HAL_I2C_Master_Transmit(_hi2c, _address, &cmd, 1, 100) == HAL_OK;
}

bool DevMS5611::readProm() {
    uint8_t buffer[2];
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t cmd = CMD_PROM_READ + ((i + 1) * 2);
        if (HAL_I2C_Master_Transmit(_hi2c, _address, &cmd, 1, 100) != HAL_OK) return false;
        if (HAL_I2C_Master_Receive(_hi2c, _address, buffer, 2, 100) != HAL_OK) return false;
        fc[i] = (buffer[0] << 8) | buffer[1];
    }
    return true;
}

uint32_t DevMS5611::readRawADC(uint8_t cmd) {
    uint8_t buffer[3];
    if (HAL_I2C_Master_Transmit(_hi2c, _address, &cmd, 1, 100) != HAL_OK) return 0;
    
    // Max delay required for OSR 4096 is 9.04ms
    HAL_Delay(10); 
    
    uint8_t reg = CMD_ADC_READ;
    if (HAL_I2C_Master_Transmit(_hi2c, _address, &reg, 1, 100) != HAL_OK) return 0;
    if (HAL_I2C_Master_Receive(_hi2c, _address, buffer, 3, 100) != HAL_OK) return 0;
    
    return ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
}

bool DevMS5611::read() {
    // Read raw pressure (D1) and raw temperature (D2) at max precision (OSR 4096)
    uint32_t D1 = readRawADC(CMD_ADC_CONV | (uint8_t)OSR::OSR_4096);
    uint32_t D2 = readRawADC(CMD_ADC_CONV | (uint8_t)OSR::OSR_4096 | 0x10);

    if (D1 == 0 || D2 == 0) return false;

    // Formulas adapted from MS5611 datasheet
    int64_t dT = D2 - ((int64_t)fc[4] << 8);
    int32_t TEMP = 2000 + ((dT * fc[5]) >> 23);

    int64_t off = ((int64_t)fc[1] << 16) + ((fc[3] * dT) >> 7);
    int64_t sens = ((int64_t)fc[0] << 15) + ((fc[2] * dT) >> 8);

    // Second order temperature compensation
    if (TEMP < 2000) {
        int64_t T2 = (dT * dT) >> 31;
        int64_t OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
        int64_t SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 2;
        
        if (TEMP < -1500) {
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) >> 1;
        }
        TEMP -= T2;
        off -= OFF2;
        sens -= SENS2;
    }

    int32_t P = (((D1 * sens) >> 21) - off) >> 15;

    temperature = (float)TEMP / 100.0f;
    pressure = (float)P / 100.0f;

    return true;
}
