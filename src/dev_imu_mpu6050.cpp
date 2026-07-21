/**
 * 
 */

#include "dev_imu_mpu6050.hpp"

 #include <stdio.h>
 #include <string.h>

 #include "stm_hal_shims.hpp"

// MPU6050 Register Map
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_ENABLE   0x38
#define REG_ACCEL_XOUT_H 0x3B
#define REG_TEMP_OUT_H   0x41
#define REG_GYRO_XOUT_H  0x43
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

DevMPU6050::DevMPU6050(I2C_HandleTypeDef* hi2c, uint8_t address)
    : _hi2c(hi2c), _address(address), 
      accScaleModifier(16384.0f), gyroScaleModifier(131.0f),
      ax(0), ay(0), az(0), gx(0), gy(0), gz(0), temperature(0) {}

bool DevMPU6050::writeRegister(uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(_hi2c, _address, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK;
}

bool DevMPU6050::begin(GyroScale gyroConfig, AccelScale accelConfig) {
    uint8_t whoAmI = 0;
    
    // Check device ID (Should return 0x68)
    HAL_I2C_Mem_Read(_hi2c, _address, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &whoAmI, 1, 100);
    if (whoAmI != 0x68) return false;

    // Wake up MPU6050 (bring out of sleep mode, use internal 8MHz oscillator)
    if (!writeRegister(REG_PWR_MGMT_1, 0x00)) return false;

    // Set Sample Rate Divider to 0 (1kHz sample rate)
    if (!writeRegister(REG_SMPLRT_DIV, 0x07)) return false;

    // Configure Gyroscope Full Scale Range
    if (!writeRegister(REG_GYRO_CONFIG, static_cast<uint8_t>(gyroConfig))) return false;
    switch(gyroConfig) {
        case GyroScale::FS_256:  gyroScaleModifier = 131.0f;  break;
        case GyroScale::FS_512:  gyroScaleModifier = 65.5f;   break;
        case GyroScale::FS_1024: gyroScaleModifier = 32.8f;   break;
        case GyroScale::FS_2048: gyroScaleModifier = 16.4f;   break;
    }

    // Configure Accelerometer Full Scale Range
    if (!writeRegister(REG_ACCEL_CONFIG, static_cast<uint8_t>(accelConfig))) return false;
    switch(accelConfig) {
        case AccelScale::FS_2G:  accScaleModifier = 16384.0f; break;
        case AccelScale::FS_4G:  accScaleModifier = 8192.0f;  break;
        case AccelScale::FS_8G:  accScaleModifier = 4096.0f;  break;
        case AccelScale::FS_16G: accScaleModifier = 2048.0f;  break;
    }

    return true;
}

bool DevMPU6050::read() {
    uint8_t dataBuffer[14];
    
    // Read 14 consecutive bytes starting from ACCEL_XOUT_H
    if (HAL_I2C_Mem_Read(_hi2c, _address, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, dataBuffer, 14, 100) != HAL_OK) {
        return false;
    }

    // Parse Accelerometer Raw Data
    int16_t rawAccX = (dataBuffer[0] << 8) | dataBuffer[1];
    int16_t rawAccY = (dataBuffer[2] << 8) | dataBuffer[3];
    int16_t rawAccZ = (dataBuffer[4] << 8) | dataBuffer[5];

    // Parse Temperature Raw Data
    int16_t rawTemp = (dataBuffer[6] << 8) | dataBuffer[7];

    // Parse Gyroscope Raw Data
    int16_t rawGyroX = (dataBuffer[8] << 8) | dataBuffer[9];
    int16_t rawGyroY = (dataBuffer[10] << 8) | dataBuffer[11];
    int16_t rawGyroZ = (dataBuffer[12] << 8) | dataBuffer[13];

    // Convert raw data to engineering units (G's and degrees/sec)
    ax = static_cast<float>(rawAccX) / accScaleModifier;
    ay = static_cast<float>(rawAccY) / accScaleModifier;
    az = static_cast<float>(rawAccZ) / accScaleModifier;

    // Formula from datasheet: Temp in °C = (RawValue / 340) + 36.53
    temperature = (static_cast<float>(rawTemp) / 340.0f) + 36.53f;

    gx = static_cast<float>(rawGyroX) / gyroScaleModifier;
    gy = static_cast<float>(rawGyroY) / gyroScaleModifier;
    gz = static_cast<float>(rawGyroZ) / gyroScaleModifier;

    return true;
}
