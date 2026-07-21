/**
 * 
 */

#ifndef _DEV_IMU_MPU6050_HPP
#define _DEV_IMU_MPU6050_HPP

#include "mymain.h"
#include "stm_console.hpp"

class DevMPU6050 {
public:
    // Configuration Enums
    enum class GyroScale : uint8_t { FS_256 = 0x00, FS_512 = 0x08, FS_1024 = 0x10, FS_2048 = 0x18 };
    enum class AccelScale : uint8_t { FS_2G = 0x00, FS_4G = 0x08, FS_8G = 0x10, FS_16G = 0x18 };

    DevMPU6050(I2C_HandleTypeDef* hi2c, uint8_t address = 0x68 << 1);

    bool begin(GyroScale gyroConfig = GyroScale::FS_512, AccelScale accelConfig = AccelScale::FS_4G);
    bool read();

    // Getters for processed data
    float getAccX() const { return ax; }
    float getAccY() const { return ay; }
    float getAccZ() const { return az; }
    float getGyroX() const { return gx; }
    float getGyroY() const { return gy; }
    float getGyroZ() const { return gz; }
    float getTemperature() const { return temperature; }

private:
    I2C_HandleTypeDef* _hi2c;
    uint8_t _address;
    float accScaleModifier;
    float gyroScaleModifier;

    // Converted readings
    float ax, ay, az;
    float gx, gy, gz;
    float temperature;

    bool writeRegister(uint8_t reg, uint8_t data);
};

#endif // _DEV_IMU_MPU6050_HPP
