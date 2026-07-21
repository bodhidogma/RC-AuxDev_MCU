/**
 * 
 */


#ifndef MSP_HANDLER_HPP
#define MSP_HANDLER_HPP

#include "mymain.h"
#include "stm_console.hpp"
 
class MspHandler {
public:
    // MSP Command IDs
    static constexpr uint8_t MSP_STATUS  = 101;
    static constexpr uint8_t MSP_RAW_IMU = 102;

    MspHandler();

    // Call this inside your USB CDC Receive Callback whenever new bytes arrive
    void processByte(uint8_t byte);

    // Call this in your main loop at a stable rate (e.g., 50Hz) to stream data
    void sendImuData(float ax, float ay, float az, float gx, float gy, float gz);

private:
    // Parser State Machine
    enum class State { WAIT_START, WAIT_M, WAIT_ARROW, READ_SIZE, READ_CMD, READ_PAYLOAD, READ_CRC };
    State _state;

    uint8_t _rxBuffer[64];
    uint8_t _payloadSize;
    uint8_t _cmd;
    uint8_t _payloadIndex;
    uint8_t _checksum;

    void handleCommand(uint8_t cmd);
    void sendPacket(uint8_t cmd, const uint8_t* payload, uint8_t size);
};

#endif // MSP_HANDLER_HPP
