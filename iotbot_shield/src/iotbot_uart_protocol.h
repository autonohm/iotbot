#ifndef IOTBOT_UART_PROTOCOL_H
#define IOTBOT_UART_PROTOCOL_H

#include <stdint.h>


// ************************************* SERIAL CONSTANTS *************************************

#define FLOAT_TO_UINT16_MULTIPLIER  100     // e.g. 12.34 (float) -> 1234 (uint16_t) for serial send
#define UINT16_TO_FLOAT_DIVISOR     100.00f // e.g. 1234 (uint16_t) -> 12.34 (float) for serial receive

#define FLOAT_TO_UINT8_MULTIPLIER   10      // e.g. 12.34 (float) -> 123 (uint8_t) for serial send
#define UINT8_TO_FLOAT_DIVISOR      10.0f   // e.g. 123 (uint8_t) -> 12.3 (float) for serial receive


// ************************************* Commands *************************************

enum ESerialRequest
{
    // Operating commands
    CMD_ENABLE              = 0x01,
    CMD_DISABLE             = 0x02,
    CMD_SET_TIMEOUT         = 0x03,
    CMD_SET_PWM_MAX         = 0x04,
    CMD_SET_POS             = 0x06,
    CMD_INVERT_ENC          = 0x07,

    CMD_SET_PWM             = 0x10,
    CMD_SET_RPM             = 0x11,
    CMD_FREQ_SCALE          = 0x12,
    CMD_SYNC                = 0x13,

    // Closed / Open loop controller parameters
    CMD_CTL_KP              = 0x20,
    CMD_CTL_KI              = 0x21,
    CMD_CTL_KD              = 0x22,
    CMD_CTL_ANTIWINDUP      = 0x23,
    CMD_CTL_INPUTFILTER     = 0x24,

    // Platform parameters
    CMD_GEAR_RATIO          = 0x30,
    CMD_TICKS_PER_REV       = 0x31,

    CMD_AUX1                = 0x40,
    CMD_AUX2                = 0x41,

    CMD_LIGHTS_OFF          = 0x42,
    CMD_LIGHTS_DIM_LIGHT    = 0x43, // Dimmed headlight
    CMD_LIGHTS_HIGH_BEAM    = 0x44, // High beam headlight
    CMD_LIGHTS_FLASH_ALL    = 0x45, // Flash lights
    CMD_LIGHTS_FLASH_LEFT   = 0x46, // Flash lights to the left
    CMD_LIGHTS_FLASH_RIGHT  = 0x47, // Flash lights to the right
    CMD_LIGHTS_PULSATION    = 0x48, // Pulsation
    CMD_LIGHTS_ROTATION     = 0x49, // Rotation light, e.g. police light in blue
    CMD_LIGHTS_RUNNING      = 0x4A  // Running light
};


enum ESerialResponse
{
    // Standard response
    CMD_SUCCESS             = 0xA0,

    // Error response
    CMD_ERR_ENCA_NOSIGNAL   = 0xE0,
    CMD_ERR_ENCB_NOSIGNAL   = 0xE1
};


// ************************************* Request (IOT to Shield) *************************************
/*
 *    1 Byte    |  1 Byte   |     8 Bytes     |  1 Byte
 *  Start Byte  |  Command  |     Payload     |   CRC
 *     0xFF                       e.g. RPM
 */

#define SERIAL_REQUEST_BUFFER_LEN 11

// Start byte for request
#define SERIAL_REQUEST_START_VALUE 0xFF

enum ESerialRequestOffset
{
    REQ_START   = 0,    // 1 byte
    REQ_COMMAND = 1,    // 1 byte
    REQ_PAYLOAD = 2,    // 8 bytes  TODO: passt die Länge ???
    REQ_CRC     = 10    // 1 byte
};

typedef struct
{
    uint8_t  start;
    uint8_t  command;
    uint16_t payload[4];
    //uint8_t  crc;

} SSerialRequestData;

extern uint8_t g_serialRequestBuffer[SERIAL_REQUEST_BUFFER_LEN];


// ************************************* Response (Shield to IOT) *************************************
/*
 *  1 Byte   |       8 Bytes       |      12 Bytes      |   8 Bytes  |  2 Bytes  |    1 Byte     |  1 Byte
 *  Command  |         RPM         |    IMU raw data    |  ToF data  |  Voltage  |  Temperature  |   CRC
 *           |      2 / wheel      |  2 / acc x, y, z   |  2 / scan  |
 *           |    fl, fr, rl, rr   |  2 / gyr x, y, z   |
 */

#define SERIAL_RESPONSE_BUFFER_LEN 33

enum ESerialResponseOffset
{
    RES_COMMAND     = 0,    // 1 byte
    RES_RPM         = 1,    // 8 bytes
    RES_IMU         = 9,    // 12 bytes
    RES_TOF         = 21,   // 8 bytes
    RES_VOLTAGE     = 29,   // 2 bytes
    RES_TEMPERATURE = 31,   // 1 byte
    RES_CRC         = 32    // 1 byte
};

typedef struct
{
    uint8_t  command;
    uint16_t rpm[4];
    uint16_t imu[6];
    uint16_t tof[4];
    uint16_t voltage;
    int8_t   temperature;
    //uint8_t  crc;

} SSerialResponseData;

extern uint8_t g_serialResponseBuffer[SERIAL_RESPONSE_BUFFER_LEN];


#endif // IOTBOT_UART_PROTOCOL_H
