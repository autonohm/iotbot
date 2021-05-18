#ifndef _UART_PROTOCOL_H_
#define _UART_PROTOCOL_H_


#include <stdint.h>


// ************************************* Commands *************************************

// Standard responses
#define SERIAL_RESPONSE_RPM         0xA0
#define SERIAL_RESPONSE_POS         0xA1

// Error responses
#define SERIAL_ERR_ENCA_NOSIGNAL    0xE0
#define SERIAL_ERR_ENCB_NOSIGNAL    0xE1


enum ESerialCommand
{
	CMD_ENABLE          = 0x01,
	CMD_DISABLE			= 0x02,
	CMD_SETTIMEOUT		= 0x03,
	CMD_SETPWMMAX		= 0x04,
	CMD_SENDRPM			= 0x05,
	CMD_SENDPOS			= 0x06,
	CMD_INVERTENC		= 0x07,

	// Operating commands
	CMD_SETPWM			= 0x10,
	CMD_SETRPM			= 0x11,
	CMD_FREQ_SCALE		= 0x12,
	CMD_SYNC			= 0x13,

	// Closed/Open loop controller parameters
	CMD_CTL_KP			= 0x20,
	CMD_CTL_KI			= 0x21,
	CMD_CTL_KD			= 0x22,
	CMD_CTL_ANTIWINDUP	= 0x23,
	CMD_CTL_INPUTFILTER	= 0x24,

	// Platform parameters
	CMD_GEARRATIO		= 0x30,
	CMD_TICKSPERREV		= 0x31,

	CMD_AUX1			= 0x40,
	CMD_AUX2			= 0x41,
	CMD_LIGHTS_OFF		= 0x42,
	CMD_DIM_LIGHT		= 0x43, // Dimmed headlight
	CMD_HIGH_BEAM		= 0x44, // High beam headlight
	CMD_FLASH_ALL		= 0x45, // Flash lights
	CMD_FLASH_LEFT		= 0x46, // Flash lights to the left
	CMD_FLASH_RIGHT		= 0x47, // Flash lights to the right
	CMD_PULSATION		= 0x48  // Pulsation
};


// ************************************* Request (IOT to Shield) *************************************
/*
 *    1 Byte    |  1 Byte   |     8 Bytes     |  1 Byte
 *  Start Byte  |  Command  |     Payload     |   CRC
 *                               e.g. RPM
 */

#define SERIAL_REQUEST_BUFFER_LEN 11


enum ESerialRequestOffset
{
	REQ_START   = 0,    // 1 byte
	REQ_COMMAND = 1,    // 1 byte
	REQ_PAYLOAD = 2,    // 8 bytes  ???
	REQ_CRC     = 10    // 1 byte
};


typedef struct
{
	uint8_t start;
    uint8_t command;
	uint16_t payload[4];
    uint8_t crc;

} SSerialRequestData;


static uint8_t g_serialRequestBuffer[SERIAL_REQUEST_BUFFER_LEN] = { 0 };


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
	RES_COMMAND     = 0,	// 1 byte
	RES_RPM         = 1,    // 8 bytes
    RES_IMU         = 9,    // 12 bytes
    RES_TOF         = 21,   // 8 bytes
    RES_VOLTAGE     = 29,   // 2 bytes
    RES_TEMPERATURE = 31,   // 1 byte
	RES_CRC         = 32    // 1 byte
};


typedef struct
{
    uint8_t command;
	uint16_t rpm[4];
    uint16_t imu[6];
    uint16_t tof[4];
    uint16_t voltage;
    uint8_t temperature;
    uint8_t crc;

} SSerialResponseData;


static uint8_t g_serialResponseBuffer[SERIAL_RESPONSE_BUFFER_LEN] = { 0 };


#endif // _UART_PROTOCOL_H_
