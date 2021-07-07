#ifndef IOTBOT_SHIELD_H
#define IOTBOT_SHIELD_H

#include <string>
#include <iostream>
#include <fstream>
#include <utility>
#include <iterator>
#include <cstring>
#include <memory>

#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include "mraa/common.hpp"
#include "mraa/uart.hpp"
#include "CRC.h"


namespace iotbot {


#include "iotbot_uart_protocol.h"


/**
 * @class CShield
 * @brief Serial interfacing class (via UART) to motor shield 
 * @author Stefan May, Manuel Kreutz
 * @date 17.05.2021
 */
class CShield
{
public:
    /**
     * constructor
     */
    CShield();

    /**
     * destructor
     */
    ~CShield() = default;

    CShield(const CShield &) = delete;              // no copy constructor
    CShield &operator=(const CShield &) = delete;   // no copy assignment operator

    CShield(CShield &&) = default;              // move constructor -> let the compiler create the constructor by default
    CShield &operator=(CShield &&) = default;   // move via assignment operator

    /**
     * open the internal port using default serial port parameters
     * @param[in] fileName the port file of the actual linux system e.g. /dev/ttyS1
     * @param[in] baudrate the speed of transmittion in bits/s e.g. 115200
     * @return true: if port was opened successfully, false: if an error occured or the port was already opened
     */
    bool openPort(const std::string & fileName, const unsigned int baudrate);

    /**
     * open the internal port using a parameter file 
     * @param[in] paramFilePath the parameter file to read e.g. /path/to/serial_port_params.txt
     * @return true: if port was opened successfully, false: if an error occured or the port was already opened
     */
    bool openPort(const std::string & paramFilePath);

    /**
     * close the internal port
     */
    void closePort();

    /**
     * check if the internal port is opened 
     * @return true: if port was opened successfully, false: if the port is not opened
     */
    bool isPortOpened() const;

    /**
     * send data over specified port -> has to be opened before
     * @param[in] data the structured data to send
     * @return true: successfully written to UART, false: an error occured
     */
    bool send(const SSerialRequestData & data);

    /**
     * receive data over specified port -> has to be opened before
     * @param[out] data the structured receiving data
     * @param[in] waitTimeoutInMilliseconds the max. time in milliseconds to wait for new data on port (0 = no waiting)
     * @return true: successfully read from UART, false: an error occured
     */
    bool receive(SSerialResponseData & data, const unsigned int waitTimeoutInMilliseconds = 0);
    
private:
    /**
     * SSerialParams
     * first:  is value set via parameter file
     * second: value
     */
    typedef struct
    {
        std::pair<bool, std::string> port;
        std::pair<bool, unsigned int> baudrate;
        std::pair<bool, unsigned int> dataBits;
        std::pair<bool, unsigned int> stopBits;
        std::pair<bool, std::string> parity;
        std::pair<bool, std::string> flowControl;
    } SSerialParams;

    SSerialParams params_;              // parameter holder for setup file handling
    std::unique_ptr<mraa::Uart> uart_;  // the underlying UART interface
    bool isPortOpened_;                 // specifies if the UART port is open or not

private:
    /**
     * read UART parameters from a file to the parameter struct
     * @param[in] filePath path to parameter file e.g. /path/to/serial_port_params.txt
     * @return true: read was successful, false: error occured
     */
    bool readParamFile(const std::string & filePath);
    
    /**
     * initialize the parameter struct value availability with false
     */
    void initParamStruct();

    /**
     * set the baudrate of the internal UART port
     * @param[in] baudrate a valid baudrate e.g. 115200
     * @return true: set up was successful, false: error occured
     */
    bool setBaudrate(const unsigned int baudrate);

    /**
     * set the mode of the internal UART port
     * @param[in] dataBits a valid amount of data bits e.g. 8
     * @param[in] parity the parity mode
     * @param[in] stopBits a valid amount of stop bits e.g. 1
     * @return true: set up was successful, false: error occured
     */
    bool setMode(const unsigned int dataBits, const mraa::UartParity parity, const unsigned int stopBits);

    /**
     * set the flow control of the internal UART port
     * @param[in] xOnOff use of XON / XOFF as flow control
     * @param[in] rtsCts use of RTS / CTS as flow control
     * @return true: set up was successful, false: error occured
     */
    bool setFlowControl(const bool xOnOff, const bool rtsCts);

    /**
     * fill the tx buffer with the given data in order of the specified protocol
     * !!! changes in the UART request protocol have to be corrected in this function !!!
     * @param[in] data the data struct storing all values to send
     * @return true: buffer filling was successful, false: error occured
     */
    bool fillTxBuffer(const SSerialRequestData & data);

    /**
     * fill the data struct with the data of the received rx buffer in order of the specified protocol
     * !!! changes in the UART response protocol have to be corrected in this function !!!
     * @param[out] data the structured receiving data
     * @return true: data receive from buffer was successful, false: error occured
     */
    bool fillDataFromRxBuffer(SSerialResponseData & data);

    /**
     * copy a value to the tx buffer at a specified buffer offset
     * @param[in] value the value to copy to the tx buffer
     * @param[in] offset specifies the place in the tx buffer to be filled
     */
    template <typename T>
    void setTxBufferWith(const T & value, const unsigned int offset);

    /**
     * copy a via offset specified value from the rx buffer to the given value
     * @param[out] value the value that will be filled
     * @param[in] offset specifies the place in the rx buffer to get the data from
     */
    template <typename T>
    void getValueFromRxBuffer(T & value, const unsigned int offset);
};


} // namespace iotbot


#endif // IOTBOT_SHIELD_H
