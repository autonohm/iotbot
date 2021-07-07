#include "iotbot_shield.hpp"


namespace iotbot {


// definition of the request (TX) and response (RX) buffers which are declared in the uart_protocol header
uint8_t g_serialRequestBuffer[SERIAL_REQUEST_BUFFER_LEN]   = { 0 };
uint8_t g_serialResponseBuffer[SERIAL_RESPONSE_BUFFER_LEN] = { 0 };


CShield::CShield() : isPortOpened_(false)
{

}



bool CShield::openPort(const std::string & fileName, const unsigned int baudrate)
{
    if (true == isPortOpened_)
    {
        std::cout << "iotbot::CShield::openPort() -> port is already opened, close it first with closePort()" << std::endl;
        return false;
    }
    
    try
    {
        uart_ = std::make_unique<mraa::Uart>(fileName);
    }
    catch (std::exception & e)
    {
        std::cerr << "iotbot::CShield::openPort() -> error in constructor mraa::Uart(): " << e.what() << std::endl;
        return false;
    }

    if (false == setBaudrate(baudrate))
    {
        return false;
    }

    if (false == setMode(8, mraa::UART_PARITY_NONE, 1))
    {
        return false;
    }

    if (false == setFlowControl(false, false))
    {
        return false;
    }

    std::cout << "iotbot::CShield::openPort() -> " << fileName << " (baudrate: " << baudrate << ", mode: 8N1) has been successfully opened" << std::endl;

    isPortOpened_ = true;
    return true;
}



bool CShield::openPort(const std::string & paramFilePath)
{
    if (true == isPortOpened_)
    {
        std::cout << "iotbot::CShield::openPort() -> port is already opened, close it first with closePort()" << std::endl;
        return false;
    }

    initParamStruct();

    if (false == readParamFile(paramFilePath))
    {
        return false;
    }

    // ******************************************** port ********************************************
    std::string port = "/dev/ttyS1";

    if (true == params_.port.first)
    {
        port = params_.port.second;
    }
    else
    {
        std::cout << "iotbot::CShield::openPort() -> no port device name in parameter file specified (trying default value: " << port << ")" << std::endl;
    }

    try
    {
        uart_ = std::make_unique<mraa::Uart>(port);
    }
    catch (std::exception & e)
    {
        std::cerr << "iotbot::CShield::openPort() -> error in constructor mraa::Uart(): " << e.what() << std::endl;
        return false;
    }

    // ******************************************** baudrate ********************************************
    unsigned int baudrate = 115200;
    
    if (true == params_.baudrate.first)
    {
        baudrate = params_.baudrate.second;
    }
    else
    {
        std::cout << "iotbot::CShield::openPort() -> no baudrate in parameter file specified (using default value: " << baudrate << ")" << std::endl;
    }

    if (false == setBaudrate(baudrate))
    {
        return false;
    }

    // ******************************************** data bits ********************************************
    unsigned int dataBits = 8;

    if (true == params_.dataBits.first)
    {
        switch (params_.dataBits.second)
        {
        case 5:
            dataBits = 5;
            break;
        case 6:
            dataBits = 6;
            break;
        case 7:
            dataBits = 7;
            break;
        case 8:
            dataBits = 8;
            break;
        default:
            std::cout << "iotbot::CShield::openPort() -> no valid data bits in parameter file specified (using default value: " << dataBits << ")" << std::endl;
        }
    }
    else
    {
        std::cout << "iotbot::CShield::openPort() -> no data bits in parameter file specified (using default value: " << dataBits << ")" << std::endl;
    }

    // ******************************************** parity ********************************************
    mraa::UartParity parity = mraa::UART_PARITY_NONE;

    if (true == params_.parity.first)
    {
        if ("NONE" == params_.parity.second)
        {
            parity = mraa::UART_PARITY_NONE;
        }
        else if ("EVEN" == params_.parity.second)
        {
            parity = mraa::UART_PARITY_EVEN;
        }
        else if ("ODD" == params_.parity.second)
        {
            parity = mraa::UART_PARITY_ODD;
        }
        else
        {
            std::cout << "iotbot::CShield::openPort() -> no valid parity mode in parameter file specified (using default value: no parity)" << std::endl;
        }
    }
    else
    {
        std::cout << "iotbot::CShield::openPort() -> no parity in parameter file specified (using default value: no parity)" << std::endl;
    }

    // ******************************************** stop bits ********************************************
    unsigned int stopBits = 1;

    if (true == params_.stopBits.first)
    {
        if (1 == params_.stopBits.second)
        {
            stopBits = 1;
        }
        else if (2 == params_.stopBits.second)
        {
            stopBits = 2;
        }
        else
        {
            std::cout << "iotbot::CShield::openPort() -> no valid stop bits mode in parameter file specified (using default value: " << stopBits << ")" << std::endl;
        }
    }
    else
    {
        std::cout << "iotbot::CShield::openPort() -> no stop bits in parameter file specified (using default value: " << stopBits << ")" << std::endl;
    }

    // ******************************************** set UART mode with data bits, parity, stop bits ********************************************
    if (false == setMode(dataBits, parity, stopBits))
    {
        return false;
    }
    
    // ******************************************** flow control ********************************************
    bool xonXoff = false;
    bool rtsCts = false;

    if (true == params_.flowControl.first)
    {
        if ("NONE" == params_.flowControl.second)
        {
            xonXoff = false;
            rtsCts = false;
        }
        else if ("XON_XOFF" == params_.flowControl.second)
        {
            xonXoff = true;
            rtsCts = false;
        }
        else if ("RTS_CTS" == params_.flowControl.second)
        {
            xonXoff = false;
            rtsCts = true;
        }
        else
        {
            std::cout << "iotbot::CShield::openPort() -> no valid flow control mode in parameter file specified (using default values: XON/XOFF = off, RTS/CTS = off)" << std::endl;
        }
    }
    else
    {
        std::cout << "iotbot::CShield::openPort() -> no flow control in parameter file specified (using default values: XON/XOFF = off, RTS/CTS = off)" << std::endl;
    }

    if (false == setFlowControl(xonXoff, rtsCts))
    {
        return false;
    }

    std::cout << "iotbot::CShield::openPort() -> " << port << " has been successfully opened" << std::endl;

    isPortOpened_ = true;
    return true;
}



bool CShield::readParamFile(const std::string & filePath)
{
    std::ifstream paramFile(filePath);

    if (true == paramFile.is_open())
    {
        std::cout << "iotbot::CShield::readParamFile() -> read parameter file: " << filePath << std::endl;

        std::string lastParamHolder;

        for (std::istream_iterator<std::string> it(paramFile); it != std::istream_iterator<std::string>(); it++)
        {
            if ("com_port" == lastParamHolder)
            {
                params_.port.first = true;
                params_.port.second = *it;

                std::cout << "iotbot::CShield::readParamFile() -> setup com_port: " << params_.port.second << std::endl;
            }
            else if ("baud_rate" == lastParamHolder)
            {
                params_.baudrate.first = true;
                params_.baudrate.second = std::stoi(*it);

                std::cout << "iotbot::CShield::readParamFile() -> setup baud_rate: " << params_.baudrate.second << std::endl;
            }
            else if ("data_bits" == lastParamHolder)
            {
                params_.dataBits.first = true;
                params_.dataBits.second = std::stoi(*it);

                std::cout << "iotbot::CShield::readParamFile() -> setup data_bits: " << params_.dataBits.second << std::endl;
            }
            else if ("stop_bits" == lastParamHolder)
            {
                params_.stopBits.first = true;
                params_.stopBits.second = std::stoi(*it);

                std::cout << "iotbot::CShield::readParamFile() -> setup stop_bits: " << params_.stopBits.second << std::endl;
            }
            else if ("parity_mode" == lastParamHolder)
            {
                params_.parity.first = true;
                params_.parity.second = *it;

                std::cout << "iotbot::CShield::readParamFile() -> setup parity_mode: " << params_.parity.second << std::endl;
            }
            else if ("flow_control" == lastParamHolder)
            {
                params_.flowControl.first = true;
                params_.flowControl.second = *it;

                std::cout << "iotbot::CShield::readParamFile() -> setup flow_control: " << params_.flowControl.second << std::endl;
            }

            lastParamHolder = *it;
        }

        paramFile.close();
    }
    else
    {
        std::cerr << "iotbot::CShield::readParamFile() -> could not open file: " << filePath << std::endl;
        return false;
    }

    return true;
}



void CShield::closePort()
{
    std::cout << "iotbot::CShield::closePort()" << std::endl;

    uart_.reset(nullptr);
    isPortOpened_ = false;
}



bool CShield::isPortOpened() const
{
    return isPortOpened_;
}



void CShield::initParamStruct()
{
    params_.port.first = false;
    params_.baudrate.first = false;
    params_.dataBits.first = false;
    params_.stopBits.first = false;
    params_.parity.first = false;
    params_.flowControl.first = false;
}



bool CShield::setBaudrate(const unsigned int baudrate)
{
    if (mraa::SUCCESS != uart_->setBaudRate(baudrate))
    {
        std::cerr << "iotbot::CShield::setBaudrate() -> error setting baudrate on UART" << std::endl;
        return false;
    }

    return true;
}



bool CShield::setMode(const unsigned int dataBits, const mraa::UartParity parity, const unsigned int stopBits)
{
    if (mraa::SUCCESS != uart_->setMode(dataBits, parity, stopBits))
    {
        std::cerr << "iotbot::CShield::setMode() -> error setting modes on UART" << std::endl;
        return false;
    }

    return true;
}



bool CShield::setFlowControl(const bool xOnOff, const bool rtsCts)
{
    if (mraa::SUCCESS != uart_->setFlowcontrol(xOnOff, rtsCts))
    {
        std::cerr << "iotbot::CShield::setFlowControl() -> error setting flow control on UART" << std::endl;
        return false;
    }

    return true;
}



bool CShield::send(const SSerialRequestData & data)
{
    if (true == isPortOpened_)
    {
        if (false == fillTxBuffer(data))
        {
            return false;
        }

        int bytesWritten = 0;

        do
        {
            bytesWritten += uart_->write(reinterpret_cast<const char*>(&g_serialRequestBuffer[bytesWritten]), SERIAL_REQUEST_BUFFER_LEN - bytesWritten);

            //std::cout << "CShield::send() -> wrote " << bytesWritten << " bytes to UART" << std::endl;

            if (-1 == bytesWritten)
            {
                std::cerr << "iotbot::CShield::send() -> error occured while writing to UART" << std::endl;
                return false;
            }

        } while (SERIAL_REQUEST_BUFFER_LEN != bytesWritten);
    }
    else
    {
        std::cout << "iotbot::CShield::send() -> port is not opened, set it up with openPort()" << std::endl;
        return false;
    }

    return true;
}



bool CShield::receive(SSerialResponseData & data, const unsigned int waitTimeoutInMilliseconds)
{
    if (true == isPortOpened_)
    {
        if (false == uart_->dataAvailable(waitTimeoutInMilliseconds))
        {
            std::cerr << "iotbot::CShield::receive() -> timeout (" << waitTimeoutInMilliseconds << " msec) while waiting for data to receive" << std::endl;
            return false;
        }

        int bytesRead = 0;
        
        do
        {
            bytesRead += uart_->read(reinterpret_cast<char*>(&g_serialResponseBuffer[bytesRead]), SERIAL_RESPONSE_BUFFER_LEN - bytesRead);

            //std::cout << "iotbot::CShield::receive() -> read " << bytesRead << " bytes from UART" << std::endl;

        } while (SERIAL_RESPONSE_BUFFER_LEN != bytesRead);

        if (false == fillDataFromRxBuffer(data))
        {
            return false;
        }
    }
    else
    {
        std::cout << "iotbot::CShield::receive() -> port is not opened, set it up with openPort()" << std::endl;
        return false;
    }

    return true;
}



bool CShield::fillTxBuffer(const SSerialRequestData & data)
{
    // IMPORTANT !!! -> order of fillTxBuffer() must be strictly followed by order of the UART request protocol

    setTxBufferWith(data.start, ESerialRequestOffset::REQ_START);

    setTxBufferWith(data.command, ESerialRequestOffset::REQ_COMMAND);

    unsigned int offset = ESerialRequestOffset::REQ_PAYLOAD;
    unsigned int dataSize = sizeof(data.payload[ESerialRequestOffset::REQ_PAYLOAD]);
    for (const auto & it : data.payload)
    {
        setTxBufferWith(it, offset);
        offset += dataSize;
    }

    // ******************************************** CRC checksum calculation ********************************************
    std::uint8_t crc = 0;
    crc = CRC::Calculate(g_serialRequestBuffer, sizeof(g_serialRequestBuffer - 1), CRC::CRC_8()); // calculate checksum over TX data buffer without CRC byte

    if (0 == crc)
    {
        std::cout << "iotbot::CShield::fillTxBuffer() -> could not calculate the CRC checksum" << std::endl;
        return false;
    }

    setTxBufferWith(crc, ESerialRequestOffset::REQ_CRC);
    return true;
}



bool CShield::fillDataFromRxBuffer(SSerialResponseData & data)
{
    // IMPORTANT !!! -> order of fillDataFromRxBuffer() must be strictly followed by order of the UART response protocol

    getValueFromRxBuffer(data.command, ESerialResponseOffset::RES_COMMAND);

    unsigned int offset = ESerialResponseOffset::RES_RPM;
    unsigned int dataSize = sizeof(data.rpm[ESerialResponseOffset::RES_RPM]);
    for (auto & it : data.rpm)
    {
        getValueFromRxBuffer(it, offset);
        offset += dataSize;
    }

    offset = ESerialResponseOffset::RES_IMU;
    dataSize = sizeof(data.imu[ESerialResponseOffset::RES_IMU]);
    for (auto & it : data.imu)
    {
        getValueFromRxBuffer(it, offset);
        offset += dataSize;
    }

    offset = ESerialResponseOffset::RES_TOF;
    dataSize = sizeof(data.tof[ESerialResponseOffset::RES_TOF]);
    for (auto & it : data.tof)
    {
        getValueFromRxBuffer(it, offset);
        offset += dataSize;
    }

    getValueFromRxBuffer(data.voltage, ESerialResponseOffset::RES_VOLTAGE);

    getValueFromRxBuffer(data.temperature, ESerialResponseOffset::RES_TEMPERATURE);

    // ******************************************** CRC checksum calculation ********************************************
    std::uint8_t crcFromBuffer = 0;
    getValueFromRxBuffer(crcFromBuffer, ESerialResponseOffset::RES_CRC);

    std::uint8_t crc = 0;
    crc = CRC::Calculate(g_serialResponseBuffer, sizeof(g_serialResponseBuffer - 1), CRC::CRC_8()); // calculate checksum over RX data buffer without CRC byte

    if (crcFromBuffer != crc)
    {
        std::cout << "iotbot::CShield::fillDataFromRxBuffer() -> CRC checksum is not equal to the received checksum, so data was probably lost during transfer" << std::endl;
        return false;
    }

    return true;
}



template <typename T>
void CShield::setTxBufferWith(const T & value, const unsigned int offset)
{
    const unsigned int valSize = sizeof(T);

    if (SERIAL_REQUEST_BUFFER_LEN < (offset + valSize))
    {
        std::cerr << "iotbot::CShield::setTxBufferWith() -> memory failure possible because tx buffer length ("
                  << SERIAL_REQUEST_BUFFER_LEN << ") < than offset (" << offset << ") + value size (" << valSize << ")" << std::endl;
        
        return;
    }

    // Assign value byte(s) to tx buffer data at given offset
    std::memcpy(&g_serialRequestBuffer[offset], &value, valSize);
}



template <typename T>
void CShield::getValueFromRxBuffer(T & value, const unsigned int offset)
{
    const unsigned int valSize = sizeof(T);

    if (SERIAL_RESPONSE_BUFFER_LEN < (offset + valSize))
    {
        std::cerr << "iotbot::CShield::getValueFromRxBuffer() -> memory failure possible because rx buffer length ("
                  << SERIAL_RESPONSE_BUFFER_LEN << ") < than offset (" << offset << ") + value size (" << valSize << ")" << std::endl;
        
        return;
    }
    
    // Assign rx buffer data at given offset to value byte(s)
    std::memcpy(&value, &g_serialResponseBuffer[offset], valSize);
}


} // namespace iotbot {
