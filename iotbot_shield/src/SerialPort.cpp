#include "SerialPort.h"


namespace iotbot { namespace serial {


CSerialPort::CSerialPort() : isPortOpened_(false), interruptDetected_(false)
{

}



CSerialPort::~CSerialPort()
{
    delete uart_;
}



bool CSerialPort::openPort(const std::string & fileName, const unsigned int baudrate, const bool useInterrupt)
{
    try
    {
        uart_ = new mraa::Uart(fileName);
    }
    catch (std::exception& e)
    {
        std::cerr << "iotbot::serial::CSerialPort::openPort() -> error in constructor mraa::Uart(): " << e.what() << std::endl;
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

    if (true == useInterrupt)
    {
        // setInterrupt();
    }

    std::cout << "iotbot::serial::CSerialPort::openPort() -> " << fileName << " (baudrate: " << baudrate << ") has been successfully opened" << std::endl;

    isPortOpened_ = true;
    return true;
}



bool CSerialPort::openPort(const std::string & paramFilePath)
{
    isPortOpened_ = false;
    initParamStruct();

    // ******************************************** input parameter file ********************************************
    if (false == readParamFile(paramFilePath))
    {
        return false;
    }

    // ******************************************** port ********************************************
    try
    {
        uart_ = new mraa::Uart(params_.port.second);
    }
    catch (std::exception & e)
    {
        std::cerr << "iotbot::serial::CSerialPort::openPort() -> error in constructor mraa::Uart(): " << e.what() << std::endl;
        return false;
    }

    // ******************************************** baudrate ********************************************
    if (true == params_.baudrate.first)
    {
        if (false == setBaudrate(params_.baudrate.second))
        {
            return false;
        }
    }
    else
    {
        std::cout << "iotbot::serial::CSerialPort::openPort() -> no baudrate in parameter file specified (using default values)" << std::endl;

        if (false == setBaudrate(115200))
        {
            return false;
        }
    }

    // ******************************************** parity ********************************************
    if (true == params_.parity.first)
    {
        if ("NONE" == params_.parity.second)
        {
            if (false == setMode(params_.dataBits.second, mraa::UART_PARITY_NONE, params_.stopBits.second))
            {
                return false;
            }
        }
        else if ("EVEN" == params_.parity.second)
        {
            if (false == setMode(params_.dataBits.second, mraa::UART_PARITY_EVEN, params_.stopBits.second))
            {
                return false;
            }
        }
        else if ("ODD" == params_.parity.second)
        {
            if (false == setMode(params_.dataBits.second, mraa::UART_PARITY_ODD, params_.stopBits.second))
            {
                return false;
            }
        }
        else
        {
            std::cout << "iotbot::serial::CSerialPort::setParity() -> no valid parity mode in parameter file specified (using default values)" << std::endl;
            
            if (false == setMode(params_.dataBits.second, mraa::UART_PARITY_NONE , params_.stopBits.second))
            {
                return false;
            }
        }
    }
    else
    {
        std::cout << "iotbot::serial::CSerialPort::openPort() -> no parity in parameter file specified (using default values)" << std::endl;

        if (false == setMode(8, mraa::UART_PARITY_NONE , 1))
        {
            return false;
        }
    }
    
    // ******************************************** flow control ********************************************
   if (true == params_.flowControl.first)
    {
        if ("NONE" == params_.flowControl.second)
        {
            if (false == setFlowControl(false, false))
            {
                return false;
            }
        }
        else if ("RTS_CTS" == params_.flowControl.second)
        {
            if (false == setFlowControl(false, true))
            {
                return false;
            }
        }
        else if ("XON_XOFF" == params_.flowControl.second)
        {
            if (false == setFlowControl(true, false))
            {
                return false;
            }
        }
        else
        {
            std::cout << "iotbot::serial::CSerialPort::setFlowControl() -> no valid flow control mode in parameter file specified (using default values)" << std::endl;
            
            if (false == setFlowControl(false, false))
            {
                return false;
            }
        }
    }
    else
    {
        std::cout << "iotbot::serial::CSerialPort::openPort() -> no flow control in parameter file specified (using default values)" << std::endl;

        if (false == setFlowControl(false, false))
        {
            return false;
        }
    }

    // ******************************************** interrupt ********************************************
    if ("YES" == params_.interrupt.second)
    {
        // setInterrupt();
    }

    std::cout << "iotbot::serial::CSerialPort::openPort() -> " << params_.port.second << " has been successfully opened" << std::endl;

    isPortOpened_ = true;
    return true;
}



bool CSerialPort::readParamFile(const std::string & filePath)
{
    std::ifstream paramFile(filePath);

    if (paramFile)
    {
        std::cout << "iotbot::serial::CSerialPort::openPort() -> read parameter file: " << filePath << std::endl;

        std::string lastParamHolder = "";

        for (std::istream_iterator<std::string> it(paramFile); it != std::istream_iterator<std::string>(); it++)
        {
            if ("com_port" == lastParamHolder)
            {
                params_.port.first = true;
                params_.port.second = *it;

                std::cout << "iotbot::serial::CSerialPort::openPort() -> setup com_port: " << params_.port.second << std::endl;
            }
            else if ("baud_rate" == lastParamHolder)
            {
                params_.baudrate.first = true;
                params_.baudrate.second = std::stoi(*it);

                std::cout << "iotbot::serial::CSerialPort::openPort() -> setup baud_rate: " << params_.baudrate.second << std::endl;
            }
            else if ("data_bits" == lastParamHolder)
            {
                params_.dataBits.first = true;
                params_.dataBits.second = std::stoi(*it);

                std::cout << "iotbot::serial::CSerialPort::openPort() -> setup data_bits: " << params_.dataBits.second << std::endl;
            }
            else if ("stop_bits" == lastParamHolder)
            {
                params_.stopBits.first = true;
                params_.stopBits.second = std::stoi(*it);

                std::cout << "iotbot::serial::CSerialPort::openPort() -> setup stop_bits: " << params_.stopBits.second << std::endl;
            }
            else if ("parity_mode" == lastParamHolder)
            {
                params_.parity.first = true;
                params_.parity.second = *it;

                std::cout << "iotbot::serial::CSerialPort::openPort() -> setup parity_mode: " << params_.parity.second << std::endl;
            }
            else if ("flow_control" == lastParamHolder)
            {
                params_.flowControl.first = true;
                params_.flowControl.second = *it;

                std::cout << "iotbot::serial::CSerialPort::openPort() -> setup flow_control: " << params_.flowControl.second << std::endl;
            }
            else if ("use_interrupt" == lastParamHolder)
            {
                params_.interrupt.first = true;
                params_.interrupt.second = *it;

                std::cout << "iotbot::serial::CSerialPort::openPort() -> setup use_interrupt: " << params_.interrupt.second << std::endl;
            }

            lastParamHolder = *it;
        }

        paramFile.close();
    }
    else
    {
        std::cerr << "iotbot::serial::CSerialPort::openPort() -> could not open file: " << filePath << std::endl;
        return false;
    }

    return true;
}



void CSerialPort::closePort()
{
    std::cout << "iotbot::serial::CSerialPort::closePort()" << std::endl;

    delete uart_;
    isPortOpened_ = false;
}



bool CSerialPort::isPortOpened() const
{
    return isPortOpened_;
}



void CSerialPort::initParamStruct()
{
    params_.port.first = false;
    params_.baudrate.first = false;
    params_.dataBits.first = false;
    params_.stopBits.first = false;
    params_.parity.first = false;
    params_.flowControl.first = false;
    params_.interrupt.first = false;
}



bool CSerialPort::setBaudrate(const unsigned int baudrate)
{
    if (mraa::SUCCESS != uart_->setBaudRate(baudrate))
    {
        std::cerr << "iotbot::serial::CSerialPort::setBaudrate() -> error setting baudrate on UART" << std::endl;
        return false;
    }

    return true;
}



bool CSerialPort::setMode(const unsigned int dataBits, const mraa::UartParity parity, const unsigned int stopBits)
{
    if (mraa::SUCCESS != uart_->setMode(dataBits, parity, stopBits))
    {
        std::cerr << "iotbot::serial::CSerialPort::setMode() -> error setting modes on UART" << std::endl;
        return false;
    }

    return true;
}



bool CSerialPort::setFlowControl(const bool xOnOff, const bool rtsCts)
{
    if (mraa::SUCCESS != uart_->setFlowcontrol(xOnOff, rtsCts))
    {
        std::cerr << "iotbot::serial::CSerialPort::setFlowControl() -> error setting flow control on UART" << std::endl;
        return false;
    }

    return true;
}



void CSerialPort::setInterrupt()
{
    // TODO !!!
}



void CSerialPort::irq_handler()
{
    // std::cout << "CSerialPort::irq_handler() -> interrupt detected" << std::endl;

    // TODO !!!

    interruptDetected_ = true;
}



void CSerialPort::processInterruptData()
{
    // TODO !!!

    interruptDetected_ = false;
}



bool CSerialPort::send(const SSerialRequestData & data)
{
    if (true == isPortOpened_)
    {
        fillTxBuffer(data);

        unsigned int bytesWritten = uart_->write(reinterpret_cast<const char*>(&g_serialRequestBuffer[0]), SERIAL_REQUEST_BUFFER_LEN);

        while (SERIAL_REQUEST_BUFFER_LEN != bytesWritten)
        {
            if (-1 == bytesWritten)
            {
                std::cerr << "iotbot::serial::CSerialPort::send() -> error occured while writing to UART" << std::endl;
                return false;
            }

            //std::cout << "iotbot::serial::CSerialPort::send() -> wrote " << bytesWritten << " bytes to UART" << std::endl;
            
            bytesWritten = uart_->write(reinterpret_cast<const char*>(&g_serialRequestBuffer[bytesWritten]), SERIAL_REQUEST_BUFFER_LEN - bytesWritten);
        }
    }
    else
    {
        std::cout << "iotbot::serial::CSerialPort::send() -> port is not opened, set it up with openPort()" << std::endl;
        return false;
    }

    return true;
}



bool CSerialPort::receive(SSerialResponseData & data, const unsigned int waitTimeoutInMilliseconds)
{
    if (true == isPortOpened_)
    {
        if (false == uart_->dataAvailable(waitTimeoutInMilliseconds))
        {
            std::cerr << "iotbot::serial::CSerialPort::receive() -> timeout (" << waitTimeoutInMilliseconds << " msec) while waiting for data to receive" << std::endl;
            return false;
        }

        unsigned int bytesRead = uart_->read(reinterpret_cast<char*>(&g_serialResponseBuffer[0]), SERIAL_RESPONSE_BUFFER_LEN);

        while (SERIAL_RESPONSE_BUFFER_LEN != bytesRead)
        {
            //std::cout << "iotbot::serial::CSerialPort::receive() -> read " << bytesRead << " bytes from UART" << std::endl;
            
            bytesRead = uart_->read(reinterpret_cast<char*>(&g_serialResponseBuffer[bytesRead]), SERIAL_RESPONSE_BUFFER_LEN - bytesRead);
        }
        
        fillDataFromRxBuffer(data);
    }
    else
    {
        std::cout << "iotbot::serial::CSerialPort::receive() -> port is not opened, set it up with openPort()" << std::endl;
        return false;
    }

    return true;
}



void CSerialPort::fillTxBuffer(const SSerialRequestData & data)
{
    // IMPORTANT !!! -> order of fillTxBuffer() must be strictly followed by order of the UART request protocol

    setTxBufferWith(data.start, ESerialRequestOffset::REQ_START);

    setTxBufferWith(data.command, ESerialRequestOffset::REQ_COMMAND);

    unsigned int offsetCounter = ESerialRequestOffset::REQ_PAYLOAD;
    unsigned int dataSize = sizeof(data.payload[ESerialRequestOffset::REQ_PAYLOAD]);
    for (const auto &it : data.payload)
    {
        setTxBufferWith(it, offsetCounter);
        offsetCounter += dataSize;
    }

    setTxBufferWith(data.crc, ESerialRequestOffset::REQ_CRC); // -> TODO :include CRCpp header to set CRC
}



void CSerialPort::fillDataFromRxBuffer(SSerialResponseData & data)
{
    // IMPORTANT !!! -> order of fillDataFromRxBuffer() must be strictly followed by order of the UART response protocol

    getValueFromRxBuffer(data.command, ESerialResponseOffset::RES_COMMAND);

    unsigned int offsetCounter = ESerialResponseOffset::RES_RPM;
    unsigned int dataSize = sizeof(data.rpm[ESerialResponseOffset::RES_RPM]);
    for (auto &it : data.rpm)
    {
        getValueFromRxBuffer(it, offsetCounter);
        offsetCounter += dataSize;
    }

    offsetCounter = ESerialResponseOffset::RES_IMU;
    dataSize = sizeof(data.imu[ESerialResponseOffset::RES_IMU]);
    for (auto &it : data.imu)
    {
        getValueFromRxBuffer(it, offsetCounter);
        offsetCounter += dataSize;
    }

    offsetCounter = ESerialResponseOffset::RES_TOF;
    dataSize = sizeof(data.tof[ESerialResponseOffset::RES_TOF]);
    for (auto &it : data.tof)
    {
        getValueFromRxBuffer(it, offsetCounter);
        offsetCounter += dataSize;
    }

    getValueFromRxBuffer(data.voltage, ESerialResponseOffset::RES_VOLTAGE);

    getValueFromRxBuffer(data.temperature, ESerialResponseOffset::RES_TEMPERATURE);

    getValueFromRxBuffer(data.crc, ESerialResponseOffset::RES_CRC); // -> TODO :include CRCpp header to set CRC
}



template <typename T>
void CSerialPort::setTxBufferWith(const T & value, const unsigned int offset)
{
    const unsigned int valSize = sizeof(T);

    if (SERIAL_REQUEST_BUFFER_LEN < (offset + valSize))
    {
        std::cerr << "iotbot::serial::CSerialPort::setTxBufferWith() -> memory failure possible because tx buffer length ("
                  << SERIAL_REQUEST_BUFFER_LEN << ") < than offset (" << offset << ") + value size (" << valSize << ")" << std::endl;
        return;
    }

    // Assign value byte(s) to tx buffer data at given offset
    std::memcpy(&g_serialRequestBuffer[offset], &value, valSize);
}



template <typename T>
void CSerialPort::getValueFromRxBuffer(T & value, const unsigned int offset)
{
    const unsigned int valSize = sizeof(T);

    if (SERIAL_RESPONSE_BUFFER_LEN < (offset + valSize))
    {
        std::cerr << "iotbot::serial::CSerialPort::getValueFromRxBuffer() -> memory failure possible because rx buffer length ("
                  << SERIAL_RESPONSE_BUFFER_LEN << ") < than offset (" << offset << ") + value size (" << valSize << ")" << std::endl;
        return;
    }
    
    // Assign rx buffer data at given offset to value byte(s)
    std::memcpy(&value, &g_serialResponseBuffer[offset], valSize);
}


}} // namespace iotbot } namespace serial
