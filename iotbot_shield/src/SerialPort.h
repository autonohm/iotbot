#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

#include <sys/types.h>
#include <termios.h>


struct ReturnData
{
	float rpmReturn[4];
	float voltage;
};


/**
 * @class SerialPort
 * @brief Serial interfacing class (UART).
 * @author Stefan May
 * @date 10.10.2015
 */
class SerialPort
{
public:

  /**
   * Constructor
   * @param[in] comPort device file link
   * @param[in] baud baud rate
   */
	SerialPort(const char* comPort, const speed_t baud);

	/**
	 * Destructor
	 */
	~SerialPort();

	/**
	 * Send data
	 * @param[in] msg send buffer
	 * @param[in] len length of send buffer
	 * @return bytes actually written
	 */
	bool send(float rpm[4]);

	/**
	 * Receive data
	 * @param[out] msg receive buffer, ensure that the buffer it at least the size of len
	 * @param[in] bytes to read
	 * @return true: received bytes==len, false: received bytes!=len
	 */
	//bool receive(std::shared_ptr<ReturnData> returnDataObj);
	bool receive(ReturnData& returnDataObj);

private:
	int _fd;

};




#endif /* _SERIAL_PORT_H_ */
