#pragma once

#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> 
#include <errno.h> 
#include <termios.h>
#include <bitset>
#include <streambuf>
#include <string>
#include <array>

namespace cwtech
{

class UsbSerialPort
{	
private:
	bool m_status;
	int m_usb_fd;
public:
	UsbSerialPort(std::string target, speed_t baudrate)
		: m_status(true)
	{
		int USB = open(target.c_str(), O_RDWR| O_NOCTTY );

		struct termios tty;
		struct termios tty_old;
		memset (&tty, 0, sizeof tty);
		
		/* Error Handling */
		if ( tcgetattr ( USB, &tty ) != 0 ) 
		{
			m_status = false;
		}
		
		/* Save old tty parameters */
		tty_old = tty;
		
		/* Set Baud Rate */
		cfsetospeed (&tty, baudrate);
		cfsetispeed (&tty, baudrate);
		
		/* Set Parameters */
		tty.c_cflag     &=  ~PARENB;
		tty.c_cflag     &=  ~CSTOPB;
		tty.c_cflag     &=  ~CSIZE;
		tty.c_cflag     |=  CS8;
		
		tty.c_cflag     &=  ~CRTSCTS;           
		tty.c_cc[VMIN]   =  1;                  
		tty.c_cc[VTIME]  =  5;                  
		tty.c_cflag     |=  CREAD | CLOCAL;     
		
		/* Make raw */
		cfmakeraw(&tty);
		
		/* Flush Port, then applies attributes */
		tcflush( USB, TCIFLUSH );
		if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
			m_status = false;
		}

		m_usb_fd = USB;
	}

	// NOTE: reads until \r or end of line
	std::string Read()
	{
		char buf = '\0';
		std::string result;	
		int n = 0;

		do 
		{
			n = read(m_usb_fd, &buf, 1);
			result.push_back(buf);
		} while(buf != '\r' && n > 0);

		if(n < 0)
		{
			m_status = false;
		}

		return result;	
	}

	void Write(std::string msg)
	{
		int n = 0;
		for(auto& c : msg)
		{
			n = write(m_usb_fd, &c, 1);
			if(n <= 0)
			{
				break;
			}
		}
		if(n < 0)
		{
			m_status = false;
		}
	}


};

}


