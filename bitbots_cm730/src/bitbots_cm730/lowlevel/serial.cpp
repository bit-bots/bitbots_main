#include "serial.hpp"

#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include <iostream>
#include <stdexcept>

#include <boost/system/system_error.hpp>

using boost::system::system_error;
using boost::system::get_generic_category;
using namespace IPC::IO;

Serial::Serial(const std::string& device)
    : device(device), fd(-1), baudrate(1) {

    open();
    set_speed(1e6);
}

Serial::~Serial() {
    if(fd != -1)
        ::close(fd);
}

void Serial::write(const uint8_t *data, size_t length) {
    ::tcflush(fd, TCIFLUSH);

    if(::write(fd, data, length) != length)
        throw system_error(errno, get_generic_category(), "Writing to serial interface");

    // Warten bis alles geschrieben ist
    if(::tcdrain(fd) == -1)
        throw system_error(errno, get_generic_category(), "Not all data was written");
}

int Serial::read(uint8_t *data, size_t length) {
    //double timeout = (0.0003 + (20 * length) / baudrate) / 4;
    //double timeout = ( 121 * length) / baudrate;
    double timeout = ( 200 * length) / baudrate;
    //double timeout = ( 600 * length) / baudrate;
    // Teoretical: 1 byte = 10.5 symbols on the bus + latency in the cabel and at processing the answer
    //::usleep((long)(1e6 * timeout));

    int rc = 0;
    int read;
    int i = 0;
    struct timeval start, now;

    gettimeofday (&start, NULL);

    while(rc != length)
    {
        read = ::read(fd, data+rc, length - rc);
        if(read == -1)
        {
            throw system_error(errno, get_generic_category(), "Reading from serial interface");
        }
        rc += read;
        gettimeofday (&now, NULL);
        if ((now.tv_sec  - start.tv_sec) * 1000000 +
             now.tv_usec - start.tv_usec > 1e6 * timeout)
            {
                break;    // timeout
            }
        ::usleep(5);
        i+=1;
    }
    if(rc != length) {
        //std::cout << "Timeout: " << 1e6 * timeout << " us, " << rc << " of " << length << " in " << i * 5 << " us Sleep " << now.tv_usec - start.tv_usec << " us real time"<< std::endl;
        if (rc == 0)
        {
            throw std::ios_base::failure("Reading from serial interface: Not enough data");
        }
        //we just let the requester handel the problems, if there are some bytes
    }
    //std::cout << "OK: Timeout was: " << 1e6 * timeout << " us, " << rc << " of " << length << " in " << i * 5 << " us Sleep " << now.tv_usec - start.tv_usec << " us real time"<< std::endl;
    return rc;
}

void Serial::open() {
    if((fd = ::open(device.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0)
        throw system_error(errno, get_generic_category(), "Error opening the device");
}

void Serial::set_speed(double baudrate) {
    this->baudrate = baudrate;

	// You must set 38400bps!
    termios newtio;
	::memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag      = B38400|CS8|CLOCAL|CREAD;
    newtio.c_iflag      = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;
    if(::tcsetattr(fd, TCSANOW, &newtio) < 0)
        throw system_error(errno, get_generic_category(), "Error setting attributes");

	// Set non-standard baudrate
    serial_struct serinfo;
    if(::ioctl(fd, TIOCGSERIAL, &serinfo) >= 0) {
        serinfo.flags &= ~ASYNC_SPD_MASK;
        serinfo.flags |= ASYNC_SPD_CUST;
        serinfo.custom_divisor = serinfo.baud_base / baudrate;

        if(::ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
            throw system_error(errno, get_generic_category(), "Error setting baudrate");
    } else {
        std::clog << "Warning, can not set custom baudrate" << std::endl;
    }

    ::tcflush(fd, TCIFLUSH);
}

