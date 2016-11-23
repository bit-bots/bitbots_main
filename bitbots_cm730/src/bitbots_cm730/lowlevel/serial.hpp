#ifndef _SERIAL_HPP
#define _SERIAL_HPP

#include <stdint.h>
#include <string>

namespace IPC {
namespace IO {

class Serial {
private:
    int fd;
    double baudrate;
    std::string device;

    void open();

public:
    Serial(const std::string& name);
    ~Serial();

    void set_speed(double bps);
    void write(const uint8_t *data, size_t length);
    int read(uint8_t *data, size_t max_length);
};

} } //namespace

#endif

