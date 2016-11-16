from libc.stdint cimport uint8_t
from libc.stddef cimport size_t

cdef import from "lowlevel/serial.hpp":
    cdef cppclass _Serial "IPC::IO::Serial":
        _Serial(char* device) except +IOError
        void write(uint8_t *data, size_t count) except +
        int read(uint8_t *data, size_t count) except +
        void set_speed(double bps) except +IOError

cdef class AbsSerial:

    cpdef read(self, int amount)
    cpdef write(self, bytes data)
    cpdef set_speed(self, double speed)

cdef class Serial(AbsSerial):
    cdef _Serial* serial

    cdef read_ptr(self, uint8_t *data, size_t count)
    cdef write_ptr(self, uint8_t *data, size_t count)

    cpdef read(self, int amount)
    cpdef write(self, bytes data)
    cpdef set_speed(self, double speed)

