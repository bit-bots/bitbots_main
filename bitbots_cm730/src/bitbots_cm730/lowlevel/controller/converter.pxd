from libc.stdint cimport uint8_t
from libcpp cimport bool

ctypedef uint8_t ubyte


cdef class Converter:
    cdef int _length

    cdef encode(self, object value, ubyte *result)
    cdef decode(self, ubyte *data)
    cdef int length(self)


cdef class ByteConverter(Converter):
    pass

cdef class WordConverter(Converter):
    pass

cdef class LoadConverter(Converter):
    pass

cdef class ColorConverter(Converter):
    pass

cdef class RawBytesConverter(Converter):
    pass

cdef class VectorConverter(Converter):
    cdef bool revert, int_data

cdef class AngleConverter(Converter):
    pass

cdef class SpeedConverter(Converter):
    pass

cdef class SignedSpeedConverter(Converter):
    pass