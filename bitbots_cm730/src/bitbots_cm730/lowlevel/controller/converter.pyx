import rospy
from libc.math cimport fabs
from bitbots_common.utilCython.datavector cimport DataVector
from bitbots_common.utilCython.pydatavector cimport PyDataVector, PyIntDataVector

cdef class Converter:
    """
    Basisconverter
    """
    cdef encode(self, object value, ubyte *result):
        """
        bla
        """
        raise NotImplementedError()

    cdef decode(self, ubyte *data): raise NotImplementedError()

    cdef int length(self):
        return self._length


cdef class ByteConverter(Converter):
    def __init__(self):
        self._length = 1

    cdef encode(self, object value, ubyte *result):
        cdef int bv = <ubyte>value
        if bv & ~0xff != 0:
            raise ValueError("Value must be between 0 and 0xff")

        result[0] = bv

    cdef decode(self, ubyte *data):
        return <int>data[0]


cdef class WordConverter(Converter):
    def __init__(self):
        self._length = 2

    cdef encode(self, object value, ubyte *result):
        cdef int iv = value
        if not(0 <= iv <= 0xffff):
            raise ValueError("Value needs to be between 0 and 0xffff")

        result[0] = iv & 0xff
        result[1] = (iv >> 8) & 0xff

    cdef object decode(self, ubyte *data):
        return (data[1] << 8) | data[0]


cdef class LoadConverter(Converter):
    def __init__(self):
        self._length = 2

    cdef encode(self, object value, ubyte *result):
        raise NotImplementedError()

    cdef object decode(self, ubyte *data):
        cdef int value = (data[1] << 8) | data[0]
        cdef float fl = (value & 0x1ff) / 1024.0
        return fl if value & 0x200 else -fl


cdef class ColorConverter(Converter):
    def __init__(self):
        self._length = 2

    cdef encode(self, object value, ubyte *result):
        cdef int r, g, b, word

        r, g, b = value
        if not(0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
            raise ValueError("Colorvalue not in the range 0 to 0xff")

        word = ((b>>3) << 10) | ((g>>3)<<5) | (r>>3)
        result[0] = word & 0xff
        result[1] = (word >> 8) & 0xff

    cdef object decode(self, ubyte *data):
        cdef object r, g, b
        cdef int word

        word = (data[1] << 8) + data[0]
        r = (word & 0x1f) << 3
        g = ((word >> 5) & 0x1f) << 3
        b = ((word >> 10) & 0x1f) << 3
        return (r, g, b)


cdef class RawBytesConverter(Converter):
    def __init__(self, int count):
        self._length = count

    cdef encode(self, object value, ubyte *result):
        cdef int i
        cdef char *chars = <bytes?>value
        for i in range(self._length):
            result[i] = chars[i]

    cdef object decode(self, ubyte *data):
        return <bytes>(<char*>data)[:self._length]



cdef class VectorConverter(Converter):
    def __init__(self, bool revert=False, int_data=False):
        self._length = 6
        self.revert = revert
        self.int_data = int_data

    cdef encode(self, object value, ubyte *result):
        cdef DataVector *dv = (<PyDataVector>value).get_data_vector()
        cdef int x = <int>(dv.get_x() + <float>0.5)
        cdef int y = <int>(dv.get_y() + <float>0.5)
        cdef int z = <int>(dv.get_z() + <float>0.5)

        if self.revert:
            result[4] = x & 0xff
            result[5] = (x >> 8) & 0xff
            result[2] = y & 0xff
            result[3] = (y >> 8) & 0xff
            result[0] = z & 0xff
            result[1] = (z >> 8) & 0xff

        else:
            result[0] = x & 0xff
            result[1] = (x >> 8) & 0xff
            result[2] = y & 0xff
            result[3] = (y >> 8) & 0xff
            result[4] = z & 0xff
            result[5] = (z >> 8) & 0xff

    cdef object decode(self, ubyte *data):
        cdef a = (data[1] << 8) | data[0]
        cdef b = (data[3] << 8) | data[2]
        cdef c = (data[5] << 8) | data[4]
        if self.int_data:
            if self.revert:
                return PyIntDataVector(c, b, a)
            return PyIntDataVector(a, b, c)
        else:
            if self.revert:
                return PyDataVector(c, b, a)
            return PyDataVector(a, b, c)


cdef int angle_to_raw(float angle):
    return int(angle * 4096 / 360 + 2048)

cdef float raw_to_angle(int raw):
    return (raw - 2048.0) * 360.0 / 4096.0

cdef class AngleConverter(Converter):
    def __init__(self):
        self._length = 2

    cdef encode(self, object value, ubyte *result):
        cdef float angle = <float>value
        if not (-180 <= angle <= 180):
            raise ValueError("Angle %d not between -180 and 180 degree" % angle)

        cdef int iv = angle_to_raw(angle)
        result[0] = iv & 0xff
        result[1] = (iv >> 8) & 0xff

    cdef object decode(self, ubyte *data):
        return raw_to_angle((data[1] << 8) | data[0])


cdef class SpeedConverter(Converter):
    def __init__(self):
        self._length = 2

    cdef encode(self, object value, ubyte *result):
        cdef float speed = value
        speed /= (117.07 / 1023.0) * 360 / 60 # degree / sec to revolution/ min and conversion to motor scale
        speed = fabs(speed)

        cdef int iv
        if speed == 0:
            iv = 0
        elif speed < 1:
            iv = 1
        elif speed > 1023:
            iv = 1023
        else:
            iv = <int>(speed + <float>0.5)

        result[0] = iv & 0xff
        result[1] = (iv >> 8) & 0xff

    cdef object decode(self, ubyte *data):
        cdef int iv = (data[1] << 8) | data[0]
        return iv * ((117.07 / 1023.0) * 360 / 60)


cdef class SignedSpeedConverter(Converter):
    def __init__(self):
        self._length = 2

    cdef encode(self, object value, ubyte *result):
        raise NotImplementedError("We dont need this")

    cdef object decode(self, ubyte *data):
        cdef int iv = (data[1] << 8) | data[0]
        cdef sing = iv & 0x200 # the 10th bit is the sing
        iv = iv & 0x1FF # use the other bits
        cdef float result = iv * ((117.07 / 1023.0) * 360 / 60)
        return result if sing else result * -1
