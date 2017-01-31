from .converter cimport ByteConverter, WordConverter, LoadConverter, SignedSpeedConverter, \
    SpeedConverter, AngleConverter, ColorConverter, RawBytesConverter, VectorConverter


cdef class Register:
    def __init__(self, int start, Converter conv):
        self.start = start
        self.conv = conv

    cdef encode(self, object value, ubyte *result):
        return self.conv.encode(value, result)

    cdef decode(self, ubyte *data):
        return self.conv.decode(data)

    cdef int length(self):
        return self.conv.length()

    cdef int get_start(self):
        return self.start


cdef class MX28RegisterTable(object):
    def __init__(self):
        """
        .. autoattribute::
        """
        self.model =            Register( 0, WordConverter())
        self.version =          Register( 2, ByteConverter())
        """ The Version of the Firmware (R/P)"""
        self.cid =              Register( 3, ByteConverter())
        self.baudrate =         Register( 4, ByteConverter())
        self.return_delay_time = Register( 5, ByteConverter())
        self.cw_angle_limit =   Register( 6, AngleConverter())
        self.ccw_angle_limit =  Register( 8, AngleConverter())
        self.system_data =      Register(10, ByteConverter()) # no doku!
        self.limit_temperatur = Register(11, ByteConverter())
        self.limit_voltage_low = Register(12, ByteConverter())
        self.limit_voltage_high = Register(13, ByteConverter())
        self.max_torque =       Register(14, WordConverter())
        self.return_level =     Register(16, ByteConverter())
        self.alarm_led  =       Register(17, ByteConverter())
        self.alarm_shutdown =   Register(18, ByteConverter())
        self.operating_mode =   Register(19, ByteConverter()) # no doku!
        self.low_calibration =  Register(20, AngleConverter()) # no doku!
        self.high_calibration = Register(22, AngleConverter()) # no doku!
        self.torque_enable =    Register(24, ByteConverter())
        self.led =              Register(25, ByteConverter())
        self.d =                Register(26, ByteConverter())
        self.i =                Register(27, ByteConverter())
        self.p =                Register(28, ByteConverter())
        self.reserved =         Register(29, ByteConverter()) # no doku!
        self.goal_position =    Register(30, AngleConverter())
        self.moving_speed =     Register(32, SpeedConverter())
        self.torque_limit =     Register(34, WordConverter())
        self.present_position = Register(36, AngleConverter())
        self.present_speed =    Register(38, SignedSpeedConverter())
        self.present_load =     Register(40, LoadConverter())
        self.present_voltage =  Register(42, ByteConverter())
        self.present_temperature = Register(43, ByteConverter())
        self.registered_instruction = Register(44, ByteConverter())
        self.pause_time =       Register(45, ByteConverter())
        self.moving =           Register(46, ByteConverter())
        self.lock =             Register(47, ByteConverter())
        self.puch =             Register(48, WordConverter())
        self.reserved4 =        Register(50, ByteConverter()) # no doku!
        self.reserved5 =        Register(51, ByteConverter()) # no doku!
        self.pot =              Register(52, WordConverter()) # no doku!
        self.pwm_out =          Register(54, WordConverter()) # no doku!
        self.p_error =          Register(56, WordConverter()) # no doku!
        self.i_error =          Register(58, WordConverter()) # no doku!
        self.d_error =          Register(60, WordConverter()) # no doku!
        self.p_error_out =      Register(62, WordConverter()) # no doku!
        self.i_error_out =      Register(64, WordConverter()) # no doku!
        self.d_error_out =      Register(66, WordConverter()) # no doku!
        # register 73 ...

        #tauschen von register 26 und 28 mit firmware > 28 oder so

    cpdef Register get_register_by_name(self, object name):
        return eval("self.%s" % name)


cdef class CM730RegisterTable:
    def __init__(self):
        self.version =          Register( 2, ByteConverter())
        self.cid =              Register( 3, ByteConverter())
        self.baudrate =         Register( 4, ByteConverter())
        self.dxl_power =        Register(24, ByteConverter())
        self.led_panel =        Register(25, ByteConverter())
        self.led_head =         Register(26, ColorConverter())
        self.led_eye =          Register(28, ColorConverter())
        self.button =           Register(30, ByteConverter())
        self.padding31_37 =     Register(31, RawBytesConverter(7))
        self.gyro =             Register(38, VectorConverter(True, True))
        self.accel =            Register(44, VectorConverter(False, True))
        self.voltage =          Register(50, ByteConverter())