# coding=utf-8
"""
Controller
^^^^^^^^^^

.. moduleauthor:: Nils Rokita <0rokita@informatik.uni-hamburg.de>


History:

* 8.9.15: Fix errors with stray bytes and corrupt data, lots of comments (Nils)
* Added MultiMotorError (Nils)

This Module handles the communication with the Hardware
"""
import rospy
from bitbots_cm730.lowlevel.controller.register_tables cimport MX28RegisterTable, CM730RegisterTable
from bitbots_cm730.lowlevel.controller.register_tables cimport Register


# Flag for debug prints in processing
DEBUG = False

class MultiMotorError(EnvironmentError):
    """
    This error contains multiple :class:`MotorError` because one bulk read
    can contain more then one motor with error.

    This object is iterable and returns a Exception
    """
    def __init__(self):
        self.errors = []
        self.packets = []

    def get_packets(self):
        """
        Return the parsed data for later processing in case of motor errors.
        This is necessary because some of the data is mostly ok, but we want to
        throw an exception so the rest of the software has to handle it
        """
        return self.packets

    def __iter__(self):
        return self.errors.__iter__()

    def add_motor_error(self, error):
        """
        Adds an error to the MultiMotorError
        :param error: the Exeption to add
        """
        self.errors.append(error)

    def set_packets(self, data):
        """
        Set the data packets for later use
        """
        self.packets = data

    def __repr__(self):
        return "MultiMotorError %s" % str(self.errors)

    def __str__(self):
        return self.__repr__()

class MotorError(EnvironmentError):
    """
    This error represent an error in the motor
    """
    def __init__(self, motor, error):
        self.motor = motor
        self.error = error
        self.error_str = get_error_list(self.error)
        if len(self.error_str) == 1:
            self.error_str = self.error_str[0]
        else:
            self.error_str = str(self.error_str).replace("'","")

    def __repr__(self):
        return "Motor %d had %s" % (self.motor, self.error_str)

    def __str__(self):
        return "Motor %d has an error: %s" % (self.motor, self.error_str)

    def get_error(self):
        """
        Returns the error
        """
        return self.error

    def get_error_str(self):
        """
        Returns the error as an string for debuging
        """
        return self.error_str

    def get_motor(self):
        """
        returns the ID of the Motor
        """
        return self.motor


MX28_REGISTER = MX28RegisterTable()
CM730_REGISTER = CM730RegisterTable()

def get_mx28_register_table():
    """
    Get the mx28 register table
    """
    return MX28

def get_cm730_register_table():
    """
    Get the register table of the cm730
    """
    return CM730

# IDs
ID_BROADCAST    = 254
ID_CM730        = 200

cdef enum:
    # Konstanten für die Typen der unterschiedlichen Packete
    INST_PING       = 0x01
    # INST_READ       = 0x02
    # INST_WRITE      = 0x03
    # INST_REG_WRITE  = 0x04
    # INST_ACTION     = 0x05
    # INST_RESET      = 0x06
    INST_SYNC_WRITE = 0x83
    INST_BULK_READ  = 0x92

from libc.string cimport memcpy

cdef class StatusPacket:
    cdef int cid
    cdef int err
    cdef int length
    cdef ubyte data[512]

    def __init__(self):
        self.length = 0

    cdef void set(self, ubyte *data, int n):
        memcpy(<char*>self.data, <char*>data + 5, n - 6)
        self.length = n - 6
        self.cid = data[2]
        self.err = data[4]

    cdef int is_error(self):
        return self.err != 0

    cdef int is_success(self):
        return self.err == 0

    cdef ubyte* ptr(self, int offset):
        return <ubyte*>self.data + offset

    def __repr__(self):
        fmt = "<StatusPacket id=0x%02x, error=%s, data=%s>"
        data = " ".join("%02x" % ord(b) for b in (<char*>self.data)[0:self.length])
        return fmt % (self.cid, bin(self.err), data)

cdef StatusPacket make_status_packet(ubyte *bytes, int n, bool corrupt_start=False):
    ''' Parsed ein StatusPacket aus dem Speicherbereich '''
    if n < 6:
        raise IOError("Can not decode Packet")

    if (not corrupt_start and bytes[0] != 0xff) or bytes[1] != 0xff:
        rospy.logdebug("StatusPacket must begin with 0xffff DATA:"<< repr((<char*>bytes)[0:n]))
        raise IOError("StatusPacket must begin with 0xffff")

    cdef StatusPacket sp = StatusPacket()
    sp.set(bytes, n)
    return sp

cdef int checksum(ubyte *data, int n):
    """
    Calculates the checksum for a Dynamixel V1 protocol

    :param data: The Bytestream
    :param n: The bytecount of data
    :return: The computed checksum
    """
    cdef int idx, sum = 0
    for idx in range(2, n):
        sum += data[idx]
    return ~sum & 0xff


cdef class BytePacket:
    """
    Dies ist eine abstrakte Basisklasse für alle `Anfrage`_-Pakete.
    Das grobe Layout eines Pakets wird durch diese Klasse festgelegt.
    Es werden alle wichtigen Eigenschaften wie die Prüfsumme und
    Länge des Pakets automatisch eingefügt.

    Im Konstruktor wird die Adresse der angesprochenen Hardware als Parameter
    *cid* übergeben, sowie der Typ des Pakets in *instruction*.
    Weitere Daten können dann über die Methode :func:`.add_bytes` in den
    Datenbereich geschrieben werden.

    Über das Attribute :attr:`.raw` kann eine Byterepräsentation des Pakets
    erfragt werden. Nachdem ein Paket abgesendet wurde, erfragt der
    :class:`~controller.Controller` über die Methode :func:`get_processor`
    eine Instanz von :class:`~controller.Processor`, um die Antwort zu
    verarbeiten.

    .. attribute:: raw
        Gibt das Paket als eine Abfolge von Bytes zurück.::

            >>> p = BytePacket(200, 0x02)
            >>> p.add_bytes((0xde, 0xad, 0xbe, 0xef))
            >>> p.raw
            '\xff\xff\xc8\x06\x02\xde\xad\xbe\xef\xf7'
    """
    def __init__(self, int cid, int instruction):
        ''' Erzeugt ein neues Packet mit cid und instruction '''
        self.idx = 5
        self.buffer[0] = self.buffer[1] = 0xff
        self.buffer[2] = cid
        self.buffer[4] = instruction

    cdef void add_byte(self, ubyte b):
        """
        Fügt weitere Bytes in den Datenbereich des Pakets ein. *seq* muss dabei
        ein :class:`tuple` mit den einzufügenden Bytes sein.
        Die Methode updatet automatisch die Länge und Prüfsumme
        des Pakets.
        """
        self.buffer[self.idx] = b
        self.idx += 1

    cdef ubyte* advance(self, int n):
        cdef ubyte* ptr = <ubyte*>self.buffer + self.idx
        self.idx += n
        return ptr

    property raw:
        def __get__(self):
            """ Konvertiert das Paket in einen String """
            self.buffer[3] = self.idx - 3
            self.buffer[self.idx] = checksum(self.buffer, self.idx)
            return (<char*>self.buffer)[:self.idx+1]

    cdef Processor get_processor(self):
        """
        Gibt eine Instanz von :class:`~controller.Processor` zurück, um die
        Antwort zu verarbeiten.

        Wird keine Antwort erwartet, wird None zurück gegeben
        """
        return None

    cdef list get_cids(self):
        """
        Returns a list of the CIDs whis paket is processing

        In the case of non bulk packets the list will only have one element
        :return: list of ints
        """
        return [self.buffer[2]]

cdef class Processor:
    """
    Standardimplementation eines Prozessors für die Antwort vom Darwin.
    Es werden einfach sechs Bytes (ein Status-Paket) gelesen.

    :members:
    """
    def __init__(self, int count=1, int length=6):
        self.count = count
        self.length = length

    cdef int get_answer_length(self):
        """
        Gibt die Anzahl der erwarteten Bytes an, die als Antwort gelesen
        werden müssen. Dies ist der im Konstruktor übergebene Wert *length*.
        """
        return self.length

    cdef int get_answer_count(self):
        """
        Gibt die Anzahl der erwarteten StatusPakete an. Dies ist
        der im Konstruktor für *count* übergebene Wert.
        """
        return self.count

    cdef process(self, list packets):
        """
        Nachdem die `Antwort`-Pakete gelesen und geparst wurden, werden sie
        dieser Methode als Liste übergeben. Die Pakete, die Daten in der
        Antwort erwarten, sollten einen eigenen Prozessor zurück geben, der
        die Antwort entsprechend parst und verarbeitet.

        Die Rückgabe dieser Methode wird von
        :func:`controller.Controller.process` an den Aufrufer zurückgegeben.
        """
        return None

    cdef int noanswer(self):
        """
        gibt True zurück wenn keine Antwort erwartet wird
        """
        return self.count * self.length == 0

cdef class PingPacket(BytePacket):
    def __init__(self, int cid):
        BytePacket.__init__(self, cid, INST_PING)

    cdef Processor get_processor(self):
        return PingProcessor()

cdef class PingProcessor(Processor):
    cdef process(self, list packets):
        if len(packets) == 0:
            rospy.logwarn("PingPacket: Got an emty response")
            return False
        return (<StatusPacket?>packets[0]).is_success()

cdef class BulkReadPacket(BytePacket):
    """
    Es lassen sich unterschiedliche Register in den unterschiedlichen
    Kontrollern auslesen.

    Die Antwort ist hier eine Zuordnung von Adressen auf Listen der
    gelesenen Werte.
    ::

        bp = controller.BulkReadPacket()
        for joint in self.robot.joints:
            bp.add(joint.cid, (MX.PRESENT_POSITION, MX.PRESENT_SPEED))

        bp.add(CM_ID, (CM.GYRO, CM.ACCEL))
        result = self.robot.ctrl.process(bp)
    """

    def __init__(self):
        BytePacket.__init__(self, ID_BROADCAST, INST_BULK_READ)
        self.registers = [None] * 256
        self.cids = []
        self.add_byte(0)

        self.processor = BulkReadProcessor()
        self.processor.registers = self.registers

    def add(self, int cid, tuple registers not None):
        """
        Liest aus dem Kontroller *cid* die Register *registers* aus.
        """
        if self.registers[cid] is not None:
            raise ValueError("Two entries for cid=%d" % cid)

        if not registers:
            raise ValueError("'registers' must not be empty")

        cdef tuple regs = tuple(registers)
        cdef int start = (<Register>regs[0]).start

        cdef int idx, count = 0
        cdef Register reg
        for idx in range(len(regs)):
            count += (<Register>regs[idx]).length()

        cdef Register prev, next
        for idx in range(len(regs) - 1):
            prev = regs[idx]
            next = regs[idx+1]
            if prev.start + prev.length() != next.start:
                raise ValueError("Your registers are not in sequence!")

        self.add_byte(count)
        self.add_byte(cid)
        self.add_byte(start)
        self.registers[cid] = regs
        self.cids.append(cid)

        (<Processor>self.processor).length += 6 + count
        (<Processor>self.processor).count += 1

    cdef Processor get_processor(self):
        return self.processor

    def __repr__(self):
        return u'<BulkReadPacket CIDs: %s>' % str(self.cids)

    cdef list get_cids(self):
        """
        Returns the list of all cids for wich the bulk packet is asking for data
        :return: list of int
        """
        return self.cids

cdef class BulkReadProcessor(Processor):
    """ Verarbeitet die Antwort auf ein BulkReadPacket """
    def __init__(self):
        Processor.__init__(self, 0, 0)

    cdef process(self, list packets):
        cdef int i, j, offset = 0
        cdef dict result = {}
        cdef tuple registers

        cdef StatusPacket packet
        cdef Register register

        cdef list values
        for i in range(len(packets)):
            offset = 0
            packet = packets[i]
            registers = self.registers[packet.cid]

            if registers is None:
                # sollte garnicht passieren
                continue

            values = []
            for j in range(len(registers)):
                register = registers[j]
                values.append(register.decode(packet.ptr(offset)))
                offset += register.length()

            result[packet.cid] = values

        for idx in range(256):
            if self.registers[idx] and not idx in result:
                rospy.logwarn("No answer for CID: " + str(idx) + " (BulkRead)")

        return result

cdef class SyncWritePacket(BytePacket):
    """
    Mit dem SyncWritePacket kann ein Registern in mehreren Motoren
    zeitgleich geschrieben werden.
    """
    def __init__(self, registers):
        BytePacket.__init__(self, ID_BROADCAST, INST_SYNC_WRITE)
        self.registers = tuple(registers)
        self.add_header()

    cdef add_header(self):
        cdef int idx, length = 0
        cdef Register register, prev = None

        for idx in range(len(self.registers)):
            register = self.registers[idx]
            length += register.length()
            if prev is not None:
                if prev.start + register.length() != register.start:
                    raise ValueError("Your registers are not in sequence")

            prev = register

        self.add_byte((<Register>self.registers[0]).start)
        self.add_byte(length)

    cpdef add(self, int cid, tuple values):
        cdef int idx
        cdef Register register

        if len(values) != len(self.registers):
            raise ValueError("You need to give a value for each register")

        self.add_byte(cid)
        for idx in range(len(self.registers)):
            register = self.registers[idx]
            register.encode(values[idx], self.advance(register.length()))


from bitbots_common.util import get_error_list

cdef class Controller:
    """
    Mit einem Controller Objekt kann man direkt in die
    Register-Tabelle eines Controllers hinein schreiben

    """
    def __init__(self, object serial):
        """
        Creates a new Controller on the Connection from serial

        :param serial: The serial controller to communicate over"""
        self.serial = serial

    cpdef process(self, BytePacket packet):
        """
        Verarbeitet ein Packet, indem es versendet wird, der
        entsprechende Prozessor dafür erzeugt und mit den
        empfangenden Daten gefüttert wird.
        Das Ergebnis des Prozessors wird dann zurück gegeben
        """
        cdef object errors = None
        self.send_packet(packet)

        cdef Processor processor = packet.get_processor()
        if processor is None or processor.noanswer():
            # Eventuell wird garkeine Antwort benötigt, dann abbrechen
            return None

        if DEBUG:
            rospy.logwarn("Process: " +  (str(packet)))

        cdef list responses = self.read_packets(
            processor.get_answer_count(),
            processor.get_answer_length(),
            packet.get_cids())

        if DEBUG:
            rospy.logwarn("Responses: " + str(responses))

        cdef StatusPacket statuspacket
        for statuspacket in responses:
            if statuspacket.is_error():
                if not errors:
                    errors = MultiMotorError()
                error = MotorError(statuspacket.cid, statuspacket.err)
                msg = ("Packet from %d has Error-Bits: %s" %
                    (error.get_motor(), error.get_error_str()))
                rospy.logerr(msg)
                err = error.get_error()
                # wir machen da mal ein packet draus (bitschifting ist toll)
                rospy.logerr("LastError", (statuspacket.err << 8) + statuspacket.cid)
                errors.add_motor_error(error)
        if errors:
            # antwort trotzdem verarbeiten
            errors.set_packets(processor.process(responses))
            # exception werfen
            raise errors
        return processor.process(responses)

    cdef send_packet(self, BytePacket packet):
        """ Sendet ein Packet an den Controller """
        cdef bytes buf = packet.raw
        self.serial.write(buf)

    cdef list read_packets(self, int count, int bytecount, list cids = []):
        """
        Reads count packets from the controller.

        :param count: The Number of packets to read
        :type count: int
        :param bytecount: The number of bytes to read
        :type bytecount: int
        :param cids: A list of cids we want to process, this list is for buss error correction purposes
        :type cids: list of ints
        :return: list of packets
        """

        cdef bytes pystr
        cdef int orig_bytecount = bytecount
        pystr, bytecount = self.serial.read(bytecount)
        if bytecount != orig_bytecount:
            rospy.logwarn("Got only %d of %d bytes, trying to interpret anyway!" % (bytecount, orig_bytecount))
        cdef ubyte* data = <ubyte*><char*>pystr

        #print "Data: ", hex(data[0]), hex(data[1]), hex(data[2]),hex(data[3]),hex(data[4]),hex(data[5])
        #print "langth ", bytecount

        cdef list packets = []
        cdef StatusPacket packet
        cdef int size, viewstart = 0, nr = 0
        cdef int found
        cdef int io_error = 0
        cdef bool corrupt_start = False

        while viewstart + 6 <= bytecount:
            corrupt_start = False  # Reset the Flag on new packet
            while (data[viewstart] != 0xff or data[viewstart+1] != 0xff) and viewstart + 6 <= bytecount:
                # This while loop searches for the beginning of the packet
                if data[viewstart] == 0xff and data[viewstart+1] in cids and \
                                data[viewstart+3] == 0 and viewstart + 5 <= bytecount and viewstart >= 0:
                    # we try some guesswork to see if the first 0xff was dropped, because wis happens because of
                    # some strange reasons. If the cid is in the wanted cids, and the error flag is 0 we try to
                    # interpret the packet.
                    if viewstart != 0 or data[viewstart - 1] != 0xff:
                        # if the byte befor viewstart is an 0xff this is a valid Packet, and was declined because of
                        # some checksum mismatch or similar
                        corrupt_start = True
                        # we decrement the viewstart so that the positions in the later code will match.
                        # this is possible because no access of viewstart + 0 is necessary as it is 0xff
                        # the only function doing this is make_status_packet and this function knows about the corrupt start
                        if bytecount != orig_bytecount:#
                            # if there are less bytes that it should be, we try recovering via strategy shift
                            viewstart -= 1
                            rospy.logwarn("Possible corrupted Haedder, try to interpret with an imaginarry 0xff in front")
                        else:
                            # Alternate recovery strategy found by observing that if the haedder is corrupt, an
                            # additional byte is found after the haedder
                            data[viewstart + 4] = data[viewstart + 3]
                            data[viewstart + 3] = data[viewstart + 2]
                            data[viewstart + 2] = data[viewstart + 1]
                            data[viewstart + 1] = data[viewstart + 0]
                            data[viewstart + 0] = 0xff
                            rospy.logwarn("Possible corrupted Haedder, try to interpret with a shift off the haedder")
                        break
                rospy.loginfo("Search packetstart!!, drop byte: %s" % hex(data[viewstart]))
                # We drop the first byte and try to get a new one, as it is possible
                # that the byte is stray byte from a former invalid read/write
                viewstart += 1
                if io_error < 2:
                    # We only try to get more bytes only two times because it would slow down the whole process
                    # if we constantly run into the timeouts if no more data is on the bus. This will most likely
                    # happen if the header is corrupt
                    io_error += 1
                    try:
                        pystr += self.serial.read(1)[0]
                        data = <ubyte*><char*>pystr
                        bytecount += 1
                        rospy.loginfo("additional byte read!")
                    except IOError:
                        # if we reach this point it is very likely that the package header was corrupt as if
                        # it was a stray byte in front of our packet there should be a byte in the que, if it is not
                        # it is highly probable that one of the 0xff bytes in the package were corrupted
                        rospy.logwarn("Could not read a byte from bus after dropping a byte, likely a corrupt packet header")
                        pass
                    if DEBUG:
                        print "Data (header): ", (hex(data[viewstart+0]), hex(data[viewstart+1]), hex(data[viewstart+2]),
                                         hex(data[viewstart+3]), hex(data[viewstart+4]), hex(data[viewstart+5]))
                        rospy.loginfo("Data Langth %d " % bytecount)

            if not(viewstart + 6 <= bytecount):
                # we have reached the end of the data (-6 bytes as the header of one packet is 6 bytes)
                rospy.logwarn("Unexpected end of packet")
                break

            size = 4 + data[viewstart+3]
            if DEBUG and False:
                print [hex(ord(x)) for x in data[viewstart:viewstart+size]]
            #print checksum(data + viewstart, size-1), data[viewstart+size-1]
            #print viewstart, data[viewstart], data[viewstart+1], size
            if checksum(data + viewstart, size) != 0:
                # We calculate the checksum over the data plus the original checksum the result should be zero
                # as the checksum is the complement of the data
                rospy.logwarn("Checksum mismatch for CID %s (propably), this will lead to search of package start!" % hex(data[viewstart +3]))
                print "Data: ", [hex(ord(x)) for x in data[viewstart:viewstart+size]]
                viewstart += 1 if not corrupt_start else 2
                # we increment viewstart twice on corrupt start because of the decrement in case of an missing 0xff
                continue

            packet = make_status_packet(data + viewstart, size, corrupt_start)
            packets.append(packet)
            viewstart += size

        if len(packets) == 0:
            raise IOError("No packets received")

        if len(packets) < count:
            rospy.logwarn("Got only %d of %d packets" % (len(packets) ,count))

        return packets

    cpdef read_register(self, int cid, Register register):
        """
        Shortcut for reading a register

        :param cid: The motor ID
        :type cid: int
        :param register: The register to read
        :type: register: Register
        :return: int or list, depends on Register
        """
        cdef BulkReadPacket p = BulkReadPacket()
        p.add(cid, (register,))
        return self.process(p)[cid][0]

    cpdef write_register(self, int cid, Register register, object value):
        """
        Shortcut for writing a register

        :param cid: The motor ID
        :type id: int
        :param register: The register to write
        :type register: Register
        :param value: The Value to Write. The type depends on the Register to write
        :type value: Object
        """
        cdef SyncWritePacket p = SyncWritePacket((register,))
        p.add(cid, (value,))
        self.process(p)

    cpdef ping(self, int cid):
        """
        Pings the Motor, and return if the Motor answered

        :param cid: The Motor ID
        :type cid: int
        :return: bool
        """
        try:
            return self.process(PingPacket(cid))
        except IOError:
            return False

