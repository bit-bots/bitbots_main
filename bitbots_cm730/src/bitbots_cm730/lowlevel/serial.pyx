#-*- coding:utf-8 -*-
"""
Serial
^^^^^^

History:

* 19.1.14: Bei Fehlern und dealloc lock aufserial freigen

Dieses Modul stellt die Kommunikation mit einem Seriellen Interface bereit
"""

from bitbots_common.util import pid_exists, own_pid
import os


cdef class AbsSerial:
    """
    Abstraktes Interface für die Serielle Kommunikation
    """
    cpdef read(self, int amount):
        """
        Ließt Daten aus dem Seriellen Device.

        Diese Methode ist Abstrakt und muss überladen werden
        :param amount: Anzahl zu lesender Byts
        :type amount: int
        """
        raise NotImplementedError()

    cpdef write(self, bytes data):
        """
        Schreibt daten in das Seriele Device

        Diese Methode ist Abstrakt und muss überladen werden
        :param data: Die zu schreibenen Daten
        :type data: bytes
        """
        raise NotImplementedError()

    cpdef set_speed(self, double speed):
        """
        Setzt die Boudrate auf dem Seriellen Device

        Diese Methode ist Abstrakt und muss überladen werden
        :param speed: Die einzustellende Geschwindigkeit
        :type speed: double
        """
        raise NotImplementedError()


cdef class Serial:
    """ Einfache Klasse zum Herstellen und Nutzen einer seriellen Verbindung
        unter Linux
    """
    def __cinit__(self, device):
        """ Öffnet die Verbindung zu einer seriellen Schnittstelle """
        try:
            fp = open("/tmp/cm370lock","r")
            pid = fp.read()
            fp.close()
            if pid and pid_exists(int(pid)):
                raise RuntimeError("It seems that an other Programm " + \
                    " use the CM-370. Please terminate it!" )
        except IOError:
            # Datei gibt es nicht...
            pass
        fp2 = open("/tmp/cm370lock","w")
        fp2.write(str(os.getpid()))
        fp2.close()
        try:
            self.serial = new _Serial(device.encode('utf8'))
        except:
            # wir greifen nicht mehr zu, lock aufgeben
            fp = open("/tmp/cm370lock","w")
            fp.write("-1")
            fp.close()
            raise

    def __dealloc__(self):
        ''' Beim Löschen noch aufräumen'''
        del self.serial
        try:
            fp = open("/tmp/cm370lock","r")
            pid = fp.read()
            fp.close()
            if pid and own_pid(int(pid)):
                fp = open("/tmp/cm370lock","w")
                fp.write("-1")
                fp.close()
        except IOError:
            pass

    cdef write_ptr(self, uint8_t* data, size_t length):
        """ Schreibt Daten """
        self.serial.write(data, length)

    cdef read_ptr(self, uint8_t* data, size_t length):
        """ Ließt Daten von der seriellen Schnittstelle """
        return self.serial.read(data, length)

    cpdef read(self, int amount):
        """ Ließt bytes aus der seriellen Schnittstelle """
        cdef bytes data = b"\x00" * amount
        cdef int byte_cont = self.read_ptr(<uint8_t*><char*>data, amount)
        return data, byte_cont

    cpdef write(self, bytes data):
        """ Schreibt einen String """
        self.write_ptr(<uint8_t*><char*>data, len(data))

    cpdef set_speed(self, double speed):
        """ Setzt die Baudrate """
        self.serial.set_speed(speed)

