#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import absolute_import

from .sensor_types import SensorTypes
from ctypes import c_int16
import smbus


class Sensor(object):

    '''\
    Base Class for defining interactions with sensors over i2c\
    '''
    # Address of sensor over I2C interface
    _address = 0x00
    # Type of sensor being read. Types of sensors found in SensorType class
    _type = SensorTypes.NONE
    # The order which the data_vector should be read from the device
    data_vector = ('x', 'y', 'z')
    # Basic initialised 6byte buffer for reading from sensors
    raw_data = [0, 0, 0, 0, 0, 0]
    low_high = True

    # Adjust Sensors for initialisation offsets
    offset = {'x': 0, 'y': 0, 'z': 0, 'T': 0}

    def __init__(self, bus=None, **kwargs):
        '''\
        Set up I2C connection to sensor and initialise parameters specific to
        the sensor\
        '''
        self.sensor = self.SMBusCommunicate(self.address, bus)
        self._init_sensor(**kwargs)

    def _init_sensor(self, **kwargs):
        '''\
        Send necessary bytes to sensor to put it into a particular mode\
        '''
        raise NotImplementedError

    def read(self):
        '''\
        Read data directly from the sensors and return sensor reading\
        '''
        raise NotImplementedError

    def poll(self):
        '''\
        Read data from sensor and return corrected SI unit values\
        '''
        return self.read()

    def clean_up(self, **kwargs):
        '''\
        Allow for clean up logic to be implemented if needed\
        '''
        pass

    @property
    def type(self):
        '''\
        Enumerated type of sensor. The name of the sensor type can be
        received from the SensorTypes class\
        '''
        return self._type

    @property
    def type_name(self):
        '''\
        Return a string name for the sensor type\
        '''
        return SensorTypes.name(self.type)

    def calibrate(self, **kwargs):
        '''\
        Provide a setup that can allow for accurate calibration of the
        sensor\
        '''
        raise NotImplementedError

    def convert_raw_data(self, index):
        '''\
        Combine 2 unsigned bytes into a single 16bit unsigned int\
        '''
        if self.low_high:
            unsigned_int = ((self.raw_data[index + 1] << 8) |
                            self.raw_data[index])
        else:
            unsigned_int = ((self.raw_data[index] << 8) |
                            self.raw_data[index + 1])
        return c_int16(unsigned_int).value

    def read_s16(self,
                 start=None,
                 num_bytes=6,
                 register_names=None):
        '''\
        Read specified bytes from sensor and return 16bit values with
        appropriate names\
        '''
        if start is None or start not in range(0x00, 0xFF):
            raise ValueError("Need valid register number")

        if register_names is None:
            register_names = self.data_vector
        self.raw_data = self.sensor.readList(start, num_bytes)
        direction_vector = []
        for d, i in zip(register_names, range(0, num_bytes, 2)):
            fixed_point = self.convert_raw_data(i) - self.offset[d]
            direction_vector.append((d, fixed_point))

        return dict(direction_vector)

    @property
    def address(self):
        '''\
        I2C address of sensor\
        '''
        return self._address

    class SMBusCommunicate(object):

        '''\
        Class to read and write bytes to I2C devices over SMBus\
        '''

        def __init__(self, address=0x00, bus=None):
            self.address = address
            self.bus = smbus.SMBus(bus if bus is not None else 1)

        def readU8(self, register):
            '''\
            Read a single byte from the given register\
            '''
            try:
                return self.bus.read_byte_data(self.address,
                                               register)
            except IOError:
                raise IOError('Error reading from register {} on '
                              'device {}'.format(hex(self.address),
                                                 hex(register)))

        def readList(self, register, length):
            '''\
            Read a sequential list of bytes, number of bytes determined by
            length, starting register defined by given register.\
            '''
            try:
                return self.bus.read_i2c_block_data(self.address,
                                                    register,
                                                    length)
            except IOError:
                addr = self.address
                raise IOError('Error reading {} bytes from device {} '
                              'starting at register {}'.format(length,
                                                               hex(addr),
                                                               hex(register)))

        def write8(self, register, byte):
            '''\
            Write the given byte to the given register\
            '''
            try:
                return self.bus.write_byte_data(self.address,
                                                register,
                                                byte)
            except IOError:
                raise IOError('Error writing {} to register {} on '
                              'device {}'.format(bin(byte)[2:].zfill(8),
                                                 hex(register),
                                                 hex(self.address)))
