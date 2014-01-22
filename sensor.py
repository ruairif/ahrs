#!/usr/bin/python
# -*- coding: utf-8 -*-

from Adafruit_I2C import Adafruit_I2C
from time import sleep
from ctypes import c_int16


class Sensor(object):

    '''\
    Base Class for defining interactions with sensors over i2c\
    '''
    # Address of sensor over I2C interface
    _address = 0x00
    # The order which the data_vector should be read from the device
    data_vector = ('x', 'y', 'z')
    # Basic initialised 6byte buffer for reading from sensors
    raw_data = [0, 0, 0, 0, 0, 0]
    low_high = True


    def __init__(self, bus=None, **kwargs):
        '''\
        Set up I2C connection to sensor and initialise parameters specific to
        the sensor\
        '''
        bus = bus if bus is not None else 1
        self.sensor = Adafruit_I2C(self.address, busnum=bus)
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
                 read_time=0.008,
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
        sleep(read_time)
        direction_vector = []
        for d, i in zip(register_names, range(0, num_bytes, 2)):
            direction_vector.append((d, self.convert_raw_data(i)))

        return dict(direction_vector)


    @property
    def address(self):
        '''\
        I2C address of sensor\
        '''
        return self._address
