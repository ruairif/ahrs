#!/usr/bin/python
# -*- coding: utf-8 -*-

from time import time, sleep
from ctypes import c_uint16
from sensor import Sensor

'''\
This module provies a generic interface for dealing with several different
sensors. These sensors include

    • Honeywell HMC5883L low-field magnetic sensing
'''

class HMC5883L(Sensor):
    '''\
    Honeywell HMC5883L low-field magnetic sensing\
    '''
    _address = 0x1e
    data_vector = ('x', 'z', 'y')
    low_high = False

    scale = 1090.0 #lsb per gauss

    def _init_sensor(self, **kwargs):
        # TODO allow non hardcoded configuration
        # Set sensor to single measurement mode, up to 160Hz
        self.sensor.write8(0x02, 0x00)
        sleep(0.05)


    def read(self):
        data_ready = self.sensor.readU8(0x09) & 0x01
        if not data_ready:
            return None

        return self.read_s16(0x03)


    def poll(self):
        direction_vector = self.read()
        # TODO Need to deal with temperature compensation
        # Return data x,y,z values in Teslas, (1T = 1e-4 Gauss)
        return [(direction_vector[d] / self.scale) * 1e-4 for d in 'xyz']


    def calibrate(self):
        pass


class ADXL345(Sensor):
    '''\
    Analog Devices ADXL345 accelerometer\
    '''
    _address = 0x53
    data_vector = ('x', 'y', 'z')
    low_high = True

    scale = 0.04


    def _init_sensor(self, **kwargs):
        # Turn Sensor on from standby
        self.sensor.write8(0x2D, 0x08)
        sleep(0.05)

        # 200kHz poll rate, High Power mode
        self.sensor.write8(0x2C, 0xB0)
        sleep(0.05)

        # Set sensor to full resolution ± 2g
        self.sensor.write8(0x31, 0x08)
        sleep(0.05)


    def read(self):
        return self.read_s16(0x32)


    def poll(self):
        direction_vector = self.read()
        return [(direction_vector[d] * self.scale) for d in 'xyz']


    def calibrate(self):
        pass
