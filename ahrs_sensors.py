#!/usr/bin/python
# -*- coding: utf-8 -*-

'''\
This module provies a generic interface for dealing with several different
sensors. These sensors include

    • Honeywell HMC5883L low-field magnetic sensing
    • Analog Devices ADXL345 accelerometer
    • InvenSense ITG-3200A 3-axis MEMS gyroscope
'''


from time import sleep
from sensor import Sensor


class HMC5883L(Sensor):

    '''\
    Honeywell HMC5883L low-field magnetic sensing\
    '''
    _address = 0x1e
    data_vector = ('x', 'z', 'y')
    low_high = False

    scale = 1090.0  # lsb per gauss

    previous = {'x': 0, 'y': 0, 'z': 0}

    def _init_sensor(self, **kwargs):
        # TODO allow non hardcoded configuration
        # Set sensor to single measurement mode, up to 160Hz
        self.sensor.write8(0x02, 0x00)
        sleep(0.05)

        self.previous = self.read_s16(0x03)
        sleep(0.05)

    def read(self):
        data_ready = self.sensor.readU8(0x09) & 0x01
        if not data_ready:
            return self.previous

        return self.read_s16(0x03)

    def poll(self):
        direction_vector = self.read()
        # TODO Need to deal with temperature compensation
        # Return data x,y,z values in Teslas, (1T = 1e-4 Gauss)
        return dict([(d, (direction_vector[d] / self.scale) * 1e-4)
                     for d in 'xzy'])

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
        self.sensor.write8(0x2C, 0x0B)
        sleep(0.05)

        # Set sensor to full resolution ± 2g
        self.sensor.write8(0x31, 0x08)
        sleep(0.05)

    def read(self):
        return self.read_s16(0x32)

    def poll(self):
        direction_vector = self.read()
        return dict([(d, (direction_vector[d] * self.scale)) for d in 'xyz'])

    def calibrate(self):
        pass


class ITG3200(Sensor):

    '''\
    InvenSense ITG-3200A 3-axis MEMS gyroscope\
    '''
    _address = 0x68
    data_vector = ('T', 'x', 'y', 'z')
    low_high = False
    scale = 0.06957  # rads per lsb

    previous = {'x': 0, 'y': 0, 'z': 0, 'T': 0}

    def _init_sensor(self, **kwargs):
        # Set sampling rate 1kHz/(byte + 1)
        self.sensor.write8(0x15, 0x07)
        sleep(0.06)

        # Set Range and Internal Sampling Rate
        self.sensor.write8(0x16, (0x03 << 3) | 0x01)
        sleep(0.06)

        # Enable interrupt pin for monitoring when new data is avaialable
        self.sensor.write8(0x17, 0x01)
        sleep(0.06)

        self.previous = self.read_s16(0x1B, 8)
        sleep(0.06)

    def read(self):
        data_ready = self.sensor.readU8(0x1A) & 0x01
        if not data_ready:
            return self.previous

        return self.read_s16(0x1B, 8)

    def poll(self):
        direction_vector = self.read()
        temperature = direction_vector['T']
        direction_vector['T'] = round(35 + ((temperature + 13200) / 280.0), 2)
        for dir in 'xyz':
            direction_vector[dir] = direction_vector[dir] * self.scale
        return direction_vector

    def calibrate(self):
        pass
