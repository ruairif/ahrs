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
from collections import namedtuple
from math import pi
from ctypes import c_uint16 as uint16
import serial
import struct


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


class Nav440(Sensor):

    '''\
    Interface with the XBow Nav440\
    '''

    DataStr = namedtuple('DataStr', 'type num data crc stripped')
    Reading = namedtuple('Reading', 'sensor direction')
    S0 = [Reading(sensor='accel', direction='x'),
          Reading(sensor='accel', direction='y'),
          Reading(sensor='accel', direction='z'),
          Reading(sensor='gyro', direction='x'),
          Reading(sensor='gyro', direction='y'),
          Reading(sensor='gyro', direction='z'),
          Reading(sensor='mag', direction='x'),
          Reading(sensor='mag', direction='y'),
          Reading(sensor='mag', direction='z'),
          Reading(sensor='temp', direction='x'),
          Reading(sensor='temp', direction='y'),
          Reading(sensor='temp', direction='z'),
          Reading(sensor='temp', direction='cpu')]

    A0 = [Reading(sensor='attit', direction='r'),
          Reading(sensor='attit', direction='p'),
          Reading(sensor='attit', direction='y'),
          Reading(sensor='gyro', direction='x'),
          Reading(sensor='gyro', direction='y'),
          Reading(sensor='gyro', direction='z'),
          Reading(sensor='accel', direction='x'),
          Reading(sensor='accel', direction='y'),
          Reading(sensor='accel', direction='z'),
          Reading(sensor='mag', direction='x'),
          Reading(sensor='mag', direction='y'),
          Reading(sensor='mag', direction='z'),
          Reading(sensor='temp', direction='x')]

    scales = {'accel': 20.0 / 2 ** 16,
              'gyro': 7*pi / 2 ** 16,
              'mag': 2.0 / 2 ** 16,
              'attit': 2*pi / 2 ** 16,
              'temp': 200.0 / 2 ** 16}

    packet_types = {'S0': S0, 'A0': A0, 'A1': A0}

    low_high = False

    def __init__(self, bus='/dev/USB0', baudrate=9600, timeout=None, **kwargs):
        self.sensor = serial.Serial(bus,
                                    baudrate=baudrate,
                                    timeout=None)
        self.sensor.close()
        self._init_sensor(**kwargs)

    def _init_sensor(self, **kwargs):
        pass

    def read(self):
        if not self.sensor.isOpen():
            self.sensor.open()
            while self.sensor.read(5) != 'UUS0\x1e':
                self.sensor.flushOutput()
                self.sensor.flushInput()
            self.sensor.read(32)
        return self.parse_packet(self.sensor.read(37))

    def poll(self):
        self.read()

    def calibrate(self):
        pass

    def parse_packet(self, packet):
        '''\
        Convert Packet from binary to data dict\
        '''
        reading = self.split_packet(packet)
        if reading.crc != self.calc_CRC(reading.stripped):
            self.sensor.close()
            raise IOError('Data not transmistted correctly,'
                          'CRC doesn\'t match packet')

        reading_info = self.packet_types[reading.type]
        parsed_packet = {'raw': packet}
        self.raw_data = reading.data
        for pos, info in zip(range(0, reading.num, 2), reading_info):
            data_val = struct.unpack('>h', self.raw_data[pos: pos + 2])[0]
            scaled = data_val * self.scales[info.sensor]
            parsed_packet.setdefault(info.sensor, {})
            parsed_packet[info.sensor][info.direction] = scaled
        return parsed_packet

    def split_packet(self, packet=None):
            if packet is not None:
                self.packet = packet
            return self.DataStr(type=self.packet[2:4],
                                num=struct.unpack('>B', self.packet[3])[0],
                                crc=struct.unpack('>H', self.packet[-2:])[0],
                                data=self.packet[5:-2],
                                stripped=self.packet[2:-2])

    def calc_CRC(self, data_packet):
            crc = 0x1D0F
            for byte_c in data_packet:
                crc ^= uint16(ord(byte_c) << 8).value
                for _ in range(8):
                    if crc & 0x8000:
                        crc = uint16((crc << 1) ^ 0x1021).value
                    else:
                        crc = uint16(crc << 1).value
            return crc
