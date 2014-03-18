#!/usr/bin/python
# -*- coding: utf-8 -*-

'''\
This module provies a generic interface for dealing with several different
sensors. These sensors include

    • Honeywell HMC5883L low-field magnetic sensing
    • Analog Devices ADXL345 accelerometer
    • InvenSense ITG-3200A 3-axis MEMS gyroscope
    • ST Microelectronics L3G4200D MEMS gyroscope
    • XBow Nav440 attitude sensor
'''


from time import sleep
from sensor import Sensor
from sensor import SensorTypes as ST
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
    _type = ST.MAGNETOMETER
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
    _type = ST.ACCELEROMETER
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

        base_data = self.read()
        self.offset['x'] = base_data['x']
        self.offset['y'] = base_data['y']
        z = base_data['z']
        self.offset['z'] = (z / abs(z)) * (251 - abs(z))

    def read(self):
        return self.read_s16(0x32)

    def poll(self):
        direction_vector = self.read()
        _ = dict([(d, (direction_vector[d] * self.scale)) for d in 'xyz'])
        return _

    def calibrate(self):
        pass


class ITG3200(Sensor):

    '''\
    InvenSense ITG-3200A 3-axis MEMS gyroscope\
    '''
    _address = 0x68
    _type = ST.GYROSCOPE
    data_vector = ('T', 'x', 'y', 'z')
    low_high = False
    scale = 14.375  # degrees per lsb
    deg_to_rad = 0.0174532925

    previous = {'x': 0, 'y': 0, 'z': 0, 'T': 0}

    def _init_sensor(self, **kwargs):
        # Set sampling rate 1kHz/(byte + 1)
        self.sensor.write8(0x15, 0x09)
        sleep(0.06)

        # Set power mode and clock
        self.sensor.write8(0x3E, 0x07 & 0x01)
        sleep(0.06)

        # Set Range and Internal Sampling Rate
        self.sensor.write8(0x16, 0x03 << 3 | 0x06)
        sleep(0.06)

        # Enable interrupt pin for monitoring when new data is avaialable
        self.sensor.write8(0x17, 0x01)
        sleep(0.06)

        self.previous = self.read_s16(0x1B, 8)
        sleep(0.06)

        base_data = self.read()
        self.offset['x'] = base_data['x']
        self.offset['y'] = base_data['y']
        self.offset['z'] = base_data['z']

    def read(self):
        data_ready = self.sensor.readU8(0x1A) & 0x01
        if not data_ready:
            return self.previous

        return self.read_s16(0x1B, 8)

    def poll(self):
        direction_vector = self.read().copy()
        temperature = direction_vector['T']
        direction_vector['T'] = round(35 + ((temperature + 13200) / 280.0), 2)
        scaled = {}
        for direction in 'xyz':
            _ = direction_vector[direction] / self.scale
            scaled[direction] = _ * self.deg_to_rad
        return scaled

    def calibrate(self):
        pass


class L3G4200D(Sensor):
    '''\
    ST Microelectronics L3G4200D 3-axis MEMS motion system\
    '''
    _address = 0x69
    _type = ST.GYROSCOPE
    data_vector = ('x', 'y', 'z')
    temperature = 0
    deg_to_rad = 0.0174532925
    scale = 70e-3  # degrees per lsb

    def _init_sensor(self, **kwargs):
        # Set output rate, bandwidth and power mode
        self.sensor.write8(0x20, 0x1F)
        sleep(0.05)
        # Set Range and Internal Sampling Rate
        self.sensor.write8(0x21, 0x20)
        sleep(0.05)
        # Enable interrupt pin for monitoring when new data is avaialable
        self.sensor.write8(0x22, 0x08)
        sleep(0.05)
        # Enable/disable self-test, SPI mode, big/little endian and update mode
        self.sensor.write8(0x23, 0x30)
        sleep(0.05)
        # Enable low and High pass filters
        self.sensor.write8(0x24, 0x12)
        sleep(0.05)

        base_data = self.read()
        self.offset['x'] = base_data['x']
        self.offset['y'] = base_data['y']
        self.offset['z'] = base_data['z']

    def read(self):
        data_ready = self.sensor.readU8(0x27) & 0x08
        if data_ready is 8:
            return None

        _ = self.read_s16(0x28, 6)
        self.temperature = _['T']
        return _

    def poll(self):
        direction_vector = self.read().copy()
        temperature = direction_vector['T']
        direction_vector['T'] = round(35 + ((temperature + 13200) / 280.0), 2)
        scaled = {}
        for direction in 'xyz':
            _ = direction_vector[direction] / self.scale
            scaled[direction] = _ * self.deg_to_rad
        return scaled

    def calibrate(self):
        pass


class Nav440(Sensor):

    '''\
    Interface with the XBow Nav440\
    '''

    _type = ST.AHRS
    DataStr = namedtuple('DataStr', 'type num data crc stripped')
    Reading = namedtuple('Reading', 'sensor direction')
    S0 = [Reading(sensor=ST.name(ST.ACCELEROMETER), direction='x'),
          Reading(sensor=ST.name(ST.ACCELEROMETER), direction='y'),
          Reading(sensor=ST.name(ST.ACCELEROMETER), direction='z'),
          Reading(sensor=ST.name(ST.GYROSCOPE), direction='x'),
          Reading(sensor=ST.name(ST.GYROSCOPE), direction='y'),
          Reading(sensor=ST.name(ST.GYROSCOPE), direction='z'),
          Reading(sensor=ST.name(ST.MAGNETOMETER), direction='x'),
          Reading(sensor=ST.name(ST.MAGNETOMETER), direction='y'),
          Reading(sensor=ST.name(ST.MAGNETOMETER), direction='z'),
          Reading(sensor=ST.name(ST.TEMPERATURE), direction='x'),
          Reading(sensor=ST.name(ST.TEMPERATURE), direction='y'),
          Reading(sensor=ST.name(ST.TEMPERATURE), direction='z'),
          Reading(sensor=ST.name(ST.TEMPERATURE), direction='cpu')]

    A0 = [Reading(sensor=ST.name(ST.ATTITUDE), direction='r'),
          Reading(sensor=ST.name(ST.ATTITUDE), direction='p'),
          Reading(sensor=ST.name(ST.ATTITUDE), direction='y'),
          Reading(sensor=ST.name(ST.GYROSCOPE), direction='x'),
          Reading(sensor=ST.name(ST.GYROSCOPE), direction='y'),
          Reading(sensor=ST.name(ST.GYROSCOPE), direction='z'),
          Reading(sensor=ST.name(ST.ACCELEROMETER), direction='x'),
          Reading(sensor=ST.name(ST.ACCELEROMETER), direction='y'),
          Reading(sensor=ST.name(ST.ACCELEROMETER), direction='z'),
          Reading(sensor=ST.name(ST.MAGNETOMETER), direction='x'),
          Reading(sensor=ST.name(ST.MAGNETOMETER), direction='y'),
          Reading(sensor=ST.name(ST.MAGNETOMETER), direction='z'),
          Reading(sensor=ST.name(ST.TEMPERATURE), direction='x')]

    packet_types = {'S0': S0, 'A0': A0, 'A1': A0}
    previous_packet = {}
    low_high = False

    def __init__(self, bus='/dev/USB0', baudrate=9600, timeout=None, **kwargs):
        self.sensor = serial.Serial(bus,
                                    baudrate=baudrate,
                                    timeout=None)
        self.sensor.close()

        _ = ST.name
        self.scales = {_(ST.ACCELEROMETER): 20.0 / 2 ** 16,
                       _(ST.GYROSCOPE): 7 * pi / 2 ** 16,
                       _(ST.MAGNETOMETER): 2.0 / 2 ** 16,
                       _(ST.ATTITUDE): 2 * pi / 2 ** 16,
                       _(ST.TEMPERATURE): 200.0 / 2 ** 16}
        self._init_sensor(**kwargs)

    def _init_sensor(self, **kwargs):
        pass

    def read(self):
        if not self.sensor.isOpen():
            self.sensor.open()
            self.sensor.flushOutput()
            self.sensor.flushInput()
            while True:  # self.sensor.read(5) != 'UUS0\x1e':
                char = self.sensor.read(1)
                if char == 'U':
                    header = self.sensor.read(4)
                    if header == 'US0\x1e':
                        break
            self.sensor.read(32)
        try:
            packet_data = self.parse_packet(self.sensor.read(37))
            self.previous_packet = packet_data
        except IOError as err:
            print('CRC value not correct. Data not transmitted correctly')
            packet_data = self.previous_packet

        finally:
            self.sensor.close()
        return packet_data

    def poll(self):
        return self.read()

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
        parsed_packet = {}
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
