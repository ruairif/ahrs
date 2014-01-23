#!/usr/bin/python
# -*- coding: utf-8 -*-

from time import time, sleep
from ctypes import c_uint16
from sensor import Sensor

'''\
This module provies a generic interface for dealing with several different
sensors. These sensors include

    • Honeywell HMC5883L low-field magnetic sensing
    • Analog Devices ADXL345 accelerometer
    • InvenSense ITG-3200A 3-axis MEMS gyroscope
    • ST Microelectronics L3G4200D 3-axis MEMS motion system
    • Bosch BMP085 Pressure and Temperature sensor
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

    calibration = {'x': (-512, 512), 'y': (-512, 512), 'z': (-512, 512)}
    g_a = 9.801*255
    multiplier = 0.004
    offset = {'x': 0,
              'y': 0,
              'z': 0}
    scale = {'x':g_a/(calibration['x'][0] - offset['x']),
             'y':g_a/(calibration['y'][0] - offset['y']),
             'z':g_a/(calibration['z'][0] - offset['z'])}


    def _init_sensor(self, **kwargs):
        self.sensor.write8(0x2D, 0x08) # Turn Sensor on from standby
        sleep(0.05)
        self.sensor.write8(0x2C, 0xB0) # 200kHz poll rate, High Power mode
        #self.sensor.write8(0x2c, 0xb8) # 200kHz poll rate, Low Power mode
        sleep(0.05)
        self.sensor.write8(0x31, 0x08) # Set sensor to full resolution ± 2g
        sleep(0.05)
        # Clean this up and make it into documentation for the sensor
        #self.sensor.write8(0x31, 0x28) # Set sensor to full resolution ± 4g
        #self.sensor.write8(0x31, 0x48) # Set sensor to full resolution ± 8g
        #self.sensor.write8(0x31, 0x68) # Set sensor to full resolution ±16g


    def read(self):
        return self.read_s16(0x32)


    def poll(self):
        direction_vector = self.read()
        return direction_vector


    def calibrate(self):
        data = self.read()
        g_a = self.g_a
        for direction in ('x', 'y', 'z'):
            calibration = self.calibration[direction]
            data_value = data[direction]
            if data_value > calibration[0]:
                self.calibration[direction] = (data_value, calibration[1])

            if data_value < calibration[1]:
                self.calibration[direction] = (calibration[0], data_value)

        self.offset = {'x': sum(self.calibration['x']) / 2.0,
                       'y': sum(self.calibration['y']) / 2.0,
                       'z': sum(self.calibration['z']) / 2.0}

        self.scale = {'x':g_a/(self.calibration['x'][0] - self.offset['x']),
                      'y':g_a/(self.calibration['y'][0] - self.offset['y']),
                      'z':g_a/(self.calibration['z'][0] - self.offset['z'])}



class ITG3200(Sensor):
    '''\
    InvenSense ITG-3200A 3-axis MEMS gyroscope\
    '''
    _address = 0x68
    data_vector = ('T', 'x', 'y', 'z')
    low_high = False

    def _init_sensor(self, **kwargs):
        # Set sampling rate 1kHz/(byte + 1)
        self.sensor.write8(0x15, 0x07)
        sleep(0.05)
        # Set Range and Internal Sampling Rate
        self.sensor.write8(0x16, (0x03 << 3) | 0x01)
        sleep(0.05)
        # Enable interrupt pin for monitoring when new data is avaialable
        self.sensor.write8(0x17, 0x01)
        sleep(0.05)

    def read(self):
        data_ready = self.sensor.readU8(0x1A) & 0x01
        if not data_ready:
            return None

        return self.read_s16(0x1B, 8)

    def poll(self):
        return self.read()

    def calibrate(self):
        pass




class L3G4200D(Sensor):
    '''\
    ST Microelectronics L3G4200D 3-axis MEMS motion system\
    '''
    _address = 0x69
    data_vector = ('x', 'y', 'z')
    temperature = 0

    def _init_sensor(self, **kwargs):
        # Set output rate, bandwidth and power mode
        self.sensor.write8(0x20, 0x1F)
        sleep(0.05)
        # Set Range and Internal Sampling Rate
        self.sensor.write8(0x21, (0x03 << 3) | 0x01)
        sleep(0.05)
        # Enable interrupt pin for monitoring when new data is avaialable
        self.sensor.write8(0x22, 0x08)
        sleep(0.05)
        # Enable/disable self-test, SPI mode, big/little endian and update mode
        self.sensor.write8(0x23, 0x80 | 0x00 & 0xF7)
        sleep(0.05)
        # fill in documentation from  pg32 of datasheet

    def read(self):
        data_ready = self.sensor.readU8(0x27) & 0x08
        if not data_ready:
            return None

        self.temperature = self.sensor.readU8(0x26)
        return self.read_s16(0x28, 6)

    def poll(self):
        return self.read()

    def calibrate(self):
        pass



class BMP085(Sensor):

    '''\
    Bosch BMP085 Pressure and Temperature sensor\
    '''

    _address = 0x77
    low_high = False
    data_vector = ('TP',)
    resolution = 0x02

    calibration_data = {}
    calibration_constants = ['AC1', 'AC2', 'AC3', 'AC4', 'AC5', 'AC6',
                              'B1',  'B2',  'MB',  'MC', 'MD']


    last_temp = 20.0
    last_time = 0

    def _init_sensor(self, **kwargs):
        # Set Resolution mode of Barometer
        # 0 low power
        # 1 normal
        # 2 fine
        # 3 ultrafine
        self.resolution = kwargs.get('resolution', 0x02)
        self.calibration_data = self._read_calibration_from_EEPROM()

    def _read_calibration_from_EEPROM(self):
        '''\
        Read in values from the sensor needed to convert the 16bit pressure
        value read from the device into Pascals
        '''
        self.data_vector = self.calibration_constants
        calibration_data = self.read_s16(0xAA, 22)
        for reg in ['AC4', 'AC5', 'AC6']:
            calibration_data[reg] = c_uint16(calibration_data[reg]).value
        self.data_vector = ('TP',)
        return calibration_data

    def _read_temp(self):
        '''\
        Read temperature value as 16bit int from the sensor\
        '''
        self.sensor.write8(0xF4, 0x2F)
        sleep(0.005)
        self.last_time = time()
        temp = self.read_s16(0xF6, 2)
        self.last_temp = temp['TP']
        return temp['TP']

    def _read_pressure(self):
        '''\
        Read pressure value from the sensor as 16bit int\
        '''
        self.sensor.write8(0xF4, 0x34 + (self.resolution << 6))
        sleep(0.014)
        return self.read_s16(0xF6, 2)['TP']

    @property
    def temp(self):
        '''\
        Return temperature value in °C\
        '''
        if time() - self.last_time > 1:
            self._read_temp()
        c = self.calibration_data
        UT = self.last_temp
        X1 = ((UT - c['AC6']) * c['AC5']) >> 15
        X2 = (c['MC'] << 11) / (X1 + c['MD'])
        B5 = X1 + X2
        T = ((B5 + 8) >> 4) / 10.0
        return 278 + T

    @property
    def pressure(self):
        '''\
        Return Pressure value in Pascals\
        '''
        self.temp()
        c = self.calibration_data
        UT = self.last_temp
        UP = self._read_pressure()
        X1 = ((UT - c['AC6']) * c['AC5']) >> 15
        X2 = (c['MC'] << 11) / (X1 + c['MD'])
        B5 = X1 + X2
        B6 = B5 - 4000
        X1 = (c['B2'] * (B6 * B6) >> 12) >> 11
        X2 = (c['AC2'] * B6) >> 11
        X3 = X1 + X2
        B3 = (((c['AC1'] * 4 + X3) << self.resolution) + 2) / 4
        X1 = (c['AC3'] * B6) >> 13
        X2 = (c['B1'] * ((B6 * B6) >> 12)) >> 16
        X3 = ((X1 + X2) + 2) >> 2
        B4 = (c['AC4'] * (X3 + 32768)) >> 15
        B7 = (UP - B3) * (50000 >> self.resolution)
        if (B7 < 0x80000000):
            p = (B7 * 2) / B4
        else:
            p = (B7 / B4) * 2

        X1 = (p >> 8) * (p >> 8)
        X1 = (X1 * 3038) >> 16
        X2 = (-7357 * p) >> 16
        p = p + ((X1 + X2 + 3791) >> 4)
        return 100000 + p/10

    def read(self):
        return {'P': self._read_pressure(), 'T': self.last_temp}

    def poll(self):
        return {'P': int(self.pressure), 'T': round(self.temp, 2)}

    def calibrate(self):
        pass
