class SensorTypes(object):

    '''\
    Class to keep a consistent naming scheme for sensor names.
    '''

    NONE = 0
    ALL = 1
    ACCELEROMETER = 2
    GYROSCOPE = 3
    MAGNETOMETER = 4
    ATTITUDE = 5
    ORIENTATION = 6
    TEMPERATURE = 7
    IMU = 8
    AHRS = 9

    SENSOR_NAMES = {0: None,
                    1: 'all',
                    2: 'accelerometer',
                    3: 'gyroscope',
                    4: 'magnetometer',
                    5: 'attitude',
                    6: 'orientation',
                    7: 'temperature',
                    8: 'imu',
                    9: 'ahrs'}

    @classmethod
    def name(self, value=0):
        '''\
        Return a string name for a sensor enum value\
        '''
        return self.SENSOR_NAMES.get(value)
