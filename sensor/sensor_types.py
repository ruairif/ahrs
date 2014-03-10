class SensorTypes(object):

    NONE = 0
    ALL = 1
    ACCELEROMETER = 2
    GYROSCOPE = 3
    MAGNETOMETER = 4

    SENSOR_NAMES = {0: None,
                    1: 'all',
                    2: 'accelerometer',
                    3: 'gyroscope',
                    4: 'magnetometer'}

    @classmethod
    def name(self, value=0):
        return self.SENSOR_NAMES.get(value)
