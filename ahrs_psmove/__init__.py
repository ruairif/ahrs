from sensor import Sensor
from sensor import SensorTypes as ST
import psmove


class PSMove(Sensor):

    '''\
    PlayStation Move sensor with accelerometer, gyroscope, magnetometer,
    light ball and buttons
    '''

    _type = ST.IMU
    a_scale = 441.4  # lsb to m/s^2
    g_scale = 7509.8  # lsb to rads/s

    def __init__(self, **kwargs):
        self.sensor = psmove.psmove_connect()
        _ = ST.name
        self.s_names = {'a': _(ST.ACCELEROMETER),
                        'g': _(ST.GYROSCOPE),
                        'm': _(ST.MAGNETOMETER),
                        'o': _(ST.ORIENTATION),
                        'T': _(ST.TEMPERATURE)}
        self._init_sensor(**kwargs)

    def _init_sensor(self, **kwargs):
        self.sensor.enable_orientation(True)
        self.sensor.reset_orientation()

    def read(self):
        _ = self.sensor
        n = self.s_names
        self.sensor.poll()
        vals = {n['a']: {'x': _.ax, 'y': _.ay, 'z': _.az},
                n['g']: {'x': _.gz, 'y': _.gy, 'z': _.gz},
                n['m']: {'x': _.mx, 'y': _.my, 'z': _.mz},
                n['o']: {'w': 1, 'x': 0, 'y': 0, 'z': 0},
                n['T']: _.get_temperature_in_celsius()}
        # _.reset_orientation()
        return vals

    def poll(self):
        _ = self.read()
        n = self.s_names
        for dir in 'xyz':
            _[n['a']][dir] /= self.a_scale
            _[n['g']][dir] /= self.g_scale

        return _

    def clean_up(self, **kwargs):
        psmove.psmove_disconnect(self.sensor)
