Attitude and Heading Reference System
=====================================

Attitude and Heading Reference System interfacing with accelerometers,
gyroscopes and magnetometers over an I2C bus.

The Honeywell HMC5883L is supported and can be used to detect the direction of
the Earth's magnetic field relative to the device.

The Analog Devices ADXL345 accelerometer can be used to measure acceleration
along the x, y and z axes relative to gravity.

The InvenSense IT-3200 gyroscope can be used to measure angular rotation rate.

Installing
----------
To use this library the host computer needs to have support for communicating
with I2C devices. The other requirement is the python-smbus library must be
installed.

On Ubuntu/Debian::

    apt-get install i2c-tools

On Arch::

    pacman -S i2c-tools
