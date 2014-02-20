#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys


try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

if sys.argv[-1] == 'publish':
    os.system('python setup.py sdist upload')
    sys.exit()

readme = ''
with open('README.rst') as f:
    readme = f.read()

reqs = []
with open('requirements.txt') as f:
    reqs = f.read().splitlines()


setup(
    name='ahrs_sensors',
    version='0.0.3',
    description='Read data from the sparkfun SEN-10724 sensor stick',
    long_description=readme,
    author='Ruairi Fahy',
    url='https://github.com/ruairif/ahrs',
    packages=[
        'ahrs_sensors',
        'sensor'
    ],
    include_package_data=True,
    install_requires=reqs,
    license='MIT',
    zip_safe=False,
    keywords='sensors',
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
    ]
)

