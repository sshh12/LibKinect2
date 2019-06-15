#!/usr/bin/env python

import os

try:
    from setuptools import setup, find_packages
except:
    raise Exception('setuptools is required for installation')


def join(*paths):
    return os.path.normpath(os.path.join(*paths))


VERSION_PATH = join(__file__, '..', 'libkinect2', 'version.py')


def get_version():

    with open(VERSION_PATH, 'r') as version:
        out = {}
        exec(version.read(), out)
        return out['__version__']


setup(
    name='libkinect2',
    version=get_version(),
    author='Shrivu Shankar',
    url='https://github.com/sshh12/LibKinect2',
    packages=find_packages(),
    package_data={
        'libkinect2': [
            'data/Kinect2-API.dll',
            'data/shape_predictor_68_face_landmarks.dat'
        ]
    },
    license='MIT'
)
