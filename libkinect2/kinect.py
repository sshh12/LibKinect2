from pkg_resources import resource_filename
from ctypes import cdll
import ctypes


kinectDLL = cdll.LoadLibrary(resource_filename(__name__, 'data/Kinect2-API.dll'))


class Kinect2:

    def __init__(self):
        print("Init!")