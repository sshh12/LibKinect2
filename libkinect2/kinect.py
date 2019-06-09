from pkg_resources import resource_filename
import numpy as np
import ctypes


F_SENSOR_COLOR  = 0x00000001
F_SENSOR_DEPTH  = 0x00000010
F_SENSOR_IR     = 0x00000100
F_SENSOR_AUDIO  = 0x00001000
COLOR_WIDTH     = 1920
COLOR_HEIGHT    = 1080
COLOR_CHANNELS  = 4
DEPTH_WIDTH     = 512
DEPTH_HEIGHT    = 424
IR_WIDTH        = 512
IR_HEIGHT       = 424
MAX_SUBFRAMES   = 8
SUBFRAME_SIZE   = 256


kinectDLL = ctypes.cdll.LoadLibrary(resource_filename(__name__, 'data/Kinect2-API.dll'))

kinectDLL.init_kinect.argtypes = [ctypes.c_int]
kinectDLL.init_kinect.restype = ctypes.c_bool

kinectDLL.close_kinect.argtypes = []
kinectDLL.close_kinect.restype = None

kinectDLL.get_color_data.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint8), ctypes.c_bool]
kinectDLL.get_color_data.restype = ctypes.c_bool

kinectDLL.get_ir_data.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint16), ctypes.c_bool]
kinectDLL.get_ir_data.restype = ctypes.c_bool

kinectDLL.get_depth_data.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint16), ctypes.c_bool]
kinectDLL.get_depth_data.restype = ctypes.c_bool

kinectDLL.get_audio_data.argtypes = [np.ctypeslib.ndpointer(dtype=np.float32), ctypes.c_bool]
kinectDLL.get_audio_data.restype = ctypes.c_bool


class Kinect2:

    def __init__(self, disable_sensors=[]):
        self.sensor_flags = 0
        if 'color' not in disable_sensors:
            self.sensor_flags |= F_SENSOR_COLOR
        if 'depth' not in disable_sensors:
            self.sensor_flags |= F_SENSOR_DEPTH
        if 'ir' not in disable_sensors:
            self.sensor_flags |= F_SENSOR_IR
        if 'audio' not in disable_sensors:
            self.sensor_flags |= F_SENSOR_AUDIO

    def _tmp(self):
        kinectDLL.read_audio_sensors()

    def connect(self):
        if not kinectDLL.init_kinect(self.sensor_flags):
            raise IOError('Unable to init Kinect2 Sensor.')
        return True

    def disconnect(self):
        kinectDLL.close_kinect()

    def get_color_image(self):
        color_ary = np.zeros((COLOR_HEIGHT, COLOR_WIDTH, COLOR_CHANNELS), np.uint8)
        if kinectDLL.get_color_data(color_ary, True):
            return color_ary
        return None

    def get_ir_image(self):
        ir_ary = np.zeros((IR_HEIGHT, IR_WIDTH, 1), np.uint16)
        if kinectDLL.get_ir_data(ir_ary, True):
            return ir_ary
        return None

    def get_depth_map(self):
        depth_ary = np.zeros((DEPTH_HEIGHT, DEPTH_WIDTH, 1), np.uint16)
        if kinectDLL.get_depth_data(depth_ary, True):
            return depth_ary
        return None

    def _process_audio(self, audio_ary):
        n = int(audio_ary[0])
        frames = []
        for i in range(n):
            subframe_data = audio_ary[i+1:i+SUBFRAME_SIZE+3]
            beam_angle = subframe_data[0]
            beam_conf = subframe_data[1]
            subframe = subframe_data[2:]
            frames.append((beam_angle, beam_conf, subframe))
        return frames

    def get_audio_frames(self):
        audio_ary = np.zeros((1 + MAX_SUBFRAMES * (SUBFRAME_SIZE + 2),), np.float32)
        if kinectDLL.get_audio_data(audio_ary, True):
            return self._process_audio(audio_ary)
        return []