"""
The Kinect2 class
"""
from .dll_lib import *
from .body import Body, Joint
from .audio import AudioFrame
import numpy as np
import time
import cv2


class Kinect2:
    """
    The main Kinect2 class for interacting with the sensor.
    """
    def __init__(self, use_sensors=['color'], use_mappings=[]):
        """
        Create a Kinect obj to use the given sensors.

        Args:
            use_sensors: [color, depth, ir, body, audio]
            use_mappings: [
                (color, camera), (depth, camera),
                (depth, color), (color, depth)
            ]

        Note:
            * At least one sensor must be provided.
            * Mappings are (from_type, to_type).
        """
        self._kinect = init_lib()
        self.sensor_flags = 0
        self.mapping_flags = 0
        if 'color' in use_sensors:
            self.sensor_flags |= F_SENSOR_COLOR
        if 'depth' in use_sensors:
            self.sensor_flags |= F_SENSOR_DEPTH
        if 'ir' in use_sensors:
            self.sensor_flags |= F_SENSOR_IR
        if 'body' in use_sensors:
            self.sensor_flags |= F_SENSOR_BODY
        if 'audio' in use_sensors:
            self.sensor_flags |= F_SENSOR_AUDIO
        if ('color', 'camera') in use_mappings:
            self.mapping_flags |= F_MAP_COLOR_CAM
        if ('depth', 'camera') in use_mappings:
            self.mapping_flags |= F_MAP_DEPTH_CAM
        if ('depth', 'color') in use_mappings:
            self.mapping_flags |= F_MAP_DEPTH_COLOR
        if ('color', 'depth') in use_mappings:
            self.mapping_flags |= F_MAP_COLOR_DEPTH
        if self.sensor_flags == 0:
            raise ValueError('At least one sensor must be provided.')

    def connect(self):
        """
        Connect to the device.
        """
        if not self._kinect.init_kinect(self.sensor_flags, self.mapping_flags):
            raise IOError('Unable to init Kinect2 Sensor.')
        return True

    def disconnect(self):
        """
        Disconnect from the device.
        """
        self._kinect.close_kinect()

    def get_color_image(self, color_format='bgr'):
        """
        Get the current color image.

        Args:
            color_format: rgba, bgr, or rgb

        Returns:
            numpy array
        """
        color_ary = np.empty((COLOR_HEIGHT, COLOR_WIDTH, COLOR_CHANNELS), np.uint8)
        if self._kinect.get_color_data(color_ary):
            if color_format == 'rgba':
                return color_ary
            elif color_format == 'bgr':
                return cv2.cvtColor(color_ary, cv2.COLOR_RGBA2BGR)
            elif color_format == 'rgb':
                return cv2.cvtColor(color_ary, cv2.COLOR_RGBA2RGB)
            else:
                raise NotImplementedError()
        return None

    def get_ir_image(self):
        """
        Get the current inferred image.

        Returns:
            numpy array
        """
        ir_ary = np.empty((IR_HEIGHT, IR_WIDTH, 1), np.uint16)
        if self._kinect.get_ir_data(ir_ary):
            return ir_ary
        return None

    def get_depth_map(self):
        """
        Get the current depth map.

        Returns:
            numpy array
        """
        depth_ary = np.empty((DEPTH_HEIGHT, DEPTH_WIDTH, 1), np.uint16)
        if self._kinect.get_depth_data(depth_ary):
            return depth_ary
        return None

    def _get_raw_bodies(self):
        body_ary = np.empty((MAX_BODIES, BODY_PROPS), np.uint8)
        joint_ary = np.empty((MAX_BODIES, MAX_JOINTS, JOINT_PROPS), np.int32)
        if self._kinect.get_body_data(body_ary, joint_ary):
            return body_ary, joint_ary
        return None, None

    def get_bodies(self):
        """
        Get the currently tracked bodies.

        Returns:
            `Body` array
        """
        body_ary, joint_ary = self._get_raw_bodies()
        bodies = []
        if body_ary is not None:
            for i in range(MAX_BODIES):
                if body_ary[i, 0]:
                    bodies.append(Body(i, body_ary[i], joint_ary[i]))
        return bodies

    def _get_raw_audio(self):
        audio_ary = np.empty((AUDIO_BUF_LEN * SUBFRAME_SIZE), np.float32)
        meta_ary = np.empty((AUDIO_BUF_LEN, 2), np.float32)
        frame_cnt = self._kinect.get_audio_data(audio_ary, meta_ary)
        return frame_cnt, audio_ary, meta_ary

    def get_audio_frames(self):
        """
        Get the latest audio frames.

        Returns:
            array of `AudioFrame`
        """
        frame_cnt, audio_ary, meta_ary = self._get_raw_audio()
        frames = []
        for i in range(frame_cnt):
            beam_angle = meta_ary[i, 0]
            beam_conf = meta_ary[i, 1]
            samples = audio_ary[i*SUBFRAME_SIZE:(i+1)*SUBFRAME_SIZE]
            frames.append(AudioFrame(beam_angle, beam_conf, samples))
        return frames

    def map(self, from_type, to_type):
        """
        Get a mapping between visual sensors.

        Args:
            from_type: The sensor space to convert (color, depth)
            to_type: The target sensor space (color, depth, camera)

        Returns:
            numpy array of mapping
        """
        result = None
        if from_type == 'color' and to_type == 'camera':
            map_ary = np.empty((COLOR_HEIGHT, COLOR_WIDTH, 3), np.float32)
            if self._kinect.get_map_color_to_camera(map_ary):
                result = map_ary
        elif from_type == 'depth' and to_type == 'camera':
            map_ary = np.empty((DEPTH_HEIGHT, DEPTH_WIDTH, 3), np.float32)
            if self._kinect.get_map_depth_to_camera(map_ary):
                result = map_ary
        elif from_type == 'depth' and to_type == 'color':
            map_ary = np.empty((DEPTH_HEIGHT, DEPTH_WIDTH, 2), np.float32)
            if self._kinect.get_map_depth_to_color(map_ary):
                result = map_ary
        elif from_type == 'color' and to_type == 'depth':
            map_ary = np.empty((COLOR_HEIGHT, COLOR_WIDTH, 2), np.float32)
            if self._kinect.get_map_color_depth(map_ary):
                result = map_ary
        return result

    def wait_for_worker(self, first_tick=0, timeout=5):
        """
        Wait for the frame fetching working to collect
        data for the first frame.
        """
        start = time.time()
        while self._kinect.get_tick() == first_tick:
            time.sleep(0.1)
            if time.time() - start >= timeout:
                raise IOError('Kinect took too long. Try restarting the device.')

    def iter_frames(self, limit_fps=60):
        """
        Iterate through sensor data.

        Args:
            limit_fps: Cap the framerate/datarate

        Returns:
            array of each type of data being collected.
        """
        i = 0
        frame_time = 1.0 / limit_fps
        start_time = time.time()
        while True:

            data = [i]
            if self.sensor_flags & F_SENSOR_COLOR:
                data.append(self.get_color_image())
            if self.sensor_flags & F_SENSOR_DEPTH:
                data.append(self.get_depth_map())
            if self.sensor_flags & F_SENSOR_IR:
                data.append(self.get_ir_image())
            if self.sensor_flags & F_SENSOR_BODY:
                data.append(self.get_bodies())
            if self.sensor_flags & F_SENSOR_AUDIO:
                data.append(self.get_audio_frames())
            if self.mapping_flags & F_MAP_COLOR_CAM:
                data.append(self.map('color', 'camera'))
            if self.mapping_flags & F_MAP_DEPTH_CAM:
                data.append(self.map('depth', 'camera'))
            if self.mapping_flags & F_MAP_DEPTH_COLOR:
                data.append(self.map('depth', 'color'))
            if self.mapping_flags & F_MAP_COLOR_DEPTH:
                data.append(self.map('color', 'depth'))
            yield data

            end_time = time.time()
            if start_time - end_time < frame_time:
                time.sleep(frame_time - (start_time - end_time))
                end_time = time.time()

            start_time = end_time
            i += 1