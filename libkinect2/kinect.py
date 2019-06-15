"""
The Kinect2 class
"""
from .dll_lib import *
import numpy as np
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

    def connect(self):
        """
        Connect to the device.
        """
        if not self._kinect.init_kinect(self.sensor_flags, self.mapping_flags):
            raise IOError('Unable to init Kinect2 Sensor.')
        return True

    def disconnect(self):
        """
        Disconnect fromthe device.
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
        meta_ary = np.empty((AUDIO_BUF_LEN * 2,), np.float32)
        frame_cnt = self._kinect.get_audio_data(audio_ary, meta_ary)
        return frame_cnt, audio_ary, meta_ary

    def get_audio_frames(self):
        """
        Get the latest audio frames.

        Returns:
            array of (angle, confidence, sample data)
        """
        frame_cnt, audio_ary, meta_ary = self._get_raw_audio()
        frames = []
        for i in range(frame_cnt):
            beam_angle = meta_ary[i*2]
            beam_conf = meta_ary[i*2+1]
            samples = audio_ary[i*SUBFRAME_SIZE:(i+1)*SUBFRAME_SIZE]
            frames.append((beam_angle, beam_conf, samples))
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
        

class Body:
    """
    A body tracked by the Kinect.

    Attributes:
        idx: The tracking index
        tracked: If this body is tracked
        engaged: State of person's engagement
        restricted: If the body is restricted
    """
    def __init__(self, idx, body_ary, joints_ary):
        """
        Create a body from raw body/joint data.

        Note:
            Should not be called by user.
            Use `kinect.get_bodies()`.
        """
        self.idx = idx
        self._body_ary = body_ary
        self._joints_ary = joints_ary
        self._load_props()

    def _load_props(self):
        self.tracked = bool(self._body_ary[0])
        self.engaged = DETECTION_MAP[self._body_ary[1]]
        self.restricted = bool(self._body_ary[2])
        ## These are not yet supported by Kinect2 )':
        ## self.neutral = DETECTION_MAP[self._body_ary[7]]
        ## self.happy = DETECTION_MAP[self._body_ary[8]]
        ## self.looking_away = DETECTION_MAP[self._body_ary[13]]
        ## self.glasses = DETECTION_MAP[self._body_ary[14]]

    def keys(self):
        """
        Return a list of keys (joints).

        Returns:
            list of joint names
        """
        return JOINT_MAP.keys()

    def __getitem__(self, joint_name):
        """
        Get a joint of this body by name.

        Returns:
            `Joint`

        Note:
            Use `body.keys()` for list of joints.
        """
        joint_idx = JOINT_MAP[joint_name.lower()]
        if joint_idx == -1:
            raise NotImplementedError()
        return Joint(joint_name, self._body_ary, self._joints_ary[joint_idx])

    def __repr__(self):
        if self.tracked:
            state = ' [Tracked]'
        else:
            state = ''
        return '<Body ({}){}>'.format(self.idx, state)


class Joint:
    """
    A joint.

    Attributes:
        name: Name of the joint
        tracking: The current tracking state
        color_pos: Position in the color camera space - (x, y)
        depth_pos: Position in the depth sensor space - (x, y)
        orientation: Orientation as (w, x, y, z)
        state: The state of the joint if provided by Kinect API
    """
    def __init__(self, joint_name, body_ary, joint_ary):
        """
        Create a joint from raw body/joint data.

        Note:
            Should not be called by user.
            Use `body[joint_name]`.
        """
        self.name = joint_name
        self._body_ary = body_ary
        self._joint_ary = joint_ary
        self._load_props()
    
    def _load_props(self):
        self.tracking = TRACKING_MAP[self._joint_ary[0]]
        self.color_pos = (self._joint_ary[1], self._joint_ary[2])
        self.depth_pos = (self._joint_ary[3], self._joint_ary[4])
        self.orientation = (
            self._joint_ary[5] / FLOAT_MULT,
            self._joint_ary[6] / FLOAT_MULT,
            self._joint_ary[7] / FLOAT_MULT,
            self._joint_ary[8] / FLOAT_MULT
        )
        if self.name == 'hand_left':
            self.confidence = HIGH_CONFIDENCE_MAP[self._body_ary[3]]
            self.state = HAND_MAP[self._body_ary[4]]
        elif self.name == 'hand_right':
            self.confidence = HIGH_CONFIDENCE_MAP[self._body_ary[5]]
            self.state = HAND_MAP[self._body_ary[6]]
        else:
            self.confidence = None
            self.state = None

    def __repr__(self):
        if self.state:
            return '<Joint {} [{}] [{}]>'.format(self.name.title(), self.state, self.tracking)
        else:
            return '<Joint {} [{}]>'.format(self.name.title(), self.tracking)
