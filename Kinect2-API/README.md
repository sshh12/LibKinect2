# Kinect2 API

This folder contains all the C++ code for generating the language agnostic DLL for interfacing with the Kinect2 Sensor.

## Dev

#### Building
1. Install the [Kinect2 SDK](https://www.microsoft.com/en-us/search?q=kinect+2+sdk)
2. Set the environment var `KINECTSDK20_DIR` to its install location
3. Open the project (`Kinect2-API.sln`) with [Visual Studio](https://visualstudio.microsoft.com/)
4. Use default Release/x64 settings to build `Kinect2-API.dll`

#### Methods Outline

See code for more info on exported functions.

```c++
// Sensor Flags
#define F_SENSOR_COLOR     0x00000001
#define F_SENSOR_DEPTH     0x00000010
#define F_SENSOR_IR        0x00000100
#define F_SENSOR_BODY      0x00001000
#define F_SENSOR_MULTI     0x00001111
#define F_SENSOR_AUDIO     0x00010000
#define F_MAP_COLOR_CAM    0x00000002
#define F_MAP_DEPTH_CAM    0x00000020
#define F_MAP_DEPTH_COLOR  0x00000200
#define F_MAP_COLOR_DEPTH  0x00002000
```

```c++
// Other Constants
#define COLOR_WIDTH        1920
#define COLOR_HEIGHT       1080
#define COLOR_CHANNELS     4
#define DEPTH_WIDTH        512
#define DEPTH_HEIGHT       424
#define IR_WIDTH           512
#define IR_HEIGHT          424
#define MAX_SUBFRAMES      8
#define AUDIO_BUF_LEN      512
#define SUBFRAME_SIZE      256
#define MAX_BODIES         6
#define BODY_PROPS         15
#define MAX_JOINTS         25
#define JOINT_PROPS        9
#define FLOAT_MULT         100000
```

```c++
// Start collecting data from the Kinect2 (flags determine what kind of data)
bool init_kinect(int sensor_flags, int mapping_flags);
// Stop collecting data and clean up.
void close_kinect();
```

```c++
// Pause the thread responsable for fetching frames
void pause_worker();
// Resume that thread
void resume_worker();
// Get the current frame count
int get_tick();
```

```c++
// Methods for collecting multidimensional data from the sensor:

// Each of these will overwrite the provided array with data
// in the shape given.

// These will have undefined behavior when called if used w/o
// appropriate flags on init.

// (COLOR_HEIGHT, COLOR_WIDTH, COLOR_CHANNELS)
bool get_color_data(UINT8* array);
// (IR_HEIGHT, IR_WIDTH, 1)
bool get_ir_data(UINT16* array);
// (DEPTH_HEIGHT, DEPTH_WIDTH, 1)
bool get_depth_data(UINT16* array);
// (MAX_BODIES, BODY_PROPS) (MAX_BODIES, MAX_JOINTS, JOINT_PROPS)
// See code for how this is encoded.
bool get_body_data(UINT8* body_array, INT32* joint_array);
// (AUDIO_BUF_LEN * SUBFRAME_SIZE) (AUDIO_BUF_LEN * 2,)
// See code for how this is encoded.
int get_audio_data(FLOAT* array, FLOAT* meta_array);
// (COLOR_HEIGHT, COLOR_WIDTH, 3)
bool get_map_color_to_camera(FLOAT* array);
// (DEPTH_HEIGHT, DEPTH_WIDTH, 3)
bool get_map_depth_to_camera(FLOAT* array);
// (DEPTH_HEIGHT, DEPTH_WIDTH, 2)
bool get_map_depth_to_color(FLOAT* array);
// (COLOR_HEIGHT, COLOR_WIDTH, 2)
bool get_map_color_depth(FLOAT* array);
```