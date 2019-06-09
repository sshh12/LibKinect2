#include "stdafx.h"
#include "Kinect2-API.h"
#include <Kinect.h>
#include <cstring>
#include <iostream>
#include <mutex>


#define EXPORTFUNC extern "C" __declspec(dllexport)


#define WORKER_TIMEOUT  5000
#define F_SENSOR_COLOR  0x00000001
#define F_SENSOR_DEPTH  0x00000010
#define F_SENSOR_IR     0x00000100
#define F_SENSOR_MULTI  0x00000111
#define F_SENSOR_AUDIO  0x00001000
#define COLOR_WIDTH     1920
#define COLOR_HEIGHT    1080
#define COLOR_CHANNELS  4
#define DEPTH_WIDTH     512
#define DEPTH_HEIGHT    424
#define IR_WIDTH        512
#define IR_HEIGHT       424
#define MAX_SUBFRAMES   8
#define AUDIO_BUF_LEN   512
#define SUBFRAME_SIZE   256


int                      sensors = 0;
IKinectSensor*           sensor;
IMultiSourceFrameReader* multi_reader;
IAudioBeamFrameReader*   audio_reader;
WAITABLE_HANDLE          audio_frame_event;
HANDLE                   audio_terminate = NULL;
HANDLE                   audio_worker_thread;

UINT8  buffer_color[COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS];
UINT16 buffer_depth[DEPTH_WIDTH * DEPTH_HEIGHT];
UINT16 buffer_ir[IR_WIDTH * IR_HEIGHT];

std::mutex buffer_audio_lock;
FLOAT  buffer_audio[AUDIO_BUF_LEN * SUBFRAME_SIZE];
FLOAT  buffer_audio_meta[AUDIO_BUF_LEN * 2];
UINT32 buffer_audio_used = 0;


EXPORTFUNC bool init_kinect(int sensor_flags) {
    if (!sensor_flags || FAILED(GetDefaultKinectSensor(&sensor))) {
        return false;
    }
    sensors = sensor_flags;
    if (sensor) {
        sensor->Open();
        int source_types = 0; 
        if (sensors & F_SENSOR_COLOR) {
            source_types |= FrameSourceTypes::FrameSourceTypes_Color;
        }
        if (sensors & F_SENSOR_DEPTH) {
            source_types |= FrameSourceTypes::FrameSourceTypes_Depth;
        }
        if (sensors & F_SENSOR_IR) {
            source_types |= FrameSourceTypes::FrameSourceTypes_Infrared;
        }
        if (sensors & F_SENSOR_AUDIO) {
            IAudioSource* audio_source;
            sensor->get_AudioSource(&audio_source);
            audio_source->OpenReader(&audio_reader);
            audio_reader->SubscribeFrameArrived(&audio_frame_event);
            audio_terminate = CreateEvent(NULL, FALSE, FALSE, NULL);
            CreateThread(NULL, 0, &audio_worker_wrapper, NULL, 0, NULL);
            audio_source->Release();
        }
        if (sensors & F_SENSOR_MULTI) {
            sensor->OpenMultiSourceFrameReader(source_types, &multi_reader);
        }
        return true;
    }
    return false;
}


EXPORTFUNC void close_kinect() {
    sensor->Close();
    sensor->Release();
}


EXPORTFUNC bool read_sensors() {
    IMultiSourceFrame* frame = NULL;
    if (FAILED(multi_reader->AcquireLatestFrame(&frame))) {
        return false;
    }
    if (sensors & F_SENSOR_COLOR) {
        IColorFrame* frame_color;
        IColorFrameReference* frameref_color = NULL;
        frame->get_ColorFrameReference(&frameref_color);
        frameref_color->AcquireFrame(&frame_color);
        if (frameref_color) frameref_color->Release();
        if (frame_color) {
            frame_color->CopyConvertedFrameDataToArray(COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS, buffer_color, ColorImageFormat_Rgba);
            frame_color->Release();
        }
    }
    if (sensors & F_SENSOR_DEPTH) {
        IDepthFrame* frame_depth;
        IDepthFrameReference* frameref_depth = NULL;
        frame->get_DepthFrameReference(&frameref_depth);
        frameref_depth->AcquireFrame(&frame_depth);
        if (frameref_depth) frameref_depth->Release();
        if (frame_depth) {
            frame_depth->CopyFrameDataToArray(DEPTH_WIDTH * DEPTH_HEIGHT, buffer_depth);
            frame_depth->Release();
        }
    }
    if (sensors & F_SENSOR_IR) {
        IInfraredFrame* frame_ir;
        IInfraredFrameReference* frameref_ir = NULL;
        frame->get_InfraredFrameReference(&frameref_ir);
        frameref_ir->AcquireFrame(&frame_ir);
        if (frameref_ir) frameref_ir->Release();
        if (frame_ir) {
            frame_ir->CopyFrameDataToArray(IR_WIDTH * IR_HEIGHT, buffer_ir);
            frame_ir->Release();
        }
    }
    return true;
}


DWORD WINAPI audio_worker_wrapper(_In_ LPVOID lpParameter)
{
    HRESULT hr = S_OK;
    hr = run_audio_worker();
    return SUCCEEDED(hr) ? 0 : 1;
}


HRESULT run_audio_worker() {
    bool running = true;
    HANDLE handles[] = { (HANDLE)audio_frame_event, audio_terminate };
    IAudioBeamFrameArrivedEventArgs* audio_frame_event_args = NULL;
    IAudioBeamFrameReference* audio_frame_ref = NULL;
    IAudioBeamFrameList* audio_frames = NULL;
    IAudioBeamFrame* audio_frame;
    UINT32 subframe_count;
    while (running) {
        DWORD result = WaitForMultipleObjects(_countof(handles), handles, FALSE, WORKER_TIMEOUT);
        if (result == WAIT_OBJECT_0) {
            audio_reader->GetFrameArrivedEventData(audio_frame_event, &audio_frame_event_args);
            audio_frame_event_args->get_FrameReference(&audio_frame_ref);
            if (FAILED(audio_frame_ref->AcquireBeamFrames(&audio_frames))) {
                audio_frame_event_args->Release();
                audio_frame_ref->Release();
                continue;
            }
            audio_frames->OpenAudioBeamFrame(0, &audio_frame);
            audio_frame->get_SubFrameCount(&subframe_count);
            buffer_audio_lock.lock();
            if (subframe_count + buffer_audio_used >= AUDIO_BUF_LEN) {
                buffer_audio_used = 0;
            }
            for (UINT32 i = 0; i < subframe_count; i++) {
                IAudioBeamSubFrame* subframe = NULL;
                audio_frame->GetSubFrame(i, &subframe);
                process_audio_subframe(subframe, i);
                subframe->Release();
            }
            buffer_audio_used += subframe_count;
            buffer_audio_lock.unlock();
            audio_frame_event_args->Release();
            audio_frame_ref->Release();
            audio_frames->Release();
            audio_frame->Release();
        }
        else {
            running = false;
        }
    }
    return S_OK;
}


void process_audio_subframe(IAudioBeamSubFrame* subframe, int index) {
    float* audio_buf = NULL;
    float beam_angle;
    float beam_conf;
    subframe->get_BeamAngle(&beam_angle);
    subframe->get_BeamAngleConfidence(&beam_conf);
    UINT buf_size = 0;
    subframe->AccessUnderlyingBuffer(&buf_size, (BYTE **)&audio_buf);
    buffer_audio_meta[(index + buffer_audio_used) * 2] = beam_angle;
    buffer_audio_meta[(index + buffer_audio_used) * 2 + 1] = beam_conf;
    memcpy(buffer_audio + (index + buffer_audio_used) * SUBFRAME_SIZE, audio_buf, SUBFRAME_SIZE * sizeof(FLOAT));
}


EXPORTFUNC bool get_color_data(UINT8* array, bool do_read) {
    if (do_read) read_sensors();
    memcpy(array, buffer_color, COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS * sizeof(UINT8));
    return true;
}


EXPORTFUNC bool get_ir_data(UINT16* array, bool do_read) {
    if (do_read) read_sensors();
    memcpy(array, buffer_ir, IR_WIDTH * IR_HEIGHT * sizeof(UINT16));
    return true;
}


EXPORTFUNC bool get_depth_data(UINT16* array, bool do_read) {
    if (do_read) read_sensors();
    memcpy(array, buffer_depth, DEPTH_WIDTH * DEPTH_HEIGHT * sizeof(UINT16));
    return true;
}

EXPORTFUNC int get_audio_data(FLOAT* array, FLOAT* meta_array) {
    if (buffer_audio_used == 0) {
        return 0;
    }
    buffer_audio_lock.lock();
    int len = buffer_audio_used;
    memcpy(array, buffer_audio, AUDIO_BUF_LEN * SUBFRAME_SIZE * sizeof(FLOAT));
    memcpy(meta_array, buffer_audio_meta, AUDIO_BUF_LEN * 2 * sizeof(FLOAT));
    buffer_audio_used = 0;
    buffer_audio_lock.unlock();
    return len;
}