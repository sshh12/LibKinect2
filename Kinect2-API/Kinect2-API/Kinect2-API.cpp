#include "stdafx.h"
#include "Kinect2-API.h"
#include <Kinect.h>
#include <cstring>
#include <iostream>
#include <mutex>


#define WORKER_TIMEOUT  5000
#define F_SENSOR_COLOR  0x00000001
#define F_SENSOR_DEPTH  0x00000010
#define F_SENSOR_IR     0x00000100
#define F_SENSOR_BODY   0x00001000
#define F_SENSOR_MULTI  0x00001111
#define F_SENSOR_AUDIO  0x00010000
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
#define MAX_BODIES      6
#define BODY_PROPS      15
#define MAX_JOINTS      25
#define JOINT_PROPS     9
#define FLOAT_MULT      100000


int                      sensors = 0;
IKinectSensor*           sensor;
std::mutex               worker_lock;

int                      multi_tick = 0;
IMultiSourceFrameReader* multi_reader;
WAITABLE_HANDLE          multi_frame_event;
HANDLE                   multi_terminate = NULL;
HANDLE                   multi_worker_thread;
UINT8                    buffer_color[COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS];
UINT16                   buffer_depth[DEPTH_WIDTH * DEPTH_HEIGHT];
UINT16                   buffer_ir[IR_WIDTH * IR_HEIGHT];
UINT8                    buffer_bodies[MAX_BODIES * BODY_PROPS];
INT32                    buffer_joints[MAX_BODIES * MAX_JOINTS * JOINT_PROPS];
ICoordinateMapper*       coord_mapper;
std::mutex               buffer_multi_lock;

IAudioBeamFrameReader*   audio_reader;
WAITABLE_HANDLE          audio_frame_event;
HANDLE                   audio_terminate = NULL;
HANDLE                   audio_worker_thread;
FLOAT                    buffer_audio[AUDIO_BUF_LEN * SUBFRAME_SIZE];
FLOAT                    buffer_audio_meta[AUDIO_BUF_LEN * 2];
UINT32                   buffer_audio_used = 0;
std::mutex               buffer_audio_lock;


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
        if (sensors & F_SENSOR_BODY) {
            source_types |= FrameSourceTypes::FrameSourceTypes_Body;
        }
        if (sensors & F_SENSOR_MULTI) {
            sensor->OpenMultiSourceFrameReader(source_types, &multi_reader);
            sensor->get_CoordinateMapper(&coord_mapper);
            multi_reader->SubscribeMultiSourceFrameArrived(&multi_frame_event);
            multi_terminate = CreateEvent(NULL, FALSE, FALSE, NULL);
            CreateThread(NULL, 0, &multi_worker_wrapper, NULL, 0, NULL);
        }
        if (sensors & F_SENSOR_AUDIO) {
            IAudioSource* audio_source;
            sensor->get_AudioSource(&audio_source);
            audio_source->OpenReader(&audio_reader);
            audio_reader->SubscribeFrameArrived(&audio_frame_event);
            audio_terminate = CreateEvent(NULL, FALSE, FALSE, NULL);
            CreateThread(NULL, 0, &audio_worker_wrapper, NULL, 0, NULL);
            SAFE_RELEASE(audio_source);
        }
        return true;
    }
    return false;
}


EXPORTFUNC void close_kinect() {
    if (sensors & F_SENSOR_MULTI) {
        SetEvent(multi_terminate);
        WaitForSingleObject(multi_worker_thread, INFINITE);
        CloseHandle(multi_worker_thread);
        CloseHandle(multi_terminate);
        multi_reader->UnsubscribeMultiSourceFrameArrived(multi_frame_event);
        SAFE_RELEASE(multi_reader);
    }
    if (sensors & F_SENSOR_AUDIO) {
        SetEvent(audio_terminate);
        WaitForSingleObject(audio_worker_thread, INFINITE);
        CloseHandle(audio_worker_thread);
        CloseHandle(audio_terminate);
        audio_reader->UnsubscribeFrameArrived(audio_frame_event);
        SAFE_RELEASE(audio_reader);
    }
    sensor->Close();
    SAFE_RELEASE(sensor);
}


EXPORTFUNC int get_tick() {
    return multi_tick;
}


EXPORTFUNC void pause_worker() {
    worker_lock.lock();
}


EXPORTFUNC void resume_worker() {
    worker_lock.unlock();
}


DWORD WINAPI multi_worker_wrapper(_In_ LPVOID lp_param) {
    HRESULT hr = S_OK;
    hr = run_multi_worker();
    return SUCCEEDED(hr) ? 0 : 1;
}


HRESULT run_multi_worker() {
    bool running = true;
    HANDLE handles[] = { (HANDLE)multi_frame_event, multi_terminate };
    IMultiSourceFrameArrivedEventArgs* multi_event_args = NULL;
    IMultiSourceFrameReference* multi_ref = NULL;
    IMultiSourceFrame* multi_frame = NULL;
    IColorFrame* frame_color = NULL;
    IColorFrameReference* frameref_color = NULL;
    IDepthFrame* frame_depth = NULL;
    IDepthFrameReference* frameref_depth = NULL;
    IInfraredFrame* frame_ir = NULL;
    IInfraredFrameReference* frameref_ir = NULL;
    IBodyFrame* frame_body = NULL;
    IBodyFrameReference* frameref_body = NULL;
    IBody* bodies[BODY_COUNT] = { 0 };
    Joint joints[MAX_JOINTS];
    JointOrientation joint_orients[MAX_JOINTS];
    while (running) {
        DWORD result = WaitForMultipleObjects(_countof(handles), handles, FALSE, WORKER_TIMEOUT);
        if (result == WAIT_OBJECT_0) {
            multi_reader->GetMultiSourceFrameArrivedEventData(multi_frame_event, &multi_event_args);
            multi_event_args->get_FrameReference(&multi_ref);
            multi_ref->AcquireFrame(&multi_frame);
            buffer_multi_lock.lock();
            worker_lock.lock();
            if (sensors & F_SENSOR_COLOR) {
                multi_frame->get_ColorFrameReference(&frameref_color);
                frameref_color->AcquireFrame(&frame_color);
                frame_color->CopyConvertedFrameDataToArray(COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS, buffer_color, ColorImageFormat_Rgba);
            }
            if (sensors & F_SENSOR_DEPTH) {
                multi_frame->get_DepthFrameReference(&frameref_depth);
                frameref_depth->AcquireFrame(&frame_depth);
                frame_depth->CopyFrameDataToArray(DEPTH_WIDTH * DEPTH_HEIGHT, buffer_depth);
            }
            if (sensors & F_SENSOR_IR) {
                multi_frame->get_InfraredFrameReference(&frameref_ir);
                frameref_ir->AcquireFrame(&frame_ir);
                frame_ir->CopyFrameDataToArray(IR_WIDTH * IR_HEIGHT, buffer_ir);
            }
            if (sensors & F_SENSOR_BODY) {
                multi_frame->get_BodyFrameReference(&frameref_body);
                frameref_body->AcquireFrame(&frame_body);
                frame_body->GetAndRefreshBodyData(_countof(bodies), bodies);
                for (int b_idx = 0; b_idx < BODY_COUNT; b_idx++) {
                    process_body(bodies[b_idx], b_idx, joints, joint_orients);
                }
            }
            worker_lock.unlock();
            buffer_multi_lock.unlock();
            multi_tick += 1;
            SAFE_RELEASE(frameref_color);
            SAFE_RELEASE(frame_color);
            SAFE_RELEASE(frameref_depth);
            SAFE_RELEASE(frame_depth);
            SAFE_RELEASE(frameref_ir);
            SAFE_RELEASE(frame_ir);
            SAFE_RELEASE(frameref_body);
            SAFE_RELEASE(frame_body);
            SAFE_RELEASE(multi_event_args);
            SAFE_RELEASE(multi_ref);
        }
        else {
            running = false;
        }
    }
    return S_OK;
}


inline void process_body(IBody* body, int body_idx, Joint* joints, JointOrientation* joint_orients) {
    BOOLEAN tracked;
    int body_offset = body_idx * BODY_PROPS;
    int joint_offset;
    Joint joint;
    Vector4 joint_orient;
    CameraSpacePoint joint_pos;
    ColorSpacePoint color_pos;
    DepthSpacePoint depth_pos;
    body->get_IsTracked(&tracked);
    buffer_bodies[body_offset] = !!tracked;
    if (tracked) {
        body->get_Engaged((DetectionResult*)&buffer_bodies[body_offset + 1]);
        body->get_IsRestricted((BOOLEAN*)&buffer_bodies[body_offset + 2]);
        body->get_HandLeftConfidence((TrackingConfidence*)&buffer_bodies[body_offset + 3]);
        body->get_HandLeftState((HandState*)&buffer_bodies[body_offset + 4]);
        body->get_HandRightConfidence((TrackingConfidence*)&buffer_bodies[body_offset + 5]);
        body->get_HandRightState((HandState*)&buffer_bodies[body_offset + 6]);
        body->GetExpressionDetectionResults(2, (DetectionResult*)&buffer_bodies[body_offset + 7]);
        body->GetActivityDetectionResults(5, (DetectionResult*)&buffer_bodies[body_offset + 7 + 2]);
        body->GetAppearanceDetectionResults(1, (DetectionResult*)&buffer_bodies[body_offset + 7 + 2 + 5]);
        body->GetJoints(MAX_JOINTS, joints);
        body->GetJointOrientations(MAX_JOINTS, joint_orients);
        for (int j_idx = 0; j_idx < MAX_JOINTS; j_idx++) {
            joint_offset = body_idx * MAX_JOINTS * JOINT_PROPS + j_idx * JOINT_PROPS;
            joint = joints[j_idx];
            joint_orient = joint_orients[j_idx].Orientation;
            joint_pos = joint.Position;
            coord_mapper->MapCameraPointToColorSpace(joint_pos, &color_pos);
            coord_mapper->MapCameraPointToDepthSpace(joint_pos, &depth_pos);
            buffer_joints[joint_offset] = joint.TrackingState;
            buffer_joints[joint_offset + 1] = (int)color_pos.X;
            buffer_joints[joint_offset + 2] = (int)color_pos.Y;
            buffer_joints[joint_offset + 3] = (int)depth_pos.X;
            buffer_joints[joint_offset + 4] = (int)depth_pos.Y;
            buffer_joints[joint_offset + 5] = (int)(joint_orient.w * FLOAT_MULT);
            buffer_joints[joint_offset + 6] = (int)(joint_orient.x * FLOAT_MULT);
            buffer_joints[joint_offset + 7] = (int)(joint_orient.y * FLOAT_MULT);
            buffer_joints[joint_offset + 8] = (int)(joint_orient.z * FLOAT_MULT);
        }
    }
}


DWORD WINAPI audio_worker_wrapper(_In_ LPVOID lp_param) {
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
    IAudioBeamFrame* audio_frame = NULL;
    IAudioBeamSubFrame* subframe = NULL;
    UINT32 subframe_count;
    while (running) {
        DWORD result = WaitForMultipleObjects(_countof(handles), handles, FALSE, WORKER_TIMEOUT);
        if (result == WAIT_OBJECT_0) {
            audio_reader->GetFrameArrivedEventData(audio_frame_event, &audio_frame_event_args);
            audio_frame_event_args->get_FrameReference(&audio_frame_ref);
            if (SUCCEEDED(audio_frame_ref->AcquireBeamFrames(&audio_frames))) {
                audio_frames->OpenAudioBeamFrame(0, &audio_frame);
                audio_frame->get_SubFrameCount(&subframe_count);
                buffer_audio_lock.lock();
                if (subframe_count + buffer_audio_used >= AUDIO_BUF_LEN) {
                    buffer_audio_used = 0;
                }
                for (UINT32 i = 0; i < subframe_count; i++) {
                    audio_frame->GetSubFrame(i, &subframe);
                    process_audio_subframe(subframe, i);
                    SAFE_RELEASE(subframe);
                }
                buffer_audio_used += subframe_count;
                buffer_audio_lock.unlock();
            } 
            SAFE_RELEASE(audio_frame_event_args);
            SAFE_RELEASE(audio_frame_ref);
            SAFE_RELEASE(audio_frames);
            SAFE_RELEASE(audio_frame);
        }
        else {
            running = false;
        }
    }
    return S_OK;
}


inline void process_audio_subframe(IAudioBeamSubFrame* subframe, int index) {
    float* audio_buf = NULL;
    float beam_angle;
    float beam_conf;
    subframe->get_BeamAngle(&beam_angle);
    subframe->get_BeamAngleConfidence(&beam_conf);
    UINT buf_size;
    subframe->AccessUnderlyingBuffer(&buf_size, (BYTE **)&audio_buf);
    buffer_audio_meta[(index + buffer_audio_used) * 2] = beam_angle;
    buffer_audio_meta[(index + buffer_audio_used) * 2 + 1] = beam_conf;
    memcpy(buffer_audio + (index + buffer_audio_used) * SUBFRAME_SIZE, audio_buf, SUBFRAME_SIZE * sizeof(FLOAT));
}


EXPORTFUNC bool get_color_data(UINT8* array) {
    buffer_multi_lock.lock();
    memcpy(array, buffer_color, COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS * sizeof(UINT8));
    buffer_multi_lock.unlock();
    return true;
}


EXPORTFUNC bool get_ir_data(UINT16* array) {
    buffer_multi_lock.lock();
    memcpy(array, buffer_ir, IR_WIDTH * IR_HEIGHT * sizeof(UINT16));
    buffer_multi_lock.unlock();
    return true;
}


EXPORTFUNC bool get_depth_data(UINT16* array) {
    buffer_multi_lock.lock();
    memcpy(array, buffer_depth, DEPTH_WIDTH * DEPTH_HEIGHT * sizeof(UINT16));
    buffer_multi_lock.unlock();
    return true;
}


EXPORTFUNC bool get_body_data(UINT8* body_array, INT32* joint_array) {
    buffer_multi_lock.lock();
    memcpy(body_array, buffer_bodies, MAX_BODIES * BODY_PROPS * sizeof(UINT8));
    memcpy(joint_array, buffer_joints, MAX_BODIES * MAX_JOINTS * JOINT_PROPS * sizeof(INT32));
    buffer_multi_lock.unlock();
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