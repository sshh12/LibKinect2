#pragma once

#include "stdafx.h"
#include <Kinect.h>

#define EXPORTFUNC extern "C" __declspec(dllexport)
#define SAFE_RELEASE(p) { if ( (p) ) { (p)->Release(); (p) = 0; } }


EXPORTFUNC bool init_kinect(int sensor_flags, int mapping_flags);
EXPORTFUNC void close_kinect();

EXPORTFUNC void pause_worker();
EXPORTFUNC void resume_worker();
EXPORTFUNC int get_tick();

HRESULT run_multi_worker();
DWORD WINAPI multi_worker_wrapper(_In_ LPVOID lp_param);
inline void process_body(IBody* body, int body_idx, Joint* joints, JointOrientation* joint_orients);

HRESULT run_audio_worker();
DWORD WINAPI audio_worker_wrapper(_In_ LPVOID lp_param);
inline void process_audio_subframe(IAudioBeamSubFrame* subframe, int index);

EXPORTFUNC bool get_color_data(UINT8* array);
EXPORTFUNC bool get_ir_data(UINT16* array);
EXPORTFUNC bool get_depth_data(UINT16* array);
EXPORTFUNC bool get_body_data(UINT8* body_array, INT32* joint_array);
EXPORTFUNC int get_audio_data(FLOAT* array, FLOAT* meta_array);
EXPORTFUNC bool get_map_color_to_camera(FLOAT* array);
EXPORTFUNC bool get_map_depth_to_camera(FLOAT* array);
EXPORTFUNC bool get_map_depth_to_color(FLOAT* array);
EXPORTFUNC bool get_map_color_depth(FLOAT* array);