#pragma once

#include "stdafx.h"
#include <Kinect.h>

#define EXPORTFUNC extern "C" __declspec(dllexport)
#define SAFE_RELEASE(p) { if ( (p) ) { (p)->Release(); (p) = 0; } }


EXPORTFUNC bool init_kinect(int sensor_flags);
EXPORTFUNC void close_kinect();

HRESULT run_multi_worker();
DWORD WINAPI multi_worker_wrapper(_In_ LPVOID lp_param);

HRESULT run_audio_worker();
DWORD WINAPI audio_worker_wrapper(_In_ LPVOID lp_param);
void process_audio_subframe(IAudioBeamSubFrame* subframe, int index);

EXPORTFUNC bool get_color_data(UINT8* array);
EXPORTFUNC bool get_ir_data(UINT16* array);
EXPORTFUNC bool get_depth_data(UINT16* array);
EXPORTFUNC int get_audio_data(FLOAT* array, FLOAT* meta_array);