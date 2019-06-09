#pragma once
#include "stdafx.h"
#include <Kinect.h>
#define EXPORTFUNC extern "C" __declspec(dllexport)


EXPORTFUNC bool init_kinect(int sensor_flags);
EXPORTFUNC void close_kinect();

EXPORTFUNC bool read_sensors();

HRESULT run_audio_worker();
DWORD WINAPI audio_worker_wrapper(_In_ LPVOID lpParameter);
void process_audio_subframe(IAudioBeamSubFrame* subframe, int index);

EXPORTFUNC bool get_color_data(UINT8* array, bool do_read);
EXPORTFUNC bool get_ir_data(UINT16* array, bool do_read);
EXPORTFUNC bool get_depth_data(UINT16* array, bool do_read);
EXPORTFUNC int get_audio_data(FLOAT* array, FLOAT* meta_array);