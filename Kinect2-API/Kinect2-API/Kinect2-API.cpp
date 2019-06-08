#include "stdafx.h"
#include <Kinect.h>
#include <cstring>


#define EXPORTFUNC extern "C" __declspec(dllexport)


#define F_SENSOR_COLOR 0x00000001
#define F_SENSOR_DEPTH 0x00000010
#define COLOR_WIDTH 1920
#define COLOR_HEIGHT 1080
#define COLOR_CHANNELS 4
#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424


IKinectSensor* sensor;
IColorFrameReader* reader_color;
IDepthFrameReader* reader_depth;


EXPORTFUNC bool init_kinect(int sensor_flags) {
	if (!sensor_flags || FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->Open();
		if (sensor_flags & F_SENSOR_COLOR) {
			IColorFrameSource* source_color = NULL;
			sensor->get_ColorFrameSource(&source_color);
			source_color->OpenReader(&reader_color);
			if (source_color) {
				source_color->Release();
				source_color = NULL;
			}
		}
		if (sensor_flags & F_SENSOR_DEPTH) {
			IDepthFrameSource* source_depth = NULL;
			sensor->get_DepthFrameSource(&source_depth);
			source_depth->OpenReader(&reader_depth);
			if (source_depth) {
				source_depth->Release();
				source_depth = NULL;
			}
		}
		return true;
	} else {
		return false;
	}
}


EXPORTFUNC bool get_color_data(UINT8* array) {
	IColorFrame* frame = NULL;
	bool success = false;
	if (SUCCEEDED(reader_color->AcquireLatestFrame(&frame))) {
		frame->CopyConvertedFrameDataToArray(COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS, array, ColorImageFormat_Bgra);
		success = true;
	}
	if (frame) frame->Release();
	return success;
}


EXPORTFUNC bool get_depth_data(UINT16* array) {
	IDepthFrame* frame = NULL;
	bool success = false;
	if (SUCCEEDED(reader_depth->AcquireLatestFrame(&frame))) {
		frame->CopyFrameDataToArray(DEPTH_WIDTH * DEPTH_HEIGHT, array);
		success = true;
	}
	if (frame) frame->Release();
	return success;
}