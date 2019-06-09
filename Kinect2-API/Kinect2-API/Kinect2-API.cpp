#include "stdafx.h"
#include <Kinect.h>
#include <cstring>
#include <iostream>


#define EXPORTFUNC extern "C" __declspec(dllexport)


#define F_SENSOR_COLOR 0x00000001
#define F_SENSOR_DEPTH 0x00000010
#define F_SENSOR_IR    0x00000100
#define COLOR_WIDTH    1920
#define COLOR_HEIGHT   1080
#define COLOR_CHANNELS 4
#define DEPTH_WIDTH    512
#define DEPTH_HEIGHT   424
#define IR_WIDTH       512
#define IR_HEIGHT      424


int sensors = 0;
IKinectSensor* sensor;
IMultiSourceFrameReader* multi_reader;

UINT8 buffer_color[COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS];
UINT16 buffer_depth[DEPTH_WIDTH * DEPTH_HEIGHT];
UINT16 buffer_ir[IR_WIDTH * IR_HEIGHT];


EXPORTFUNC bool init_kinect(int sensor_flags) {
	if (!sensor_flags || FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	sensors = sensor_flags;
	if (sensor) {
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
		sensor->Open();
		sensor->OpenMultiSourceFrameReader(source_types, &multi_reader);
		if (multi_reader) {
			return true;
		}
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
		IInfraredFrame * frame_ir;
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