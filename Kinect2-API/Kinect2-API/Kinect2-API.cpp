#include "stdafx.h"
#include <Kinect.h>
#include <cstring>
#include <iostream>


#define EXPORTFUNC extern "C" __declspec(dllexport)


#define F_SENSOR_COLOR  0x00000001
#define F_SENSOR_DEPTH  0x00000010
#define F_SENSOR_IR     0x00000100
#define F_SENSOR_AUDIO  0x00001000
#define COLOR_WIDTH     1920
#define COLOR_HEIGHT    1080
#define COLOR_CHANNELS  4
#define DEPTH_WIDTH     512
#define DEPTH_HEIGHT    424
#define IR_WIDTH        512
#define IR_HEIGHT       424
#define MAX_SUBFRAMES   8
#define SUBFRAME_SIZE   256


int sensors = 0;
IKinectSensor* sensor;
IMultiSourceFrameReader* multi_reader;
IAudioSource* audio_source;
IAudioBeamFrameReader* audio_reader;

UINT8 buffer_color[COLOR_WIDTH * COLOR_HEIGHT * COLOR_CHANNELS];
UINT16 buffer_depth[DEPTH_WIDTH * DEPTH_HEIGHT];
UINT16 buffer_ir[IR_WIDTH * IR_HEIGHT];
FLOAT buffer_audio[MAX_SUBFRAMES * (SUBFRAME_SIZE + 2)];
UINT32 buffer_audio_size = 0;


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
			sensor->get_AudioSource(&audio_source);
			audio_source->OpenReader(&audio_reader);
		}
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


void process_audio_subframe(IAudioBeamSubFrame* subframe, int index) {
	float* audio_buf = NULL;
	float beam_angle;
	float beam_conf;
	subframe->get_BeamAngle(&beam_angle);
	subframe->get_BeamAngleConfidence(&beam_conf);
	UINT buf_size = 0;
	subframe->AccessUnderlyingBuffer(&buf_size, (BYTE **)&audio_buf);
	buffer_audio[index * SUBFRAME_SIZE] = beam_angle;
	buffer_audio[index * SUBFRAME_SIZE + 1] = beam_angle;
	memcpy(buffer_audio + index * SUBFRAME_SIZE + 2, audio_buf, SUBFRAME_SIZE * sizeof(FLOAT));
}


EXPORTFUNC bool read_audio_sensors() {
	if (sensors & F_SENSOR_AUDIO) {
		IAudioBeamFrameList* audio_frames;
		IAudioBeamFrame* audio_frame;
		UINT32 subframe_count;
		audio_reader->AcquireLatestBeamFrames(&audio_frames);
		if (!audio_frames) {
			return false;
		}
		audio_frames->OpenAudioBeamFrame(0, &audio_frame);
		audio_frame->get_SubFrameCount(&subframe_count);
		for (UINT32 i = 0; i < subframe_count; i++) {
			IAudioBeamSubFrame* subframe = NULL;
			audio_frame->GetSubFrame(i, &subframe);
			process_audio_subframe(subframe, i);
			subframe->Release();
		}
		buffer_audio_size = subframe_count;
		audio_frames->Release();
		audio_frame->Release();
		return true;
	}
	return false;
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

EXPORTFUNC bool get_audio_data(FLOAT* array, bool do_read) {
	if (do_read) read_audio_sensors();
	array[0] = buffer_audio_size;
	memcpy(array + 1, buffer_audio, (1 + MAX_SUBFRAMES * (SUBFRAME_SIZE + 2)) * sizeof(FLOAT));
	return true;
}