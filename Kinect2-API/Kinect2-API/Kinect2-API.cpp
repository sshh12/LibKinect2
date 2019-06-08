#include "stdafx.h"
#include <Kinect.h>
#include <cstring>
#define EXPORT extern "C" __declspec(dllexport)


EXPORT long long mysum(int n, int* array) {
	long long res = 0;
	for (int i = 0; i < n; ++i) {
		res += array[i];
	}
	return res;
}
#define width 1920
#define height 1080
IKinectSensor* sensor;         // Kinect sensor
IColorFrameReader* reader;     // Kinect color data source
IDepthFrameReader* dreader;     // Kinect depth data source
byte data[width * height * 4];
UINT16 ddata[512 * 424];

EXPORT int initKinect() {
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return false;
	}
	if (sensor) {
		sensor->Open();
		IColorFrameSource* framesource = NULL;
		sensor->get_ColorFrameSource(&framesource);
		framesource->OpenReader(&reader);
		if (framesource) {
			framesource->Release();
			framesource = NULL;
		}
		IDepthFrameSource* dframesource = NULL;
		sensor->get_DepthFrameSource(&dframesource);
		dframesource->OpenReader(&dreader);
		if (dframesource) {
			dframesource->Release();
			dframesource = NULL;
		}
		return true;
	} else {
		return false;
	}
}

EXPORT void getKinectData(byte* array) {
	IColorFrame* frame = NULL;
	if (SUCCEEDED(reader->AcquireLatestFrame(&frame))) {
		frame->CopyConvertedFrameDataToArray(width * height * 4, data, ColorImageFormat_Bgra);
		memcpy(array, data, width * height * 4);
	}
	if (frame) frame->Release();
}

EXPORT void getKinectDepthData(UINT16* array) {
	IDepthFrame* frame = NULL;
	if (SUCCEEDED(dreader->AcquireLatestFrame(&frame))) {
		frame->CopyFrameDataToArray(512 * 424, ddata);
		memcpy(array, ddata, 512 * 424 * 2);
	}
	if (frame) frame->Release();
}