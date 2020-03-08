#include "VideoFrame.h"

VideoFrame::VideoFrame(uint8_t* buf, size_t length, VideoFrameEncoding frameEncoding) {
	rawBuffer = buf;
	bufferSize = length;
	encoding = frameEncoding;
}

VideoFrame::~VideoFrame() {
	delete[] rawBuffer;
}