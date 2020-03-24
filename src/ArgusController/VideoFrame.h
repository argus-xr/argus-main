#ifndef VIDEOFRAME_H
#define VIDEOFRAME_H

#include <cstddef>
#include <cstdint>

class VideoFrame {
public:
	enum class VideoFrameEncoding {
		VFE_BMP,
		VFE_JPEG,
		VFE_NULL,
	};
	VideoFrame(uint8_t* buf, size_t length, VideoFrameEncoding frameEncoding);
	~VideoFrame();
	uint8_t* rawBuffer = NULL;
	size_t bufferSize = 0;
	VideoFrameEncoding encoding = VideoFrameEncoding::VFE_NULL;
};

#endif // VIDEOFRAME_H
