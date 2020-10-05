#ifndef IMUDATA_H
#define IMUDATA_H

#include <stdint.h>

struct IMUData {
	int16_t aX, aY, aZ;
	int16_t gX, gY, gZ;
	uint64_t timestamp_us; // microseconds
};

#endif // IMUDATA_H
