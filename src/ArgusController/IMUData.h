#ifndef IMUDATA_H
#define IMUDATA_H

#include <stdint.h>

struct IMUData {
	float aX, aY, aZ;
	float gX, gY, gZ;
	uint64_t timestamp_us; // microseconds
};

#endif // IMUDATA_H
