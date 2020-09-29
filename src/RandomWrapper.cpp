#include "RandomWrapper.h"

void RandomWrapper::init() {
	if (!initialized) {
		generator.seed((unsigned)time(0));
		uint64_dist = std::uniform_int_distribution<unsigned long long>(
			std::numeric_limits<std::uint64_t>::min(),
			std::numeric_limits<std::uint64_t>::max()
		);;
	}
}

uint64_t RandomWrapper::getRandomUInt64() {
	init();
	return uint64_dist(generator);
}

bool RandomWrapper::initialized = false;
std::default_random_engine RandomWrapper::generator;
std::uniform_int_distribution<unsigned long long> RandomWrapper::uint64_dist;