#ifndef RANDOMWRAPPER_H
#define RANDOMWRAPPER_H

#include <random>
#include <ctime>
#include <type_traits>

class RandomWrapper {
public:
	static uint64_t getRandomUInt64();
protected:
	static std::default_random_engine generator;
	static std::uniform_int_distribution<unsigned long long> uint64_dist;
	static void init();
	static bool initialized;
};

#endif // RANDOMWRAPPER_H