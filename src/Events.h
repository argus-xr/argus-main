#ifndef EVENTS_H
#define EVENTS_H

#include "eventbus/EventBus.h"

class Events {
public:
	static Dexode::EventBus eBus;
private:
	static Events instance;
	Events();
};

#endif // EVENTS_H