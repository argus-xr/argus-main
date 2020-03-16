#ifndef ARGUSEVENTQUEUE_H
#define ARGUSEVENTQUEUE_H

#include <mutex>
#include <Events.h>
#include <queue>

class ArgusEventQueue { // simple naive queue. Won't be fast, but it'll work until we replace it. Should be thread-safe.
public:
	void addEvent(BaseTaskMessage* ev);
	bool executeEventOnThread();
	void executePendingEvents();
protected:
	std::mutex eventBusMutex;
	std::queue<BaseTaskMessage*> eventQueue;
};

#endif // ARGUSEVENTQUEUE_H