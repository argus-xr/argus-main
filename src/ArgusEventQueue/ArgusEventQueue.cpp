#include "ArgusEventQueue.h"

void ArgusEventQueue::addEvent(BaseTaskMessage* ev) {
	if (ev == nullptr) {
		return;
	}
	std::lock_guard<std::mutex> guard(eventBusMutex);
	eventQueue.push(ev);
}

bool ArgusEventQueue::executeEventOnThread() {
	std::lock_guard<std::mutex> guard(eventBusMutex);
	if (eventQueue.empty()) {
		return false;
	}
	eventQueue.front()->executeOnThisThread();
	eventQueue.pop();
	return true;
}

void ArgusEventQueue::executePendingEvents() {
	std::lock_guard<std::mutex> guard(eventBusMutex);
	while (!eventQueue.empty()) {
		eventQueue.front()->executeOnThisThread();
		eventQueue.pop();
	}
}