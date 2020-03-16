#ifndef EVENTS_H
#define EVENTS_H

class BaseTaskMessage {
public:
	virtual void executeOnThisThread() = 0;
};

#endif // EVENTS_H