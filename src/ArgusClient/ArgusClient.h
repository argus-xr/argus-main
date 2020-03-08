#ifndef ARGUSCLIENT_H
#define ARGUSCLIENT_H

#include "ArgusViewer.h"
#include "ArgusController.h"

class ArgusClient {
	ArgusViewer* viewer = nullptr;
	ArgusController* controller = nullptr;
};

#endif // ARGUSCLIENT_H