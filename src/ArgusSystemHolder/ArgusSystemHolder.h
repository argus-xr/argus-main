#ifndef ARGUSSYSTEMHOLDER_H
#define ARGUSSYSTEMHOLDER_H

// forward declarations to avoid #including them in here. Reduces build times.
class ArgusDeviceManager;
class Server;
#ifdef SDL_FOUND
class ArgusVizUI;
#endif

class ArgusSystemHolder { // holds references to important systems, and initializes them.
public:
	void startArgus();

	ArgusDeviceManager* getDeviceManager();
	bool deviceManagerReady();

	Server* getServer();
	bool serverReady();

#ifdef SDL_FOUND
	ArgusVizUI* getUIManager();
	bool UIManagerReady();
#endif

	void pollSystems(); // Might want to put this in a thread of its own at some point?

private:
	void startDeviceManager();
	ArgusDeviceManager* deviceManager = nullptr;
	bool deviceManagerIsReady = false;

	void startServer();
	Server* server;
	bool serverIsReady = false;

#ifdef SDL_FOUND
	void startUIManager();
	ArgusVizUI* UIManager = nullptr;
	bool UIManagerIsReady = false;
#endif
};

#endif // ARGUSSYSTEMHOLDER_H