#include "ArgusSystemHolder.h"

#include "ArgusDeviceManager.h"

#include "tcp_server.h"

#ifdef SDL_FOUND
#include "SDLUI.h"
#undef main // SDL.h has #define main SDL_main which causes issues.
#endif // SDL_FOUND

void ArgusSystemHolder::startArgus() {
	startDeviceManager();
}

void ArgusSystemHolder::startDeviceManager() {
	deviceManager = new ArgusDeviceManager();
	deviceManagerIsReady = true;
}

void ArgusSystemHolder::pollSystems() {
	server->poll();
#ifdef SDL_FOUND
	UIManager->poll();
#endif // SDL_FOUND
}

ArgusDeviceManager* ArgusSystemHolder::getDeviceManager() {
	return deviceManager;
}

bool ArgusSystemHolder::deviceManagerReady() {
	return deviceManagerIsReady;
}

Server* ArgusSystemHolder::getServer() {
	return server;
}

bool ArgusSystemHolder::serverReady() {
	return serverIsReady;
}

void ArgusSystemHolder::startServer() {
	server = new Server();
	server->start();
}

#ifdef SDL_FOUND
ArgusVizUI* ArgusSystemHolder::getUIManager() {
	return UIManager;
}

bool ArgusSystemHolder::UIManagerReady() {
	return UIManagerIsReady;
}

void ArgusSystemHolder::startUIManager() {
	UIManager = new ArgusVizUI();
	UIManager->start();
	UIManagerIsReady = true;
}
#endif // SDL_FOUND