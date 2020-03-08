#ifndef SDL_UI_H
#define SDL_UI_H

#include <SDL.h>

class ArgusVizUI {
public:
	void start();
	void registerForEvents();
	void poll();
	void stop();
protected:
	SDL_Window* window = NULL;
	SDL_Surface* screenSurface = NULL;
};

#endif // SDL_UI_H