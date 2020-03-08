#ifndef SDL_UI_H
#define SDL_UI_H

#include <SDL.h>
#include <memory>
#include "VideoFrame.h"

class ArgusVizUI {
public:
	void start();
	void poll();
	void stop();
	void setNewFrame(std::shared_ptr<VideoFrame> frame);
	static ArgusVizUI* inst();
protected:
	SDL_Window* window = NULL;
	SDL_Surface* screenSurface = NULL;
	SDL_Surface* frameSurface = NULL;
private:
	static ArgusVizUI* instance; // fugly hack; do not look
};

#endif // SDL_UI_H