#include "SDLUI.h"

#include <stdio.h>
#include "Events.h"
#include "ControllerEvents.h"

#ifdef SDL2_IMAGE_FOUND
#include "SDL_image.h"
#endif

void ArgusVizUI::start() {
	instance = this; // fugly singleton hack to get stuff working on monday.
	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
		return;
	}
	window = SDL_CreateWindow("Argus Server", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL_WINDOW_SHOWN);
	if (window == NULL) {
		printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
		return;
	}

#ifdef SDL2_IMAGE_FOUND
	if (!IMG_Init(IMG_INIT_JPG)) {
		printf("Could not initialize SDL2_image! IMG_Error: %s\n", IMG_GetError());
		SDL_DestroyWindow(window);
		SDL_Quit();
		return;
	}
#endif

	screenSurface = SDL_GetWindowSurface(window);

	SDL_FillRect(screenSurface, NULL, SDL_MapRGB(screenSurface->format, 0xFF, 0xFF, 0xFF));

	SDL_UpdateWindowSurface(window);
}

void ArgusVizUI::poll() {
	SDL_Event e;
	while (SDL_PollEvent(&e) != 0) {
		if (e.type == SDL_QUIT) {
			stop();
		}
	}
}

void ArgusVizUI::stop() {
	SDL_DestroyWindow(window);

	SDL_Quit();
}

void ArgusVizUI::setNewFrame(std::shared_ptr<VideoFrame> frame) {
	if (frameSurface != NULL) {
		SDL_FreeSurface(frameSurface);
		frameSurface = NULL;
	}
	SDL_RWops* ops = SDL_RWFromMem(frame->rawBuffer, frame->bufferSize);
	if (frame->encoding == VideoFrame::VideoFrameEncoding::VFE_BMP) {
		frameSurface = SDL_LoadBMP_RW(ops, 1);
	}
#ifdef SDL2_IMAGE_FOUND
	else {
		frameSurface = IMG_LoadJPG_RW(ops);
		//frameSurface = IMG_Load_RW(ops, 1);
		//frameSurface = IMG_Load("D:\Argus\Server\argus-main-build\Debug\BLAH.jpg");
		SDL_RWclose(ops);
	}
#endif

	if (frameSurface != NULL) {
		SDL_BlitSurface(frameSurface, NULL, screenSurface, NULL);
		SDL_UpdateWindowSurface(window);
	}
}

ArgusVizUI* ArgusVizUI::inst() {
	return instance;
}

ArgusVizUI* ArgusVizUI::instance;