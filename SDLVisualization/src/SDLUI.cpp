#include "SDLUI.h"

#include <stdio.h>
#include "Events.h"
#include "ControllerEvents.h"

void ArgusVizUI::start() {
	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
		return;
	}
	window = SDL_CreateWindow("Argus Server", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 500, 500, SDL_WINDOW_SHOWN);
	if (window == NULL) {
		printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
		return;
	}
	screenSurface = SDL_GetWindowSurface(window);

	SDL_FillRect(screenSurface, NULL, SDL_MapRGB(screenSurface->format, 0xFF, 0xFF, 0xFF));

	SDL_UpdateWindowSurface(window);

	registerForEvents();
}

void ArgusVizUI::registerForEvents() {
	Events::eBus.listen<NewVideoFrameEvent>([](const NewVideoFrameEvent& event) {
		printf("New VideoFrame event received!\n");
		});
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