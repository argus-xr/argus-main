#include "ArgusSystemHolder.h"

#ifdef SDL_FOUND
#undef main // SDL.h has #define main SDL_main which causes issues.
#endif // SDL_FOUND

Server* server;

int main(int argc, char* argv[]) {
    ArgusSystemHolder* system = new ArgusSystemHolder();
    system->startArgus();
    while (true) {
        system->pollSystems();
    }
    return 0;
}