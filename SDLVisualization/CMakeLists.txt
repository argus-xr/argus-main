cmake_minimum_required(VERSION 3.13)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/") # for FindSDL2.cmake.

find_package(SDL2 COMPONENTS SDL2)
find_package(SDL2_image)
if(SDL2_FOUND)
	target_compile_definitions(argus-main PRIVATE SDL_FOUND)
	target_link_libraries(argus-main SDL2::SDL2)

	include_directories(${SDL2_INCLUDE_DIRS})
	
	target_sources(argus-main PRIVATE include/SDLUI.h src/SDLUI.cpp)
	target_include_directories(argus-main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

if(SDL2_IMAGE_FOUND)
	target_compile_definitions(argus-main PRIVATE SDL2_IMAGE_FOUND)
	target_link_libraries(argus-main ${SDL2_IMAGE_LIBRARIES})
	target_include_directories(argus-main PRIVATE ${SDL2_IMAGE_INCLUDE_DIRS})
endif()