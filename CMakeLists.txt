cmake_minimum_required(VERSION 3.13)  # CMake version check
project(argus-server)                   # Create project "argus-main"
set(CMAKE_CXX_STANDARD 17)            # Enable c++17 standard
add_executable(argus-main "")

set(DEPS_FOLDER_RELEASE ${CMAKE_SOURCE_DIR}/compiledlibs/release)
set(DEPS_FOLDER_DEBUG ${CMAKE_SOURCE_DIR}/compiledlibs/debug)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")

add_subdirectory(lib)

add_subdirectory(${CMAKE_SOURCE_DIR}/SDLVisualization)

add_subdirectory(src)

find_package(pugixml)

target_link_libraries(argus-main pugixml::pugixml)