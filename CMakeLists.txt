cmake_minimum_required(VERSION 3.13)  # CMake version check
project(argus-main)                   # Create project "argus-main"
set(CMAKE_CXX_STANDARD 17)            # Enable c++17 standard
add_executable(argus-main "")

set(DEPS_FOLDER_RELEASE ${CMAKE_SOURCE_DIR}/compiledlibs/release)
set(DEPS_FOLDER_DEBUG ${CMAKE_SOURCE_DIR}/compiledlibs/debug)

target_include_directories(argus-main PUBLIC lib/kissnet)

add_subdirectory(lib/argus-utils/argus-netbuffer)
target_link_libraries(argus-main argus-libnetbuffer)
target_include_directories(argus-main PUBLIC lib/argus-utils)

add_subdirectory(src)