cmake_minimum_required(VERSION 3.13)  # CMake version check
project(argus-main)               # Create project "argus-main"
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard

set(DEPS_FOLDER_RELEASE ${CMAKE_SOURCE_DIR}/compiledlibs/release)
set(DEPS_FOLDER_DEBUG ${CMAKE_SOURCE_DIR}/compiledlibs/debug)

#plibsys
include_directories(${DEPS_FOLDER_RELEASE}/plibsys) #plibsysconfig.h
include_directories(lib/plibsys)
set(LINK_FILES_RELEASE ${DEPS_FOLDER_RELEASE}/plibsys/plibsys.lib)
set(LINK_FILES_DEBUG ${DEPS_FOLDER_DEBUG}/plibsys/plibsys.lib)

# Add main.cpp file of project root directory as source file
set(SOURCE_FILES main.cpp)

include_directories(lib)
link_directories(lib)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(argus-main ${SOURCE_FILES})
target_link_libraries(argus-main optimized ${LINK_FILES_RELEASE})
target_link_libraries(argus-main debug ${LINK_FILES_DEBUG})