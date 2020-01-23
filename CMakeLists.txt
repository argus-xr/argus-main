cmake_minimum_required(VERSION 3.13)  # CMake version check
project(argus-main)               # Create project "argus-main"
set(CMAKE_CXX_STANDARD 14)            # Enable c++14 standard
add_executable(argus-main "")

set(DEPS_FOLDER_RELEASE ${CMAKE_SOURCE_DIR}/compiledlibs/release)
set(DEPS_FOLDER_DEBUG ${CMAKE_SOURCE_DIR}/compiledlibs/debug)

#boost
include_directories(${CMAKE_SOURCE_DIR}/lib)
set(Boost_DEBUG 1)
find_package(Boost 1.70 REQUIRED COMPONENTS system)
if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIR})
	target_link_libraries(argus-main Boost::system)
endif()

add_subdirectory(src)