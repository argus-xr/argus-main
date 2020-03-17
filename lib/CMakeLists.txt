cmake_minimum_required(VERSION 3.13)  # CMake version check

target_include_directories(argus-main PUBLIC kissnet)

add_subdirectory(argus-utils/argus-netbuffer)
target_link_libraries(argus-main argus-libnetbuffer)
target_include_directories(argus-main PUBLIC argus-utils)

target_include_directories(argus-main PUBLIC ./)