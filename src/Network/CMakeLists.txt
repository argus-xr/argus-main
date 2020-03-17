target_sources(argus-main PRIVATE ConnectionTCP.cpp ConnectionTCP.h ServerTCP.cpp ServerTCP.h)
target_include_directories(argus-main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})