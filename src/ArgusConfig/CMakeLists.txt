target_sources(argus-main PRIVATE ArgusConfig.cpp ArgusConfig.h ControllerConfig.cpp ControllerConfig.h)
target_include_directories(argus-main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})