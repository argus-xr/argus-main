add_definitions(-D_USE_MATH_DEFINES) #To avoid some issues with cmath, Eigen and Kalman.

target_sources(argus-main PRIVATE main.cpp Events.cpp Events.h RandomWrapper.cpp RandomWrapper.h)
target_include_directories(argus-main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(ArgusClient)
add_subdirectory(ArgusController)
add_subdirectory(ArgusConfig)
add_subdirectory(ArgusViewer)
add_subdirectory(Network)
add_subdirectory(KalmanFilter)
