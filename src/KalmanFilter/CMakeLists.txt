target_sources(argus-main PRIVATE KalmanFilter.hpp SystemModel.hpp OrientationMeasurementModel.hpp PositionMeasurementModel.hpp AccelerationMeasurementModel.hpp)
target_include_directories(argus-main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
find_package(kalman REQUIRED)
find_package(eigen3 REQUIRED)

target_include_directories(argus-main PRIVATE ${KALMAN_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
add_definitions( ${EIGEN3_DEFINITIONS} )