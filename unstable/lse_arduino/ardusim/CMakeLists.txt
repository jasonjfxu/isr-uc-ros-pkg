cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(ardusim src/Ardusim.cpp)

rosbuild_add_executable(gp_node src/general_purpose.cpp src/Ardusim.cpp)
target_link_libraries(gp_node cereal_port)

rosbuild_add_executable(nose_node src/nose.cpp src/Ardusim.cpp)
target_link_libraries(nose_node cereal_port)

rosbuild_add_executable(roomba_sonar_node src/roomba_sonar.cpp src/Ardusim.cpp)
target_link_libraries(roomba_sonar_node cereal_port)

rosbuild_add_executable(thermistor_node src/thermistor.cpp src/Ardusim.cpp)
target_link_libraries(thermistor_node cereal_port)

rosbuild_add_executable(anemometer_node src/anemometer.cpp src/Ardusim.cpp)
target_link_libraries(anemometer_node cereal_port)

rosbuild_add_executable(nose_calibration_node src/nose_calibration.cpp)
target_link_libraries(nose_calibration_node cereal_port)

