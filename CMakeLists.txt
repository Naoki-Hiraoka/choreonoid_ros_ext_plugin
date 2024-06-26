cmake_minimum_required(VERSION 3.0.2)
project(choreonoid_ros_ext_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport nav_msgs)
find_package(choreonoid REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_library(CnoidROSExtPlugin SHARED
  src/CnoidROSExtPlugin.cpp
  src/ClockPublisherItem.cpp
  src/ClockShmItem.cpp
  src/CraneItem.cpp
  src/CameraPublisherItem.cpp
  src/DepthCameraPublisherItem.cpp
  src/OdometryCameraPublisherItem.cpp
  src/OdometryPublisherItem.cpp
  src/SimulatorWorldResetItem.cpp
  src/PositionDraggerItem.cpp
  )
target_link_libraries(CnoidROSExtPlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )

set_target_properties(CnoidROSExtPlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})




add_library(SpringDamperController SHARED src/SpringDamperController.cpp)
target_link_libraries(SpringDamperController
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  )
set_target_properties(SpringDamperController PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR}/simplecontroller
  LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR}/simplecontroller
  PREFIX "")
install(TARGETS SpringDamperController
  RUNTIME DESTINATION ${CHOREONOID_PLUGIN_DIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
  LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR}/simplecontroller CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
