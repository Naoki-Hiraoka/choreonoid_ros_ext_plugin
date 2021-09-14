cmake_minimum_required(VERSION 3.0.2)
project(choreonoid17_ros_plugin)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED roscpp)
find_package(choreonoid REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

# 相対パスを絶対パスに直す
set(CHOREONOID_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_LIBRARIES_ABS ${CHOREONOID_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

add_library(CnoidV17ROSPlugin SHARED
  src/CnoidV17ROSPlugin.cpp
  src/ClockPublisherItem.cpp
  )
target_link_libraries(CnoidV17ROSPlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_ABS}
  )

set_target_properties(CnoidV17ROSPlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})

