cmake_minimum_required(VERSION 2.8.3)
project(moveit_trayectorias)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  include
)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(pruebas_precision src/nodo_precision.cpp src/MyRobot.cpp)
target_link_libraries(pruebas_precision ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pruebas_precision DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(trayectorias src/nodo_trayectorias.cpp src/PuertaTrayectorias.cpp src/MyRobot.cpp)
target_link_libraries(trayectorias ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS trayectorias DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



