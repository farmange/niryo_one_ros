cmake_minimum_required(VERSION 2.8.3)
project(orthopus_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  niryo_one_msgs
  roscpp
  std_msgs
#   control_msgs
  moveit_ros_planning_interface
  qpoases_ros
  actionlib

)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS
#     control_msgs
    moveit_ros_planning_interface
      qpoases_ros
  DEPENDS
    EIGEN3
    )
    
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

############ joystick_cartesian_interface
add_executable(joystick_cartesian_interface
  src/joystick_cartesian_interface.cpp
)
target_link_libraries(joystick_cartesian_interface ${catkin_LIBRARIES})
add_dependencies(joystick_cartesian_interface ${catkin_EXPORTED_TARGETS})

############ xbox_to_twist
add_executable(xbox_to_twist
  src/xbox_to_twist.cpp
)
target_link_libraries(xbox_to_twist ${catkin_LIBRARIES})
add_dependencies(xbox_to_twist ${catkin_EXPORTED_TARGETS})

