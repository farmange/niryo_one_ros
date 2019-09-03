cmake_minimum_required(VERSION 2.8.3)
project(orthopus_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  niryo_one_msgs
  roscpp
  std_msgs
  moveit_ros_planning_interface
  qpoases_ros
  actionlib
  spacenav_node
  joy
  tf2_geometry_msgs
  rospy
  web_video_server
  libuvc_camera
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS
    moveit_ros_planning_interface
    qpoases_ros
  DEPENDS
    EIGEN3
    )
  
include_directories(
  include
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

############ xbox wrapper 
add_executable(xbox_wrapper
  src/device_wrapper.cpp
  src/xbox_wrapper.cpp
)
target_link_libraries(xbox_wrapper ${catkin_LIBRARIES})
add_dependencies(xbox_wrapper ${catkin_EXPORTED_TARGETS})

############ spacenav wrapper
add_executable(spacenav_wrapper
  src/device_wrapper.cpp
  src/spacenav_wrapper.cpp
)
target_link_libraries(spacenav_wrapper ${catkin_LIBRARIES})
add_dependencies(spacenav_wrapper ${catkin_EXPORTED_TARGETS})

############ spacenav wrapper
add_executable(web_app_wrapper
  src/device_wrapper.cpp
  src/web_app_wrapper.cpp
)
target_link_libraries(web_app_wrapper ${catkin_LIBRARIES})
add_dependencies(web_app_wrapper ${catkin_EXPORTED_TARGETS})

############ wrapper test
add_executable(cartesian_controller
  src/inverse_kinematic.cpp
  src/tool_controller.cpp
  src/cartesian_controller.cpp
)
target_link_libraries(cartesian_controller ${catkin_LIBRARIES})
add_dependencies(cartesian_controller ${catkin_EXPORTED_TARGETS})
