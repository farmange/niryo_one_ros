cmake_minimum_required(VERSION 2.8.3)
project(orthopus_space_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  niryo_one_msgs
  roscpp
  std_msgs
  moveit_ros_planning_interface
  qpoases_ros
  spacenav_node
  joy
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

############ device xbox 
add_executable(device_xbox
  src/input_device/device.cpp
  src/input_device/device_xbox.cpp
)
target_link_libraries(device_xbox ${catkin_LIBRARIES})
add_dependencies(device_xbox ${catkin_EXPORTED_TARGETS})

############ device spacenav 
add_executable(device_spacenav
  src/input_device/device.cpp
  src/input_device/device_spacenav.cpp
)
target_link_libraries(device_spacenav ${catkin_LIBRARIES})
add_dependencies(device_spacenav ${catkin_EXPORTED_TARGETS})

############ device web app
add_executable(device_web_app
  src/input_device/device.cpp
  src/input_device/device_web_app.cpp
)
target_link_libraries(device_web_app ${catkin_LIBRARIES})
add_dependencies(device_web_app ${catkin_EXPORTED_TARGETS})

############ cartesian controller
add_executable(cartesian_controller
  src/inverse_kinematic.cpp
  src/forward_kinematic.cpp
  src/velocity_integrator.cpp
  src/trajectory_controller.cpp
  src/pose_manager.cpp
  src/robot_manager.cpp
  src/cartesian_controller.cpp
  src/types/space_base.cpp
  
)
target_link_libraries(cartesian_controller ${catkin_LIBRARIES})
add_dependencies(cartesian_controller ${catkin_EXPORTED_TARGETS})

