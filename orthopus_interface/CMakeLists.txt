cmake_minimum_required(VERSION 2.8.3)
project(orthopus_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  niryo_one_msgs
  roscpp
  std_msgs
  moveit_ros_planning_interface
  qpoases_ros
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

############ web app wrapper
add_executable(web_app_wrapper
  src/device_wrapper.cpp
  src/web_app_wrapper.cpp
)
target_link_libraries(web_app_wrapper ${catkin_LIBRARIES})
add_dependencies(web_app_wrapper ${catkin_EXPORTED_TARGETS})

############ cartesian controller
add_executable(cartesian_controller
  src/inverse_kinematic.cpp
  src/forward_kinematic.cpp
  src/velocity_integrator.cpp
  src/constraints_compensator.cpp
  src/trajectory_controller.cpp
  src/pose_manager.cpp
  src/robot_manager.cpp
  src/cartesian_controller.cpp
  
  src/fsm/state_disable.cpp
  src/fsm/state_idle.cpp
  src/fsm/state_joint_trajectory.cpp
  src/fsm/state_space_control.cpp
  src/fsm/state_space_trajectory.cpp
)
target_link_libraries(cartesian_controller ${catkin_LIBRARIES})
add_dependencies(cartesian_controller ${catkin_EXPORTED_TARGETS})

