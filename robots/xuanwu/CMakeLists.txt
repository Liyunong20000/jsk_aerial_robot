cmake_minimum_required(VERSION 3.0.2)
project(xuanwu)

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros_control
  roscpp
  sensor_msgs
  spinal
  )

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model
)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

# Build xuanwu_joy_gripper_node
add_executable(xuanwu_joy_gripper_node src/xuanwu_joy_gripper_node.cpp)
target_link_libraries(xuanwu_joy_gripper_node ${catkin_LIBRARIES})
add_dependencies(xuanwu_joy_gripper_node ${catkin_EXPORTED_TARGETS})

# Install executable
install(TARGETS xuanwu_joy_gripper_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##mujoco_model_convert(
#  ${PROJECT_SOURCE_DIR}
##  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

##if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_subdirectory(test)
##endif()
