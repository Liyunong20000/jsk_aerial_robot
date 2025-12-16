cmake_minimum_required(VERSION 3.0.2)
project(xuanwu)

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros_control
  roscpp
  geometry_msgs
  nav_msgs
  gazebo_msgs
  apriltag_ros
  tf
  dynamic_reconfigure
  aerial_robot_msgs
)

find_package(Eigen3 REQUIRED)

# autodiff requires C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# path to the autodiff checkout inside this package (autodiff is header only)
set(AUTODIFF_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/autodiff)

# Settings for qpOASES
# Compiles qpOASES along with your project
set(QPOASES_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES)
# Check if directory exists to avoid build errors if you forgot Step 1
if(EXISTS "${QPOASES_ROOT}/CMakeLists.txt")
    add_subdirectory(${QPOASES_ROOT})
else()
    message(FATAL_ERROR "qpOASES not found in third_party/qpOASES")
endif()

# MUST come before catkin_package()
generate_dynamic_reconfigure_options(
  cfg/Eskf.cfg
  cfg/Planner.cfg
)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model
  INCLUDE_DIRS ${AUTODIFF_ROOT} #${QPOASES_ROOT}/include # export headers
)

# === include the headers for external libraries ===
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${AUTODIFF_ROOT}
  ${EIGEN3_INCLUDE_DIRS}
  ${QPOASES_ROOT}/include
)

add_executable(noisy_odom src/noisy_odom.cpp)
target_link_libraries(noisy_odom ${catkin_LIBRARIES})
add_dependencies(noisy_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(eskf_node src/eskf_node.cpp)
target_link_libraries(eskf_node ${catkin_LIBRARIES})
add_dependencies(eskf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_logger src/odom_logger.cpp)
target_link_libraries(odom_logger ${catkin_LIBRARIES})
add_dependencies(odom_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(mpc_planner_node src/planner_node.cpp)
target_link_libraries(mpc_planner_node
	${catkin_LIBRARIES}
	qpOASES
)
add_dependencies(mpc_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

##mujoco_model_convert(
#  ${PROJECT_SOURCE_DIR}
##  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

##if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_subdirectory(test)
##endif()

