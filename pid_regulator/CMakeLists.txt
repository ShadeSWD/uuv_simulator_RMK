cmake_minimum_required(VERSION 2.8.3)
project(pid_regulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ddynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/PidParameters.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(pid src/pid.cpp)

catkin_install_python(PROGRAMS src/uuv_pid_controller.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(pid
    ${catkin_LIBRARIES}
)



