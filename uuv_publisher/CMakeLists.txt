cmake_minimum_required(VERSION 2.8.3)
project(uuv_publisher)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

catkin_install_python(PROGRAMS scripts/uuv_data_publisher.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
