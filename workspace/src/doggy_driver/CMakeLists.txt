cmake_minimum_required(VERSION 2.8.3)
project(doggy_driver)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   scripts/doggy_driver.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

