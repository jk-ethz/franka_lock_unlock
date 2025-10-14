cmake_minimum_required(VERSION 3.0.2)
project(franka_lock_unlock)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

# catkin-compatible entry points
catkin_install_python(PROGRAMS
  scripts/franka_lock_unlock
  scripts/franka_shutdown
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
