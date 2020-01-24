cmake_minimum_required(VERSION 2.8.3)
project(mecademic_robot_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  catkin_virtualenv
  mecademic_pydriver
  mecademic_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy 
  std_msgs
  geometry_msgs
  sensor_msgs
  mecademic_msgs
)

catkin_generate_virtualenv(
  # Select an alternative version of the python interpreter - it must be installed on the system. Minor version is optional.
  PYTHON_VERSION 3.6  # Default 2

  # Choose not to use underlying system packages. This excludes any python packages installed by apt or system-pip from the environment.
  USE_SYSTEM_PACKAGES TRUE  # Default TRUE

  # Disable including pip requirements from catkin dependencies of this package.
  ISOLATE_REQUIREMENTS FALSE  # Default FALSE

  # Provide extra arguments to the underlying pip invocation
  #EXTRA_PIP_ARGS
  #  --no-binary=:all:
  #  -vvv
)

catkin_install_python(
  PROGRAMS 
    src/mecademic_robot_driver/mecademic_robot_driver.py
    src/mecademic_robot_driver/mecademic_robot_feedback.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(FILES requirements.txt
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



