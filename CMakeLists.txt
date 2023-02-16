cmake_minimum_required(VERSION 2.8.3)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(image_transport_py)

find_package(
	catkin
	REQUIRED
	COMPONENTS
	rospy
	sensor_msgs
	std_msgs
)

catkin_python_setup()

file(GLOB python_scripts_DIR
	"src/*.py"
)

catkin_package(
	CATKIN_DEPENDS
)

catkin_install_python(
	PROGRAMS ${python_scripts_DIR}
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)