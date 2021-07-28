cmake_minimum_required(VERSION 2.8.3)
project(wait_action)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  rospy
)

catkin_python_setup()

# Generate actions in the 'action' folder
add_action_files(
  FILES
  Wait.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wait_action
#  CATKIN_DEPENDS actionlib actionlib_msgs rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)

catkin_install_python(PROGRAMS
  scripts/wait_node.py
  scripts/wait_node_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)