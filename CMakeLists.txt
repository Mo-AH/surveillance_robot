## Init project.
cmake_minimum_required(VERSION 2.8.3)
project(surveillance_robot)

## Find dependences.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  actionlib_msgs
  message_generation
  armor
  armor_py_api
  smach
)

## Uncomment if the package has a setup.py
catkin_python_setup()

## Generate messages in the 'msg' folder.
add_message_files(
  FILES
  Point.msg
  DoorConnection.msg
)

## Generate services in the 'srv' folder.
add_service_files(
  FILES
  GetPose.srv
  SetPose.srv
)

## Generate actions in the 'action' folder.
add_action_files(
  FILES
  Control.action
  Plan.action
)

## Generate added messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

## The catkin_package macro generates cmake config files for your package.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES surveillance_robot
   CATKIN_DEPENDS message_runtime # rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Define runnables.
catkin_install_python(
  PROGRAMS 
    scripts/robot_state.py
    scripts/smach_robot.py
    scripts/map_builder.py
    scripts/planner.py
    scripts/controller.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
