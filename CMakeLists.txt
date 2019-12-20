cmake_minimum_required(VERSION 2.8.3)
project(pattern_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
  rostest
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
   FILES
   Params.msg
   Point.msg
 )

add_service_files(
   FILES
   CreateTransform.srv
   TransformId.srv
   SetActive.srv
   GetCurrentId.srv
   GetIds.srv
   CreateLinearPattern.srv
   CreateRectangularPattern.srv
   CreateCircularPattern.srv
   CreateScatterPattern.srv
   SetIterationOrder.srv
   GetTransformParams.srv
   SetParent.srv
   Filename.srv
   GetTransformId.srv
   UpdateTransform.srv
   GetTransforms.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pattern_manager
#  CATKIN_DEPENDS geometry_msgs rospy std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
catkin_add_nosetests(test/test_xform.py)
catkin_add_nosetests(test/test_plugin.py)
catkin_add_nosetests(test/test_pattern.py)
catkin_add_nosetests(test/test_util.py)
