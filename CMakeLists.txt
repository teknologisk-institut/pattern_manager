cmake_minimum_required(VERSION 2.8.3)
project(pattern_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
  rostest
)

catkin_python_setup()

add_message_files(
   FILES
   GroupDeps.msg
 )

add_service_files(
   FILES
   ElementName.srv
   ElementIterator.srv
   PatternTypes.srv
   CreateGroup.srv
   CreatePattern.srv
   GroupTree.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
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
catkin_add_nosetests(test/test_group.py)
catkin_add_nosetests(test/test_manager.py)
catkin_add_nosetests(test/test_pattern_plugins.py)
catkin_add_nosetests(test/test_patterns.py)
