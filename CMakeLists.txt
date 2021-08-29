cmake_minimum_required(VERSION 3.0.2)
project(ece3091)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   CamData.msg
   DistanceData.msg
   MotorCmd.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service2.srv
# )

generate_messages(DEPENDENCIES std_msgs ${PROJECT_NAME})

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

