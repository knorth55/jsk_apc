cmake_minimum_required(VERSION 2.8.3)
project(synthetic2d)

find_package(catkin REQUIRED
  dynamic_reconfigure
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(FILES
  IsTarget.srv
)
generate_messages(
  # DEPENDENCIES std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/MaskRCNNRelook.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grasp_prediction_arc2017
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY nodes/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)
install(DIRECTORY samples launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(launch/play_rosbag.launch)
#   roslaunch_add_file_check(samples/sample_fcn_object_segmentation.launch)
#   add_rostest(samples/sample_fcn_object_segmentation.launch ARGS gui:=false GPU:=-1)
# endif()
