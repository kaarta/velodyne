cmake_minimum_required(VERSION 2.8.3)
project(velodyne_launch)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  velodyne_pointcloud
  message_generation
  message_runtime
  roscpp
  rosbag
  tf
)

# Find system dependencies
find_package(Boost REQUIRED COMPONENTS system thread system filesystem)

# Generate added messages and services
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# Specify additional locations of header files
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

# Mark executables and/or libraries
install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)
install(FILES Copying
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
