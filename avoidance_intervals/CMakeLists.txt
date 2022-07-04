cmake_minimum_required(VERSION 2.8.3)
project(avoidance_intervals)
add_compile_options(-std=c++14 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
#add_compile_options(-std=c++11)
#set(CMAKE_BUILD_TYPE Debug)

message(WARNING "find packages")
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
  velocity_scaling_iso15066
  std_msgs
  human_motion_prediction
)

# message(WARNING "find human motion predicition")
# find_package(human_motion_prediction REQUIRED)

find_package(fcl REQUIRED)
find_package(octomap REQUIRED)

message(WARNING "catkin packages")
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES avoidance_intervals
  CATKIN_DEPENDS moveit_core moveit_ros_planning roscpp velocity_scaling_iso15066 std_msgs human_motion_prediction
  DEPENDS FCL OCTOMAP
)
message(WARNING "include stuff")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/avoidance_model.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${FCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )
  
message(WARNING "adding test_model exe")

add_executable(test_model src/test_model.cpp)
message(WARNING "linking test_model exe")
target_link_libraries(test_model
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
message(WARNING "depending test_model exe")
add_dependencies(test_model ${catkin_EXPORTED_TARGETS})
install(TARGETS test_model DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dirrt_star.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
