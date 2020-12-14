cmake_minimum_required(VERSION 3.0.2)
project(ros_flir_spinnaker)

add_compile_options(-std=c++11 -Wall -Wextra -Wignored-qualifiers -Wreorder)

message(STATUS "================ Le Manchot =================")
message(STATUS "== Project | Le Manchot                    ==")
message(STATUS "== Author  | Parham Nooralishahi           ==")
message(STATUS "== Contact | parham.nooralishahi@gmail.com ==")
message(STATUS "== Organization | Laval University         ==")
message(STATUS "=============================================")

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  dynamic_reconfigure
  image_proc
  cv_bridge
  image_transport
  nodelet
  roscpp
  roslaunch
  sensor_msgs
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/phmSpinnaker.cfg
)

set(PHM_LIB phmspinnaker)

# If Spinnacker is already present, use the found version. If not, download it.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

find_package(Spinnaker REQUIRED)
if(NOT Spinnaker_FOUND)
  message(STATUS "libSpinnaker not found in system library path")
  include(cmake/DownloadSpinnaker.cmake)
  download_spinnaker(Spinnaker_LIBRARIES Spinnaker_INCLUDE_DIRS)
endif()

message(STATUS "libSpinnaker library: ${Spinnaker_LIBRARIES}")
message(STATUS "libSpinnaker include: ${Spinnaker_INCLUDE_DIRS}")

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(STATUS "OpenCV not found in system library path")
endif()

message(STATUS "OpenCV library: ${OpenCV_LIBRARIES}")
message(STATUS "OpenCV include: ${OpenCV_INCLUDE_DIRS}")

add_service_files(
  FILES
  ExecuteCommand.srv
  ExecuteOnOff.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros_flir_spinnaker
#  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_proc image_transport nodelet roscpp roslaunch sensor_msgs message_runtime
CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Spinnaker_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB PHM_INCLUDE
    "include/*.hpp"
    "include/*.h"
)

add_library(${PHM_LIB}
  src/SpinnakerCamera.cpp
  src/Camera.cpp
  src/A700.cpp
  ${PHM_INCLUDE}
)
target_link_libraries(${PHM_LIB} 
  ${Spinnaker_LIBRARIES}
  ${OpenCV_LIBS}
  pthread
)
add_dependencies(${PHM_LIB} ${PROJECT_NAME}_gencfg)

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node
  ${PHM_LIB}
  ${catkin_LIBRARIES}
)
add_dependencies(test_node ${PROJECT_NAME}_gencfg)

add_executable(flir_node src/flir_node.cpp)
target_link_libraries(flir_node
  ${PHM_LIB}
  ${catkin_LIBRARIES}
)
add_dependencies(flir_node ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_flir_spinnaker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
