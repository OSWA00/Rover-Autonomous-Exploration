cmake_minimum_required(VERSION 3.0.2)
project(rover_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

find_package(catkin REQUIRED 
COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_srvs
  roslint
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    std_srvs
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# add_library(${PROJECT_NAME}_core
# )
#target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_11)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
  src/RoverControl.cpp
)
#target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

# add_dependencies(${PROJECT_NAME}_core
#   ${catkin_EXPORTED_TARGETS}
# )

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# target_link_libraries(${PROJECT_NAME}_core
#   ${catkin_LIBRARIES}
# )

target_link_libraries(${PROJECT_NAME}
  # ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
# install(
#   TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
# )

roslint_cpp()
