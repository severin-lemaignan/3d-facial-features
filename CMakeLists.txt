cmake_minimum_required(VERSION 2.8.3)
project(3d_face_features)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        tf 
        std_msgs
        sensor_msgs
        cv_bridge 
        image_transport 
        image_geometry 
        nodelet)

## System dependencies
find_package(OpenCV REQUIRED)

find_package(dlib REQUIRED)
include_directories(${dlib_INCLUDE_DIRS})

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
if(2.9.0 VERSION_GREATER ${OpenCV_VERSION})
    message(FATAL "OpenCV3 required!")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_markers
  CATKIN_DEPENDS 
    tf
    nodelet
  DEPENDS OpenCV
  LIBRARIES 
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${dlib_LIBRARIES}
    )

if(TRUE)
    add_definitions(-DHEAD_POSE_ESTIMATION_DEBUG)
endif()

add_executable(run src/main.cpp src/head_pose_estimation.cpp)
target_link_libraries(run ${LIBS})

#add_library(markers_nodelet src/chilitagsdetector.cpp src/nodelet.cpp)
#add_dependencies(markers_nodelet ${catkin_EXPORTED_TARGETS})
#target_link_libraries(markers_nodelet ${LIBS})


#############
## Install ##
#############

install(TARGETS run
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/3d_facial_features.launch
    launch/run_gscam.launch
#    launch/gscam_nodelet_pipeline.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(FILES
#    config/markers_configuration_sample.yml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#)

#install(FILES
#    nodelet_markers.xml
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

