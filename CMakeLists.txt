# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rosaria)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        nav_msgs
        geometry_msgs
        sensor_msgs
        tf
        dynamic_reconfigure)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(
        FILES
        BumperState.msg
)
#uncomment if you have defined services
#add_service_files(
#  FILES
#  # TODO: List your msg files here
#)

generate_dynamic_reconfigure_options(cfg/RosAria.cfg)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${ZBAR_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
find_package(Aria QUIET)
if (Aria_FOUND)
    if (EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
        add_definitions(-DADEPT_PKG)
    endif ()
    include_directories(${Aria_INCLUDE_DIRS})
    link_directories(${Aria_LIBRARY_DIRS})
else ()
    # The installation package provided by Adept doesn't follow Debian policies
    if (EXISTS "/usr/local/Aria/include/Aria.h")
        add_definitions(-DADEPT_PKG)
        include_directories(/usr/local/Aria/include)
        link_directories(/usr/local/Aria/lib)
    endif ()
endif ()

add_executable(RosAria RosAria.cpp LaserPublisher.cpp)
add_executable(tourGuide tourGuide.cpp StateDisplay.cpp StateDisplay.h QRDetect.cpp)
add_executable(behavior1 ObstacleAvoid.cpp)
add_executable(behavior2 locateOpenSpace.cpp)
add_executable(behavior3 driveStraight.cpp)
add_executable(behavior4 avoidPass.cpp)
add_executable(auctioner auctioner.cpp)
add_executable(allInOne allInOne.cpp)
add_dependencies(RosAria rosaria_gencfg)
add_dependencies(RosAria rosaria_gencpp)

target_link_libraries(RosAria ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)
target_link_libraries(tourGuide ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} zbar pthread)
target_link_libraries(behavior1 ${catkin_LIBRARIES})
target_link_libraries(behavior2 ${catkin_LIBRARIES})
target_link_libraries(behavior3 ${catkin_LIBRARIES})
target_link_libraries(behavior4 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(auctioner ${catkin_LIBRARIES})
target_link_libraries(allInOne ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
set_target_properties(RosAria PROPERTIES COMPILE_FLAGS "-fPIC")
set_target_properties(tourGuide PROPERTIES COMPILE_FLAGS "-fPIC")
#set_target_properties(RosAria PROPERTIES LINK_FLAGS "-Wl")

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES geometry_msgs std_msgs
)
# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs tf
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
        TARGETS
        RosAria
        tourGuide
#        behavior1
#        behavior2
#        behavior3
#        behavior4
#        obstacleAvoid
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
