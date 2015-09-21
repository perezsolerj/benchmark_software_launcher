cmake_minimum_required(VERSION 2.8.3)
project(benchmark_software_launcher)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 nav_msgs
 esm_tracking
 image_transport
 uwsimbenchmarks
 visp_bridge
)

find_package(VISP REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    esm_tracking
    uwsimbenchmarks	
    image_transport
    visp_bridge
  DEPENDS
    VISP
)

#Library type definition needed for ESMlibrary (INRIA)
if( CMAKE_SIZEOF_VOID_P EQUAL 8)
  set( LIBRARY_TYPE STATIC)
else( CMAKE_SIZEOF_VOID_P EQUAL 8)	
  set( LIBRARY_TYPE SHARED )
endif( CMAKE_SIZEOF_VOID_P EQUAL 8)	

# ESMtracker library building
add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS})

add_library(esmtracker ${LIBRARY_TYPE}
src/action_layer/ESMTracking.cpp
src/action_layer/VirtualImage.cpp)

target_link_libraries(esmtracker ${catkin_LIBRARIES})

# Building executables
add_executable(perceptionesmtracker src/action_layer/perception_esmtracker.cpp)
add_executable(visptracker src/action_layer/visp_tracker.cpp)
add_executable(navigatorPIcontroller src/action_layer/navigatorPIcontroller.cpp)
add_executable(navigatorPcontroller src/action_layer/navigatorPcontroller.cpp)
add_executable(vehicleThrusterAllocator src/control_layer/vehicleThrusterAllocator.cpp)
add_executable(visualStationKeeping src/task_layer/visualStationKeeping.cpp)

TARGET_LINK_LIBRARIES(perceptionesmtracker  ${catkin_LIBRARIES} ${VISP_LIBRARIES} esmtracker)
TARGET_LINK_LIBRARIES(visptracker  ${catkin_LIBRARIES} ${VISP_LIBRARIES} esmtracker)
TARGET_LINK_LIBRARIES(navigatorPIcontroller  ${catkin_LIBRARIES})
TARGET_LINK_LIBRARIES(navigatorPcontroller  ${catkin_LIBRARIES})
TARGET_LINK_LIBRARIES(vehicleThrusterAllocator  ${catkin_LIBRARIES})
TARGET_LINK_LIBRARIES(visualStationKeeping  ${catkin_LIBRARIES})

# Install
install(
  TARGETS
    perceptionesmtracker
    visptracker
    navigatorPIcontroller
    navigatorPcontroller
    vehicleThrusterAllocator
    visualStationKeeping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)



