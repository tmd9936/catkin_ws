cmake_minimum_required(VERSION 2.8.3)
project(object_detection_gl_ros)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
set( LIBRARY_PATH "${PROJECT_SOURCE_DIR}/lib/${ARCHITECTURE}" )
set( GL_LIBRARY "${LIBRARY_PATH}/libobject_detection.so" "${LIBRARY_PATH}/libgl_driver.so" "${LIBRARY_PATH}/libserial.so" )

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs 
  geometry_msgs
  pcl_ros
  laser_geometry
)

find_package( OpenCV REQUIRED )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs visualization_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${GL_LIBRARY}

)
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${GL_LIBRARY}
)

# 라이다와 opencv연결 
add_executable(lidar_data_process
  src/lidar_data_process.cpp
)

target_link_libraries(lidar_data_process
  ${catkin_LIBRARIES}
  ${GL_LIBRARY}

)
add_dependencies(lidar_data_process
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${GL_LIBRARY}
)

# opencv
add_executable( kf_tracker src/main.cpp )

target_link_libraries ( kf_tracker 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
)

add_dependencies(kf_tracker
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

## Mark executables and/or libraries for installation
install(TARGETS kf_tracker
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)