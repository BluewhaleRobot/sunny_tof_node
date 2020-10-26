cmake_minimum_required(VERSION 2.8.3)
project(sunny_tof_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread ")

set(CMAKE_INSTALL_RPATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/bin)

set(BOOST_LIBS thread)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 3.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy sensor_msgs pcl_conversions pcl_msgs pcl_ros
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#添加外部库搜索路径
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(sunny_tof_node src/sunny_tof_node.cpp)

target_link_libraries(sunny_tof_node
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      )
target_link_libraries(sunny_tof_node   libbz2.so    libCalcPhaseRadialGray.so libCaliIniProcessing.so libCorFppnWiggling.so libCorPSF.so libDepthFilter.so libspectre_processing.so libTOFCalDis.so  libtof_dev_sdk.so)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

# copy to devel
file(GLOB ADDWA_LOCAL_PLANNER_LIB
  "lib/*"
)
file(COPY ${ADDWA_LOCAL_PLANNER_LIB} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
