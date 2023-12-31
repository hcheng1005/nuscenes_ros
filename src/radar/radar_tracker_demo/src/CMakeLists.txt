
cmake_minimum_required(VERSION 3.8) # C++17 support was added in CMake 3.8
project(radar_tracker)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release) # Debug, Release, RelWithDebInfo
find_package(Boost 1.65.0 COMPONENTS filesystem thread REQUIRED) # Ubuntu 18.04

find_package(catkin
             REQUIRED
             rosbag
             sensor_msgs
             cv_bridge
             pcl_conversions
             pcl_msgs
             message_generation
             geometry_msgs
             std_msgs
             nav_msgs
             tf)

find_package(OpenCV REQUIRED core)

add_message_files(FILES RadarObjects.msg RadarObject.msg)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs
                  tf
                  nav_msgs)

catkin_package(INCLUDE_DIRS
               include
               DEPENDS
               OpenCV)

include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

include_directories(include)
# include_directories(/home/zdhjs-05/myGitHubCode/nuscenes2bag-master/wk_spc/src/include/)
include_directories(/home/charles/myCode/nus_ros/src/nuscenes_ros/include/)


# set(SRCS
#     src/radarTracker.cpp
#     src/DBSCAN.cc)

file(GLOB_RECURSE SOURCE_FILES 
  # src/main.cpp
  src/tracker/*.cc
  src/new_tracker/*.cc
  include/common/*.cc
  include/common/*.cpp
)

# add_executable(${PROJECT_NAME} ${SOURCE_FILES} src/main.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES} src/new_tracker/randomMatriceTest.cc)

target_compile_options(${PROJECT_NAME}
                       PRIVATE
                               -Wall
                               -Wextra
                               -pedantic
                               )

target_compile_definitions(${PROJECT_NAME} PRIVATE "-DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}")

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
                      ${OpenCV_LIBRARIES}
                      ${PCL_COMMON_LIBRARY}
                      ${PCL_IO_LIBRARY}
                      ${catkin_LIBRARIES})
