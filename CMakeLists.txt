cmake_minimum_required(VERSION 2.8.3)
project(zed_camera_ros)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs image_transport cv_bridge camera_info_manager)
find_package( OpenCV REQUIRED )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(zed_camera_ros src/zed_camera_ros.cpp) 
target_link_libraries(zed_camera_ros ${catkin_LIBRARIES}  ${OpenCV_LIBS} )
