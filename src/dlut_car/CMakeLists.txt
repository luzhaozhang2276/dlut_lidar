cmake_minimum_required(VERSION 2.8.3)
project(dlut_car)

find_package(catkin REQUIRED COMPONENTS roscpp rospy serial std_msgs joy message_generation cartographer_ros cartographer_ros_msgs)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

add_executable(car_serial
  src/car_serial_pub.cpp
)

add_dependencies(car_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(car_serial
  ${catkin_LIBRARIES}
)
