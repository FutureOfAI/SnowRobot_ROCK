cmake_minimum_required(VERSION 2.8.3)
project(uwb_zigzag)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  std_msgs
  nav_msgs
  eigen_stl_containers
  message_filters
  #rac_serial
  serial
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  include/spdlog
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(initialization_uwb  src/initialization_uwb.cpp) 
add_dependencies(initialization_uwb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(initialization_uwb  ${catkin_LIBRARIES})

add_executable(motion_uwb  src/motion_uwb.cpp) 
add_dependencies(motion_uwb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_uwb  ${catkin_LIBRARIES})

add_executable(location_uwb  src/location_uwb.cpp) 
add_dependencies(location_uwb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(location_uwb  ${catkin_LIBRARIES})

add_executable(map_uwb  src/map_uwb.cpp) 
add_dependencies(map_uwb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_uwb  ${catkin_LIBRARIES})

add_executable(uwb_reader  src/uwb_reader.cpp) 
add_dependencies(uwb_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwb_reader  ${catkin_LIBRARIES})

add_executable(imu_reader  src/imu_reader.cpp) 
add_dependencies(imu_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_reader  ${catkin_LIBRARIES})

add_executable(mag_reader  src/mag_reader.cpp) 
add_dependencies(mag_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_reader  ${catkin_LIBRARIES})

add_executable(plan_uwb src/plan_uwb.cpp src/calibinit.cpp src/calib.cpp src/contour.cpp src/astar.cpp src/blockallocator.cpp src/path.cpp)
add_dependencies(plan_uwb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_uwb ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(RosTopic_test src/RosTopic_test.cpp) 
add_dependencies(RosTopic_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(RosTopic_test  ${catkin_LIBRARIES})

add_executable(uwb_usb_reader src/uwb_usb_reader.cpp) 
add_dependencies(uwb_usb_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwb_usb_reader  ${catkin_LIBRARIES})

#add_executable(uwbRawData src/serial_uwb.cpp) 
#add_dependencies(uwbRawData ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(uwbRawData  ${catkin_LIBRARIES})

# add_executable(joycontrol src/joycontrol.cpp)
# target_link_libraries(joycontrol ${catkin_LIBRARIES})

# add_executable(superJoy  src/superJoy.cpp)
# add_dependencies(superJoy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(superJoy  ${catkin_LIBRARIES})
