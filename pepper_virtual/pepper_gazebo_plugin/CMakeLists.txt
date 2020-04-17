cmake_minimum_required(VERSION 2.8.3)
project(pepper_gazebo_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_plugins
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
# Gazebo cxx flags should have all the required C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED)

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    gazebo_plugins
)
install(DIRECTORY launch worlds
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} include)

add_library(gazebo_ros_pepper_drive src/pepper_drive.cpp)
target_link_libraries(gazebo_ros_pepper_drive ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_pepper_force_based_drive src/pepper_force_based_drive.cpp)
target_link_libraries(gazebo_ros_pepper_force_based_drive ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS gazebo_ros_pepper_drive gazebo_ros_pepper_force_based_drive
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)