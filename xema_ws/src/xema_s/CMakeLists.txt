cmake_minimum_required(VERSION 3.5)
project(xema_s)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_python REQUIRED) #for python scripts
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED) #for python scripts
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


set(dependencies "rclcpp" "geometry_msgs" "nav_msgs" "tf2" "tf2_ros")

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster ${dependencies})


install(TARGETS
  tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch urdf meshes rviz worlds scripts
  DESTINATION share/${PROJECT_NAME})


# Install Python executables #They must start with the shebag tag
install(PROGRAMS
  scripts/omni_teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME} )

ament_package()


