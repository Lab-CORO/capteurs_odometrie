cmake_minimum_required(VERSION 3.8)
project(encodeur)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(wiringPi REQUIRED)


find_library(PIGPIO_LIB pigpio pigpiod_if2 wiringPi)
if(NOT PIGPIO_LIB)
  message(FATAL_ERROR "pigpio library not found. Please install it.")
endif()

SET(CMAKE_CXX_FLAGS "-fpermissive")

# Ajouter l'exécutable encodeurs
add_executable(encodeurs src/RED.cpp src/test_RED_dual.cpp)
ament_target_dependencies(encodeurs rclcpp std_msgs)

target_link_libraries(encodeurs
  ${PIGPIO_LIB}
  pthread
  pigpiod_if2
  rt
)
install(TARGETS encodeurs
  DESTINATION lib/${PROJECT_NAME})


# central inertielle
add_executable(centrale_inertielle src/centrale_inertielle.cpp)
ament_target_dependencies(centrale_inertielle
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    sensor_msgs
    geometry_msgs
)
target_link_libraries(centrale_inertielle
  wiringPi
)
install(TARGETS centrale_inertielle
  DESTINATION lib/${PROJECT_NAME})

# Generate action and service msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AssembleCloud.srv"
  "action/StartScan.action"
  DEPENDENCIES geometry_msgs sensor_msgs 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
