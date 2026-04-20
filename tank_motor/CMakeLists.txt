cmake_minimum_required(VERSION 3.8)
project(tank_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_library(LGPIO_LIBRARY lgpio REQUIRED)

add_executable(motor_node src/motor.cpp)

ament_target_dependencies(motor_node
  rclcpp
  geometry_msgs
)

target_link_libraries(motor_node ${LGPIO_LIBRARY})

add_executable(keyboard_controller src/keyboard_controller.cpp)
ament_target_dependencies(keyboard_controller rclcpp geometry_msgs)

add_executable(actuator_node src/actuator_controller.cpp)

ament_target_dependencies(actuator_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(actuator_node ${LGPIO_LIBRARY})

install(TARGETS
  actuator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS keyboard_controller DESTINATION lib/${PROJECT_NAME})

install(TARGETS motor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
