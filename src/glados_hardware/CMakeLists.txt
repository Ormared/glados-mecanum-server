cmake_minimum_required(VERSION 3.8)
project(glados_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

# Custom msg
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Int8Array.msg"
  )
  ament_export_dependencies(rosidl_default_runtime)
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

# Serial node
  add_executable(serial_node src/serial_node.cpp)
  ament_target_dependencies(serial_node rclcpp std_msgs)

  target_link_libraries(serial_node "${cpp_typesupport_target}")

  install(TARGETS
    serial_node
    DESTINATION lib/${PROJECT_NAME}
  )

# Teleop node
  add_executable(teleop_to_serial_node src/teleop_to_serial_node.cpp)
  ament_target_dependencies(teleop_to_serial_node rclcpp geometry_msgs std_msgs)

  target_link_libraries(teleop_to_serial_node "${cpp_typesupport_target}")

  install(TARGETS
          teleop_to_serial_node
          DESTINATION lib/${PROJECT_NAME}
  )

# Voltmeter node
  add_executable(digital_voltmeter_node src/digital_voltmeter_node.cpp)
  include_directories(include)

  ament_target_dependencies(digital_voltmeter_node
  rclcpp
  std_msgs)

  install(TARGETS digital_voltmeter_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

