cmake_minimum_required(VERSION 3.8)
project(udemy_tutorials_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(udemy_tutorials_interfaces REQUIRED)

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

# CPP Node executable creation
add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)  # Add rclcpp as a dependency

# Add RobotNewsStation executable
add_executable(robot_news_stations src/robot_news_station.cpp)
ament_target_dependencies(robot_news_stations rclcpp example_interfaces)

# Add Smartphone executable
add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

# Add number_publisher executable
add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp std_msgs)

# Add number_counter executable
add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp std_msgs example_interfaces)

# Add service server node executable
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

# Add service client node executable (no OOP)
add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

# Add service client node executable (OOP)
add_executable(add_two_ints_client_oop src/add_two_ints_client_oop.cpp)
ament_target_dependencies(add_two_ints_client_oop rclcpp example_interfaces)

# Add executable for hw_status_publisher
add_executable(hw_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hw_status_publisher
    rclcpp
    udemy_tutorials_interfaces)

install(TARGETS
  cpp_node
  robot_news_stations
  smartphone
  number_publisher
  number_counter
  add_two_ints_server
  add_two_ints_client_no_oop
  add_two_ints_client_oop
  hw_status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
