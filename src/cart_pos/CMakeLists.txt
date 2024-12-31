cmake_minimum_required(VERSION 3.8)
project(cart_pos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # Add std_msgs dependency for Float32

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cartposvector.msg"
  DEPENDENCIES builtin_interfaces
)

add_executable(cart_pos_node src/cart_pos.cpp)
target_include_directories(cart_pos_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cart_pos_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cart_pos_node
  moveit_ros_planning_interface
  rclcpp
  tf2
  tf2_msgs
  std_msgs  # Added std_msgs for Float32 message
)

# Link generated interfaces
#rosidl_target_interfaces(cart_pos_node
#  ${PROJECT_NAME} "rosidl_typesupport_cpp"
#)

# Get the typesupport target and link it manually
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cart_pos_node ${typesupport_target})

install(TARGETS cart_pos_node
  DESTINATION lib/${PROJECT_NAME})

# Install the msg files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

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
