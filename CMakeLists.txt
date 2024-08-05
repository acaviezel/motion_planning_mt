cmake_minimum_required(VERSION 3.8)
project(motion_planning_mt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(Franka REQUIRED)

add_executable(my_hello_moveit src/my_hello_moveit.cpp)
add_executable(rrt_planner src/rrt_planner.cpp)
add_executable(random_scene_generator src/random_scene_generator.cpp)
add_executable(planning_scene_maintainer src/planning_scene_maintainer.cpp)
add_executable(explicit_reference_governor src/explicit_reference_governor.cpp)
add_executable(distance_calculator src/distance_calculator.cpp)
add_executable(motion_planner src/motion_planner.cpp)
add_executable(dynamic_random_scene src/dynamic_random_scene.cpp)
add_executable(moving_box src/moving_box.cpp)

target_include_directories(my_hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  my_hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

target_include_directories(rrt_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rrt_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  rrt_planner
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
  "moveit_msgs"
  "shape_msgs"
  "sensor_msgs"
  "trajectory_msgs"
  "geometry_msgs"
)

target_include_directories(random_scene_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(random_scene_generator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  random_scene_generator
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_msgs"
  "shape_msgs"
  "sensor_msgs"
  "trajectory_msgs"
)

target_include_directories(planning_scene_maintainer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planning_scene_maintainer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  planning_scene_maintainer
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_msgs"
  "shape_msgs"
  "sensor_msgs"
  "trajectory_msgs"
)

target_include_directories(explicit_reference_governor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(explicit_reference_governor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  explicit_reference_governor
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
  "sensor_msgs"
  "trajectory_msgs"
)

target_include_directories(distance_calculator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(distance_calculator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  distance_calculator
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_msgs"
  "shape_msgs"
  "sensor_msgs"
  "trajectory_msgs"
  "Franka"
)

target_include_directories(motion_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  motion_planner
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
  "moveit_msgs"
  "shape_msgs"
  "sensor_msgs"
  "trajectory_msgs"
  "geometry_msgs"
)

target_include_directories(dynamic_random_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamic_random_scene PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dynamic_random_scene
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_msgs"
  "shape_msgs"
  "sensor_msgs"
  "trajectory_msgs"
)

target_include_directories(moving_box PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moving_box PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moving_box
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_msgs"
  "shape_msgs"
  "sensor_msgs"
  "geometry_msgs"
)

install(TARGETS 
  my_hello_moveit
  rrt_planner
  random_scene_generator
  planning_scene_maintainer
  explicit_reference_governor
  distance_calculator
  motion_planner
  dynamic_random_scene
  moving_box
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
