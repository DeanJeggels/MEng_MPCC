cmake_minimum_required(VERSION 3.8)
project(mpcc)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})
find_package(nav_msgs REQUIRED)
include_directories(${nav_msgs_INCLUDE_DIRS})
find_package(geometry_msgs REQUIRED)
include_directories(${geometry_msgs_INCLUDE_DIRS})
find_package(std_msgs REQUIRED)
include_directories(${std_msgs_INCLUDE_DIRS})
find_package(visualization_msgs REQUIRED)
include_directories(${visualization_msgs_INCLUDE_DIRS})
find_package(tf2 REQUIRED)
include_directories(${tf2_INCLUDE_DIRS})
find_package(tf2_ros REQUIRED)
include_directories(${tf2_ros_INCLUDE_DIRS})
find_package(tf2_geometry_msgs REQUIRED)
include_directories(${tf2_geometry_msgs_INCLUDE_DIRS})
find_package(gazebo_msgs REQUIRED)
include_directories(${gazebo_msgs_INCLUDE_DIRS})
find_package(ament_index_cpp REQUIRED)
include_directories(${ament_index_cpp_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(mpc++ CONFIG REQUIRED)
include_directories(${mpc++_INCLUDE_DIRS})
find_package(NLopt REQUIRED)
include_directories(${NLOPT_INCLUDE_DIRS})
find_package(std_srvs REQUIRED)
include_directories(${std_srvs_INCLUDE_DIRS})
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_sensor_msgs)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  include
)

# LinearSpline source file (used by multiple executables)
set(LINEAR_SPLINE_SRC
  src/linear_spline.cpp
)

# Create the linear_spline_visualizer executable
add_executable(linear_spline_visualizer
  src/linear_spline_visualizer.cpp
  ${LINEAR_SPLINE_SRC}
)
ament_target_dependencies(linear_spline_visualizer
  rclcpp
  nav_msgs
  geometry_msgs
)

# Create the system_dynamics_node executable
add_executable(system_dynamics_node
  src/system_dynamics_node.cpp
)
ament_target_dependencies(system_dynamics_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

# Create the contouring_error_node executable
add_executable(contouring_error_node
  src/contouring_error_node.cpp
)
ament_target_dependencies(contouring_error_node
  rclcpp
  nav_msgs
  std_msgs
  visualization_msgs
  ament_index_cpp
)

add_executable(mpcc_bag_manager src/mpcc_bag_manager.cpp)
ament_target_dependencies(mpcc_bag_manager rclcpp std_srvs rosbag2_cpp nav_msgs)
install(TARGETS mpcc_bag_manager DESTINATION lib/${PROJECT_NAME})

add_executable(occupancy_extractor_node 
  src/occupancy_extractor.cpp)

target_include_directories(occupancy_extractor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  occupancy_extractor_node
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "sensor_msgs"
  "std_msgs"
)

# Install targets
install(TARGETS occupancy_extractor_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(actual_path_publisher
  src/actual_path_pub.cpp
)
ament_target_dependencies(actual_path_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  gazebo_msgs
  std_msgs
  visualization_msgs
  ament_index_cpp
)

# Create the mpcc_controller executable as a library and link Eigen3 properly
add_library(mpcc_controller_lib src/mpcc_controller.cpp)
add_library(mpcc_controller_opt_lib src/mpcc_controller_optimized.cpp)
add_library(mpcc_lib src/mpcc.cpp)
add_library(mpcc_controller_obs_lib src/mpcc_obstacle.cpp)
add_library(karretjie_lib src/karretjie.cpp)
add_library(karretjie_opt_lib src/karretjie_optimized.cpp)

target_link_libraries(mpcc_controller_lib
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${eiquadprog_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  Eigen3::Eigen
  NLopt::nlopt
)

target_link_libraries(mpcc_lib
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${eiquadprog_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${gazebo_msgs_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  Eigen3::Eigen
  NLopt::nlopt
)

target_link_libraries(mpcc_controller_opt_lib
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${eiquadprog_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${gazebo_msgs_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  Eigen3::Eigen
  NLopt::nlopt
)

target_link_libraries(mpcc_controller_obs_lib
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${eiquadprog_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${ament_index_cpp_LIBRARIES}
  ${gazebo_msgs_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  Eigen3::Eigen
  NLopt::nlopt
)

# MPCC Solver Benchmark executable - CORRECTED
add_executable(mpcc_solver_benchmark src/mpcc_solver_benchmark.cpp)

# Dependencies for benchmark node
ament_target_dependencies(mpcc_solver_benchmark 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  std_msgs 
  gazebo_msgs 
  visualization_msgs 
  Eigen3
  std_srvs
)

# Link MPC++ library correctly for benchmark node
target_link_libraries(mpcc_solver_benchmark
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${gazebo_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  Eigen3::Eigen
  NLopt::nlopt
)

add_executable(mpcc_controller src/mpcc_controller.cpp) # Create mpcc_controller executable
target_link_libraries(mpcc_controller
mpcc_controller_lib
)

add_executable(mpcc src/mpcc.cpp) # Create mpcc executable
target_link_libraries(mpcc
mpcc_lib
)

add_executable(mpcc_controller_opt src/mpcc_controller_optimized.cpp) # Create mpcc_controller executable
target_link_libraries(mpcc_controller_opt
mpcc_controller_opt_lib
)

add_executable(mpcc_controller_obs src/mpcc_obstacle.cpp)
target_link_libraries(mpcc_controller_obs
mpcc_controller_obs_lib
)

add_executable(karretjie src/karretjie.cpp) # Create karretjie executable
target_link_libraries(karretjie
  karretjie_lib
)

add_executable(karretjie_opt src/karretjie_optimized.cpp) # Create karretjie executable
target_link_libraries(karretjie_opt
  karretjie_opt_lib
)

# Add executable for path recorder
add_executable(path_recorder src/path_recorder.cpp)

# Dependencies for path recorder
ament_target_dependencies(path_recorder
  rclcpp
  gazebo_msgs
  std_srvs
  geometry_msgs
)

ament_target_dependencies(mpcc_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  gazebo_msgs
)

ament_target_dependencies(mpcc_controller_opt
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  gazebo_msgs
)

ament_target_dependencies(mpcc_controller_obs
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  gazebo_msgs
)

ament_target_dependencies(mpcc
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(karretjie
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(karretjie_opt
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install executables and libraries 
install(TARGETS 
  linear_spline_visualizer 
  system_dynamics_node 
  contouring_error_node 
  mpcc_controller 
  mpcc_controller_lib 
  mpcc_controller_opt
  mpcc_controller_opt_lib
  mpcc_controller_obs
  mpcc_controller_obs_lib
  actual_path_publisher
  mpcc
  mpcc_lib
  karretjie
  karretjie_lib
  karretjie_opt
  karretjie_opt_lib
  mpcc_solver_benchmark  # Added benchmark to install
  path_recorder          # Added path_recorder to install
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/trajectory_viz.py
  scripts/trajectory_viz_karretjie.py
  scripts/compute_max_curvature.py
  scripts/analyze_benchmark.py
  scripts/frame_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories 
install(DIRECTORY 
  include 
  launch 
  description 
  maps 
  meshes 
  models 
  rviz 
  scripts 
  worlds 
  test 
  config 
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY racetracks
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.csv"
)

ament_package()
