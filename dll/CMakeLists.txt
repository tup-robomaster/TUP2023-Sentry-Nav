cmake_minimum_required(VERSION 3.5)
project(dll)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(dll_node src/dll_node.cpp)
target_compile_options(dll_node PRIVATE ${OpenMP_FLAGS})
# add_executable(grid3d_node_dll src/grid3d_node.cpp)
# target_compile_options(grid3d_node_dll PRIVATE ${OpenMP_FLAGS})

# add_library(vins_lib
#     src/estimator/parameters.cpp
#     src/estimator/estimator.cpp
#     src/estimator/feature_manager.cpp
#     src/factor/pose_local_parameterization.cpp
#     src/factor/projectionTwoFrameOneCamFactor.cpp
#     src/factor/projectionTwoFrameTwoCamFactor.cpp
#     src/factor/projectionOneFrameTwoCamFactor.cpp
#     src/factor/marginalization_factor.cpp
#     src/utility/utility.cpp
#     src/utility/visualization.cpp
#     src/utility/CameraPoseVisualization.cpp
#     src/initial/solve_5pts.cpp
#     src/initial/initial_aligment.cpp
#     src/initial/initial_sfm.cpp
#     src/initial/initial_ex_rotation.cpp
#     src/featureTracker/feature_tracker.cpp)
# target_link_libraries(vins_lib  ${OpenCV_LIBS} ${CERES_LIBRARIES})

target_link_libraries(dll_node
   ${CERES_LIBRARIES}
   ${OpenMP_LIBS}
)
# target_link_libraries(grid3d_node_dll
#    ${OpenMP_LIBS}
# )

ament_target_dependencies(dll_node rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros octomap_ros)
# ament_target_dependencies(grid3d_node_dll rclcpp rcpputils std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros pcl_ros)

# added from DEBUG

# Install launch files
install(
  DIRECTORY launch
            maps
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS dll_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
