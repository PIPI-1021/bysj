cmake_minimum_required(VERSION 3.8)
project(agv_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17) # 建议指定C++17以支持PCL
endif()

# 1. 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(robot_localization REQUIRED)
# 2. 包含头文件路径
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 3. 编译地面过滤节点
add_executable(ground_filter_node src/ground_filter_node.cpp)
ament_target_dependencies(ground_filter_node rclcpp sensor_msgs pcl_conversions tf2 tf2_ros)
target_link_libraries(ground_filter_node ${PCL_LIBRARIES})

# 3.1 编译点云融合节点 (对应你的研究思路：环境融合)
add_executable(pointcloud_fusion_node src/pointcloud_fusion_node.cpp)
ament_target_dependencies(pointcloud_fusion_node 
  rclcpp sensor_msgs pcl_conversions pcl_ros tf2 tf2_ros tf2_sensor_msgs
)
target_link_libraries(pointcloud_fusion_node ${PCL_LIBRARIES})

# 4. 安装资源文件夹
install(DIRECTORY
  description
  launch
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

# 5. 安装可执行文件
install(TARGETS
  ground_filter_node
  pointcloud_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
