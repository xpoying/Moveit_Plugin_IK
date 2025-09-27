cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖（Eigen3 用 find_package 而非 ament，因它是系统库）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)  # 正确查找 Eigen3
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成插件库（目标名：inverse_kinematics_plugin）
add_library(${PROJECT_NAME}_plugin SHARED
  src/IKPlugin.cpp
  src/IKSolution.cpp
)

# 包含目录（仅头文件目录）
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}  # 添加 Eigen3 头文件目录
)

# 链接依赖（ament_target_dependencies 链接 ROS 2 包，target_link_libraries 链接 Eigen3）
ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp
  moveit_core
  pluginlib
  tf2_eigen
  geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_plugin Eigen3::Eigen)  # 正确链接 Eigen3

# 编译定义（按需保留，通常用于 Windows 动态库，Linux 可选）
target_compile_definitions(${PROJECT_NAME}_plugin 
  PRIVATE "INVERSE_KINEMATICS_PLUGIN_BUILDING_DLL"
)

# 关键：导出插件描述文件（文件名与实际创建的 plugins.xml 一致）
pluginlib_export_plugin_description_file(moveit_core plugins.xml)

# 安装插件描述文件（到 share/包名 目录，供系统查找）
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# 安装头文件（到 install/include 目录）
install(DIRECTORY include/
  DESTINATION include
  
)

# 安装插件库（合并重复目标，统一导出名）
install(TARGETS ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出依赖和目标（确保其他包能找到本插件）
ament_export_dependencies(
  rclcpp
  moveit_core
  pluginlib
  tf2_eigen
  geometry_msgs
)
ament_export_targets(export_${PROJECT_NAME}_plugin)  # 与 install(TARGETS) 的 EXPORT 一致

ament_package()