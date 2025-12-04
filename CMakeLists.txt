cmake_minimum_required(VERSION 3.8)
project(s1_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 依存パッケージの探索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 2. 実行ファイルの作成
# (src/pid_feedback.cpp をコンパイルして pid_node という実行ファイルを作る)
add_executable(pid_node src/pid_feedback.cpp)

# 3. ライブラリのリンク
ament_target_dependencies(pid_node
  rclcpp
  geometry_msgs
  nav_msgs
)

# 4. インストール設定 (実行ファイル)
# (install/lib/s1_controller/ の中に配置する)
install(TARGETS
  pid_node
  DESTINATION lib/${PROJECT_NAME}
)

# 5. インストール設定 (LaunchとConfig)
# (install/share/s1_controller/ の中にディレクトリごと配置する)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
