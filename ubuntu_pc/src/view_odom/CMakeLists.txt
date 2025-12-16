cmake_minimum_required(VERSION 3.8)
project(view_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

# ----------------------------------------------------
# 1. 必要なROS 2パッケージの検索
# ----------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ----------------------------------------------------
# 2. 実行可能ファイルの定義とコンパイル
# ----------------------------------------------------
# ソースファイル名が 'view_odom.cpp' であると仮定
add_executable(view_odom src/view_odom.cpp)

target_include_directories(view_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include)

target_compile_features(view_odom PUBLIC cxx_std_17)

# ----------------------------------------------------
# 3. 依存関係のリンク
# ----------------------------------------------------
ament_target_dependencies(view_odom 
  rclcpp 
  nav_msgs 
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 【重要】UDP通信、スレッド処理に必要な標準ライブラリをリンク
target_link_libraries(view_odom ${CMAKE_DL_LIBS} ${CMAKE_THREAD_LIBS_INIT})

# ----------------------------------------------------
# 4. インストール設定
# ----------------------------------------------------
install(TARGETS
  view_odom
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------
# 5. RViz設定ファイルをインストール
# ----------------------------------------------------
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------------------------------
# 6. Python launchファイルをインストール
# ----------------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  # PATTERN "*.py"
)
ament_package()