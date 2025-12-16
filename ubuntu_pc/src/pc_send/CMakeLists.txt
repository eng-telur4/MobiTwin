cmake_minimum_required(VERSION 3.8)
project(pc_send)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 実行可能ファイルの定義
add_executable(pc_send src/pc_send.cpp)

# リンクするライブラリの指定: C++ノードでCのソケット関数を使用
ament_target_dependencies(pc_send rclcpp sensor_msgs)

# ネットワークとスレッド関連の標準ライブラリをリンク
target_link_libraries(pc_send ${CMAKE_DL_LIBS} ${CMAKE_THREAD_LIBS_INIT})

# インストール設定
install(TARGETS
  pc_send
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
