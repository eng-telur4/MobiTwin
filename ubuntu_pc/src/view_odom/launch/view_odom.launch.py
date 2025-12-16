import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージ名とRViz設定ファイルのパスを取得
    pkg_name = 'view_odom'
    rviz_config_dir = os.path.join(get_package_share_directory(pkg_name), 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'view_odom.rviz')
    
    # -----------------------------------------------------------
    # 1. UDP統合ノードの定義
    # -----------------------------------------------------------
    view_odom_node = Node(
        package=pkg_name,
        executable=pkg_name, # 実行可能ファイル名も view_odom と仮定
        name='view_odom',
        output='screen',
        # UDP通信設定はC++コード内にハードコードされているため、ここではパラメータ不要
    )

    # -----------------------------------------------------------
    # 2. RViz2の起動と設定ファイルの読み込み
    # -----------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_visualizer',
        output='screen',
        # -d オプションで、準備した設定ファイルを読み込む
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        view_odom_node,
        rviz_node, # RVizを同時に起動
    ])