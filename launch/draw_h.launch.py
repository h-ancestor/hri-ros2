import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Khởi động mô phỏng Gazebo UR3e nhẹ nhàng (không cần MoveIt)
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_simulation_gz'),
                'launch',
                'ur_sim_control.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            'launch_rviz': 'true'
        }.items()
    )

    # 2. Khai báo Node vẽ chữ H trực tiếp
    draw_h_node = Node(
        package='ur3_letter_h',
        executable='draw_h_node.py',
        name='draw_h_node',
        output='screen'
    )

    # Chờ 8 giây để Gazebo bật xong là chạy node ngay
    delayed_node = TimerAction(
        period=8.0,
        actions=[draw_h_node]
    )

    return LaunchDescription([
        ur_control_launch,
        delayed_node
    ])