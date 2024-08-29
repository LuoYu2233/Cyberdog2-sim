from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import os

def generate_launch_description():
    python_executable = '/usr/bin/python3'  # 使用 `which python3` 得到的路径
    script_path = '/home/cyberdog_sim/src/cyberdog_simulator/motion/main.py'
    script_path1 = '/home/cyberdog_sim/src/cyberdog_simulator/motion/data.py'
    
    return LaunchDescription([
        Node(
            package='lidar_pkg',
            executable='lidar_node',
            name='lidar_node',
        ),
        Node(
            package='get_RGB',
            executable='get_rgb',
            name='rgb_node',
        ),
        ExecuteProcess(
            cmd=[python_executable, script_path1],
            output='screen',
            cwd=os.path.dirname(script_path1)  # 设置工作目录为脚本所在目录
        )
    ])

if __name__ == '__main__':
    generate_launch_description()