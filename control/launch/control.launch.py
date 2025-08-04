from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # Получаем путь к установленному пакету 'control'
    package_prefix = get_package_prefix('control')
    # Путь к скрипту processes.py внутри lib/control
    script_path = os.path.join(package_prefix, 'lib', 'control', 'processes.py')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', script_path],
            output='screen'
        ),
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen',
            arguments=['udpin://0.0.0.0:14540']
        )
    ])
