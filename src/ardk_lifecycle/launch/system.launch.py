from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ardk_lifecycle'),
        'config',
        'ardk_params.yaml'
    )

    state_manager_node = Node(
        package='ardk_lifecycle',
        executable='state_manager',
        name='state_manager',
        output='screen',
        parameters=[config]
        # We assume root access or properly configured sudoers isn't strictly needed 
        # for the subprocesses if running as the user who owns the ROS install/workspace.
        # Smoke tests ran fine as user 'gio'.
    )

    return LaunchDescription([
        state_manager_node
    ])
