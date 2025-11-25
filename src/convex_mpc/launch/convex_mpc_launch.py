from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Use the installed share directory to locate the params file
    convex_mpc_params_file = os.path.join(
        get_package_share_directory('convex_mpc'),
        'config',
        'convex_mpc_params.yaml'
    )

    return LaunchDescription([
        # MPC controller node
        Node(
            package='convex_mpc',
            executable='convex_mpc_controller',
            name='convex_mpc_controller',
            parameters=[convex_mpc_params_file],
        ),

        # External reference trajectory generator node
        Node(
            package='convex_mpc',
            executable='reference_trajectory_node',
            name='reference_trajectory_node',
            parameters=[
                convex_mpc_params_file,  # gives it N_STATES, N_MPC, mpc_dt
                {
                    # override reference behavior here if you want
                    'vx_ref': 0.3,
                    'vy_ref': 0.0,
                    'wz_ref': 0.0,
                },
            ],
        ),

        # Joystick node (if you still want it running)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
    ])

