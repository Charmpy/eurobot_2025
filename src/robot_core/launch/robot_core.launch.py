import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node



def generate_launch_description():
    package_name='robot_core' #<--- CHANGE ME

    config_drive = os.path.join(get_package_share_directory("uart_drive"),'config','uart_params.yaml')
    print(config_drive)
    # config_drive = os.path.join(
    #     '~/eurobot_2025/src/uart_drive',
    #     'config',
    #     'uart_params.yaml'
    #     )
        
    uart_drive = Node(
        package = 'uart_drive',
        name = 'uart_node',
        executable = 'uart_drive',
        output="screen",
        parameters=[config_drive],
    )

    config_grippers = os.path.join(get_package_share_directory("uart_grippers"),'config','uart_params.yaml')
    # config_grippers = os.path.join(
    #     '~/eurobot_2025/src/uart_grippers',
    #     'config',
    #     'uart_params.yaml'
    #     )
    uart_grippers = Node(
        package = 'uart_grippers',
        name = 'grippers_node',
        executable = 'uart_grippers',
        output="screen",
        parameters=[config_grippers],
    )

    # lidar = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a3_launch.py')]),
    #                 # launch_arguments={'gz_args': ['-r --render-engine ogre -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    #                 # launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    # )

    return LaunchDescription([  
        # lidar,      
        uart_drive,
        uart_grippers,
    ])