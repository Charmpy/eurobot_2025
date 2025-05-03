import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='shesnar' #<--- CHANGE ME

    build_map = LaunchConfiguration('build_map')
    is_localization = LaunchConfiguration('is_localization')    
    map_file_path = LaunchConfiguration('map')    

    declare_build_map_cmd = DeclareLaunchArgument(
        'build_map', default_value='false', description='build map'
    )

    declare_localization_cmd = DeclareLaunchArgument(
        'is_localization', default_value='false', description='localization'
    )

    default_map = os.path.join(
            get_package_share_directory(package_name),
            'maps',
            'euro_map_2.yaml'
            )     
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=default_map, description='Full path to map yaml file to load'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    config_ekf= os.path.join(get_package_share_directory("shesnar"),'config','ekf_params.yaml')

    ekf = Node(
        package = 'robot_localization',
        name = 'ekf_filter_node',
        executable = 'ekf_node',
        output="screen",
        parameters=[config_ekf],
    )

    # # to do map 
    slam_params = os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','online_async_launch.py'
                )]),
        # condition=IfCondition( build_map ),        
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': slam_params,
        }.items()
    )

    nav_params = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')
    # nav_params = os.path.join(get_package_share_directory(package_name),'config','nav2_params_amcl.yaml')
    start_localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','localization_launch.py'
                    # get_package_share_directory(package_name),'launch','localization_launch_amcl.py'
                )]), 
                # condition=IfCondition( is_localization ), 
                launch_arguments={'map': map_file_path, 'use_sim_time': 'false', 'params_file': nav_params}.items()
    )
    
    start_navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )]), 
                # condition=IfCondition( is_navigation ), 
                launch_arguments={'use_sim_time': 'false', 'map_subscribe_transient_local': 'true', 'params_file': nav_params}.items()
    )


    start_route_controller = Node(
        package="route_controller",
        executable="driver",
        output='screen',
        arguments=[
        ],
        parameters = [{'use_sim_time': False}]
    )

    start_depth_cameras = Node(
        package="depth_camera",
        executable="depth_camera_handler",
        output='screen',
        arguments=[
        ],
        parameters = [{'use_sim_time': False}]
    )
    
    # Launch them all!
    return LaunchDescription([        
        declare_build_map_cmd,
        declare_localization_cmd,
        declare_map_yaml_cmd,
        rsp,
        ekf,
        # slam_launch,        

        start_localization,
        start_navigation,
        # start_depth_cameras,
        # start_route_controller,
    ])