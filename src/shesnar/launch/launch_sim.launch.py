import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled

    package_name='shesnar' 

    pkg_path = os.path.join(get_package_share_directory(package_name))
    # xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    # meshes_path = os.path.join(pkg_path, 'meshes')
    # worlds_path = os.path.join(pkg_path, 'worlds')

    meshes_path = "/home/none/eurobot_2025/install/shesnar/share/shesnar/meshes"

    # os.environ['GZ_SIM_RESOURCE_PATH'] = meshes_path + ":" + worlds_path
    os.environ['GZ_SIM_RESOURCE_PATH'] = meshes_path

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'shesnar'],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])
