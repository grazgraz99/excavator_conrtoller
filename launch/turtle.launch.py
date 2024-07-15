
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('excavator_control'),
        'config',
        'params.yaml'
    )
    
    rviz_config = os.path.join(
        get_package_share_directory('excavator_control'),
        'rviz',
        'odom_and_markers.rviz'
    )
    #print(f"Resolved rviz_config path: {rviz_config}")
    ld.add_action(SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'))
    # Path to the TurtleBot3 Gazebo launch file
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_gazebo_launch = os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')

    # Include the TurtleBot3 Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_gazebo_launch),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    path_markers_node = Node(
        package="excavator_control",
        executable="centerline_markers_node.py",
        name="centerline_markers_node"
    )

    local_controller_node = Node(
        package="excavator_control",
        executable="diff_drive_pure_pursuit_updated_copy.py",
        name="local_controller",
        parameters=[config]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]#'/home/alex/ros2_ws/src/excavator_control/rviz/odom_and_markers.rviz']
    )

    ld.add_action(gazebo_launch)
    ld.add_action(path_markers_node)
    ld.add_action(local_controller_node)
    ld.add_action(rviz_node)
    
    return ld