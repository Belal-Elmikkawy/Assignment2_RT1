import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Find the Simulation Package
    try:
        sim_pkg_share = FindPackageShare('bme_gazebo_sensors')

        # Include launch file
        simulation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sim_pkg_share, 'launch', 'spawn_robot.launch.py'])
            )
        )
        has_sim = True
    except Exception:
        print("WARNING: 'bme_gazebo_sensors' not found. Launching only custom nodes.")
        has_sim = False

    # Define C++ Controller Node
    controller_node = Node(
        package='assignment2_rt',
        executable='controller_node_cpp',
        name='controller',
        prefix='xterm -e',
        output='screen'
    )

    # Define C++ Monitoring Node
    monitor_node = Node(
        package='assignment2_rt',
        executable='monitoring_node_cpp',
        name='monitor',
        prefix='xterm -e',
        output='screen'
    )

    launch_list = []
    if has_sim:
        launch_list.append(simulation)

    launch_list.append(monitor_node)
    launch_list.append(controller_node)

    return LaunchDescription(launch_list)
