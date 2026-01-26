import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch the Simulation (from bme_gazebo_sensors)
    try:
        sim_pkg_share = FindPackageShare('bme_gazebo_sensors')
        simulation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sim_pkg_share, 'launch', 'spawn_robot.launch.py'])
            )
        )
        has_sim = True
    except Exception:
        print("WARNING: 'bme_gazebo_sensors' not found. Ensure it is in your workspace.")
        has_sim = False

    # Launch Controller (from assignment2_rt)
    controller_node = Node(
        package='assignment2_rt',    # <--- YOUR PACKAGE
        executable='controller_node.py',
        name='controller',
        prefix='xterm -e',
        output='screen'
    )

    # Launch Monitor (from assignment2_rt)
    monitor_node = Node(
        package='assignment2_rt',    # <--- YOUR PACKAGE
        executable='monitoring_node.py',
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
