# @file final_mission.launch.py
#
# @author Chase Snyder
#
# @brief Launch file for coordinating odometry node, controller node, and
#        providing URDF via robot_state_publisher and visualization via RViz.
#
# Only real change is I changed the publisher to start first to avoid
# Image_callbacking before evaultor was ready

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    pkg_share = FindPackageShare('robot_bringup').find('robot_bringup')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xml')
    rviz_path = os.path.join(pkg_share, 'rviz', 'simulation.rviz')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        #I had time stamp issues so this help me force everything
        #to be the in the "same" time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Publish the TF tree from URDF first (base_link -> camera_link -> camera_optical_frame, etc.)
        # Robot's current urdf should be identical to it's sdf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_desc},
            ],
        ),

        # Odometry (odom -> base_link TF, plus /odom topic)
        Node(
            package='robot_simulator_cpp',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Controller (subscribes missions, publishes cmd_vel, detects ArUco, reports)
        Node(
            package='robot_simulator_py',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node( # robot visualization
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
