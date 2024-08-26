import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
   
    base_turtle_control_node = Node(
        package='my_package',
        executable='base_turtle_control',
        name='base_turtle_control'
    )



    launch_description = LaunchDescription()

    launch_description.add_action(base_turtle_control_node)


    return launch_description