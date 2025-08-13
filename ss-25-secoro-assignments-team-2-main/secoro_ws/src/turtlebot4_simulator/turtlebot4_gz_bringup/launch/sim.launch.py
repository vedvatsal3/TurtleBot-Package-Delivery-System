# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from pathlib import Path

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Simulation World'),
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('gz_resource_path', default_value='',
                          description='Gazebo resource path'),
    DeclareLaunchArgument('gz_model_path', default_value='',
                            description='Gazebo model path'),
]


def set_gz_resource_path_env_variable(context):
    """
    Set the GZ_SIM_RESOURCE_PATH environment variable based on the
    compile time argument value.
    """
    # Directories
    pkg_turtlebot4_gz_bringup = get_package_share_directory(
        'turtlebot4_gz_bringup')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_irobot_create_gz_bringup = get_package_share_directory(
        'irobot_create_gz_bringup')
    
    gz_resource_path_env_variable = LaunchConfiguration('gz_resource_path').perform(context)
            
    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_turtlebot4_gz_bringup, 'worlds'),
            os.path.join(pkg_irobot_create_gz_bringup, 'worlds'),
            str(Path(pkg_turtlebot4_description).parent.resolve()),
            str(Path(pkg_irobot_create_description).parent.resolve()),
            gz_resource_path_env_variable,
        ])
    )
    
    # Get the compile time argument value from the context
    return [gz_resource_path]

def generate_launch_description():

    # Directories
    pkg_turtlebot4_gz_gui_plugins = get_package_share_directory(
        'turtlebot4_gz_gui_plugins')
    pkg_irobot_create_gz_plugins = get_package_share_directory(
        'irobot_create_gz_plugins')
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    verify_gz_env = ExecuteProcess(
        cmd=['sh', '-c', 'echo GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH'],
        output='screen'
    )
    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_turtlebot4_gz_gui_plugins, 'lib'),
            os.path.join(pkg_irobot_create_gz_plugins, 'lib')
        ])
    )

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Gazebo harmonic (commenting gui for better performance)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.sdf',
                ' -r',
                ' -v 4',
                # ' --gui-config ',
                # PathJoinSubstitution([
                #     pkg_turtlebot4_gz_bringup,
                #     'gui',
                #     LaunchConfiguration('model'),
                #     'gui.config'
                # ])
            ])
        ]
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=set_gz_resource_path_env_variable))
    ld.add_action(verify_gz_env)
    # ld.add_action(gz_gui_plugin_path) # commenting gui for better performance
    ld.add_action(gazebo)
    ld.add_action(clock_bridge)
    return ld
