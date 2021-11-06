# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    joints_config = PathJoinSubstitution(
        [FindPackageShare('robot_arm_driver'), 'config', 'joints.yaml']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('robot_arm_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        Node(
            package='robot_arm_driver',
            executable='robot_arm_driver',
            name='robot_arm_driver',
            output='screen',
            parameters=[joints_config]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'publish_joints': 'false',
                'rviz': LaunchConfiguration("rviz")
            }.items()
        )
    ])