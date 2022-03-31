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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("robot_arm_description"), "urdf", "arm.urdf.xacro"]
    )

    robot_description = Command(
        ['xacro ', urdf_path]
    )

    joints_config = PathJoinSubstitution(
        [FindPackageShare('robot_arm_driver'), 'config', 'joints.yaml']
    )

    controller_config = PathJoinSubstitution(
        [FindPackageShare('robot_arm_driver'), 'config', 'controller.yaml']
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

        DeclareLaunchArgument(
            name='launch_description', 
            default_value='true',
            description='Run with robot description'
        ),

        DeclareLaunchArgument(
            name='joint_states_topic', 
            default_value='/joint_states',
            description='Robot Arm Joint States Topic'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controller_config,
            ],
        ),

        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['robot_arm_controller'],
        ),

        Node(
            package='robot_arm_driver',
            executable='robot_arm_driver',
            name='robot_arm_driver',
            output='screen',
            parameters=[joints_config],
            remappings=[
                ('/joint_states', LaunchConfiguration("joint_states_topic")),
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'publish_joints': 'false',
                'rviz': LaunchConfiguration("rviz")
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_description")),
        )
    ])