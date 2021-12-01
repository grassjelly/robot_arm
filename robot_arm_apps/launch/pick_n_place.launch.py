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
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition


def generate_launch_description():
    arm_driver_launch = PathJoinSubstitution(
        [FindPackageShare('robot_arm_driver'), 'launch', 'robot_arm_driver.launch.py']
    )

    depth_sensor_launch = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )

    moveit_config_launch = PathJoinSubstitution(
        [FindPackageShare('robot_arm_moveit_config'), 'launch', 'demo.launch.py']
    )

    camera_transform_publisher = PathJoinSubstitution(
        [FindPackageShare('robot_arm_perception'), 'launch', 'camera_transform_publisher.launch.py']
    )

    object_finder_server = PathJoinSubstitution(
        [FindPackageShare('robot_arm_perception'), 'launch', 'object_finder_server.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='fake_robot', 
            default_value='false',
            description='Use Fake Robot'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depth_sensor_launch),
            launch_arguments= {'align_depth' : 'true'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_driver_launch),
            condition=UnlessCondition(LaunchConfiguration("fake_robot")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_transform_publisher)
        ),

        Node(
            package='robot_arm_perception',
            executable='object_finder_server',
            name='object_finder_server',
            output='screen'        
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_config_launch),
            launch_arguments= {'fake_robot' : LaunchConfiguration("fake_robot")}.items()
        ),

        # Node(
        #     package='robot_arm_apps',
        #     executable='pick_n_place',
        #     name='pick_n_place',
        #     output='screen'        
        # )
    ])