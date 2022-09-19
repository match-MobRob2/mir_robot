# Copyright (c) 2018-2022, Martin Günther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: relffok

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    mir_description_dir = get_package_share_directory('mir_description')

    def create_robot_description(context):
        ns = context.launch_configurations['namespace']
        if ns.startswith('/'):
            ns = ns[1:]
        urdf_dir = os.path.join(mir_description_dir, 'urdf')
        xacro_file = os.path.join(urdf_dir, 'mir.urdf.xacro')
        doc = xacro.process_file(xacro_file, mappings={'tf_prefix': ns})
        robot_desc = doc.toprettyxml(indent='  ')
        return [SetLaunchConfiguration('robot_description', robot_desc)]

    return LaunchDescription(
        [
            DeclareLaunchArgument('prefix', default_value='', description='Robot prefix'),
            DeclareLaunchArgument('namespace', default_value='', description='Namespace to push all topics to'),
            DeclareLaunchArgument(
                'joint_state_publisher_enabled',
                default_value='false',
                description='Enable to publish joint states using joint state publisher',
            ),
            OpaqueFunction(function=create_robot_description),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='both',
                parameters=[
                    {'use_sim_time': use_sim_time, 'robot_description': LaunchConfiguration('robot_description')}
                ],
                namespace=LaunchConfiguration('namespace'),
            ),
            Node(
                condition=IfCondition(LaunchConfiguration('joint_state_publisher_enabled')),
                package='joint_state_publisher',
                namespace=LaunchConfiguration('namespace'),
                executable='joint_state_publisher',
            ),
        ]
    )
