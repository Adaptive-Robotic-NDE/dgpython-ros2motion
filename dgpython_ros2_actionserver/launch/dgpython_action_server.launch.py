#########################################################################
# AdaptiveNDE Project                                                   #
# tari-ROS Package                                                      # 
#########################################################################
# Copyright 2023-2024 University of Dayton                              #
#                                                                       #
# Redistribution and use in source and binary forms, with or without    # 
# modification, are permitted provided that the following conditions    #
# are met:                                                              #
#                                                                       # 
# 1. Redistributions of source code must retain the above copyright     #
#    notice, this list of conditions and the following disclaimer.      #
# 2. Redistributions in binary form must reproduce the above copyright  #
#    notice, this list of conditions and the following disclaimer in    #
#    the documentation and/or other materials provided with the         #
#    distribution.                                                      #
# 3. Neither the name of the copyright holder nor the names of its      #
#    contributors may be used to endorse or promote products derived    #
#    from this software without specific prior written permission.      #
#                                                                       #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
# “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR #
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  #
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,#
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      #
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, #
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY #
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   #
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE #
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  #
#########################################################################
# This material is based in part upon work supported by the U.S. Air    #
# Force under Contract FA8650-19-D-5230.                                #
#                                                                       #
# DISTRIBUTION A. Approved for public release: distribution unlimited.  #
#########################################################################
# File:  dgpython_action_server.launch.py                                                     
# Authors: Denis Stogl (PickNik), Tyler Lesthaeghe, Liam Murray
# Created: 2023-06-22
# Description:
#   Launch File for Dataguzzler/MoveIt Bridge Action Server
# Derived From:                                                         
#   https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
#       Adapted from ur_moveit.launch.py
#       under BSD-3-Clause                              
#########################################################################
# Revision Log                                                          
# 
# 2023-06-22:  Initial Commit
# 2023-11-14:  Modifications for UR Driver Changes
# 
#########################################################################
# Development Notes
#   
# 
#########################################################################

# Copyright (c) 2021 PickNik, Inc.
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
#    * Neither the name of the {copyright_holder} nor the names of its
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
# Author: Denis Stogl

import launch
import os
import sys

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

launch_d415 = LaunchConfiguration("launch_d415")
launch_azure = LaunchConfiguration("launch_azure")
launch_keyence = LaunchConfiguration("launch_keyence")
launch_zed = LaunchConfiguration("launch_zed")
tooling = LaunchConfiguration("tooling")

def get_robot_description():


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("tari_ur_launch"), "etc", "", "ex-ur10e-1_calibration.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur10e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("tari_ur_launch"), "urdf", "ur_withenvironment.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur10e",
            " ",
            "prefix:=",
            "",
            " ",
            "tooling:=",
            tooling,
            " ",
            "launch_d415:=",
            launch_d415,
            " ",
            "launch_azure:=",
            launch_azure,
            " ",
            "launch_keyence:=",
            launch_keyence,
            " ",
            "launch_zed:=",
            launch_zed,
        ]
    )


    robot_description = {"robot_description": robot_description_content}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("tari_ur_launch"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            "",
            " ",
            "tooling:=",
            "new",
            " ",
            "launch_d415:=",
            launch_d415,
            " ",
            "launch_azure:=",
            launch_azure,
            " ",
            "launch_keyence:=",
            launch_keyence,
            " ",
            "launch_zed:=",
            launch_zed,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic
    
def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    demo_node = Node(
        package="dgpython_ros2_actionserver",
        executable="dgpython_action_server",
        name="dgpython_action_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
        #prefix=['xterm -e gdb -ex run --args']
    )
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_d415",
            default_value="true",
            description="Launch realsense D415 camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_azure",
            default_value="false",
            description="Launch Azure Kinect camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_keyence",
            default_value="false",
            description="Launch Keyence laser profiler",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_zed",
            default_value="false",
            description="Launch ZED Stereo camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("tooling",
                              default_value="new",
                              description="End of Arm Tooling Choice (old/new)",
                              choices=["old", "new"])
    )


    return launch.LaunchDescription(declared_arguments + [demo_node])
