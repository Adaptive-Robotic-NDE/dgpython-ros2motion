#########################################################################
# AdaptiveNDE Project                                                   #
# dgpython_ros2_actionserver Package                                    #
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
# 2026-09-29:  Hardening Complete Automated Pipeline
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
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from ur_moveit_config.launch_common import load_yaml

launch_d415 = LaunchConfiguration("launch_d415")
launch_azure = LaunchConfiguration("launch_azure")
azure_bracket = LaunchConfiguration("azure_bracket")
launch_keyence = LaunchConfiguration("launch_keyence")
launch_zed = LaunchConfiguration("launch_zed")
launch_ut = LaunchConfiguration("launch_keyence")
tooling = LaunchConfiguration("tooling")
prefix = LaunchConfiguration("prefix")
tf_prefix = LaunchConfiguration("tf_prefix")
ur_type = LaunchConfiguration("ur_type")

# WHICH CELL'S ROOM THIS PLANS AGAINST.
#
# ur_cell.urdf.xacro takes the environment as a file path and defaults to the
# BARE one: an arm bolted to `world`, no bench. Right for a description with no
# cell, wrong for a planner -- MoveIt would plan against a scene with no bench
# while Gazebo has one, and the arm would plan straight through it.
#
# Resolved at import from TARI_CELL_PACKAGE so it needs no launch argument
# threaded through three repositories. Set it to "" for the bare environment.
#
# The description itself must reach the node as ParameterValue(..., str): a
# Command() substitution handed straight to a Node parameter is auto-typed by
# launch, which parses it as YAML. A 31 kB URDF is not YAML, so it fails with
# "Unable to parse the value of parameter robot_description as yaml" at LAUNCH
# time, long after the xacro itself succeeded -- which points at the description
# rather than at the plumbing.

def _cell_env(cell_package):
    import os
    from ament_index_python.packages import (PackageNotFoundError,
                                             get_package_share_directory)
    shared = get_package_share_directory("tari_ur_description")
    urdf = os.path.join(shared, "urdf", "environments", "bare", "environment.urdf.xacro")
    srdf = os.path.join(shared, "srdf", "environments", "bare", "environment.srdf.xacro")
    # The arm's own measured geometry. None means the cell does not ship one
    # and the caller should fall back to UR nominal values.
    calib = None
    cell = (cell_package or "").strip()
    if cell:
        try:
            share = get_package_share_directory(cell)
        except PackageNotFoundError:
            return urdf, srdf, calib
        cu = os.path.join(share, "urdf", "environment.urdf.xacro")
        cs = os.path.join(share, "srdf", "environment.srdf.xacro")
        ck = os.path.join(share, "etc", "calibration.yaml")
        if os.path.exists(cu):
            urdf = cu
        if os.path.exists(cs):
            srdf = cs
        if os.path.exists(ck):
            calib = ck
    return urdf, srdf, calib


_ENV_URDF, _ENV_SRDF, _CELL_CALIBRATION = _cell_env(
    os.environ.get("TARI_CELL_PACKAGE", "tari_cell_ex_ur10e_1"))


def get_robot_description():


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    # THE CELL'S MEASURED CALIBRATION, and every node in the cell has to render
    # with the same one. Hardcoding UR nominal values here while
    # ur_moveit.launch.py picks the cell's etc/calibration.yaml makes move_group
    # and this node describe two different arms: move_group's FK of the current
    # joint values and the TF tree then disagree by about 1 mm at probe_end, and
    # that is exactly the error ros.Move() accumulates per call -- it commands a
    # TF-reported pose that move_group's model cannot reach, lands a millimetre
    # away, and the next command is taken from there.
    #
    # Falls back to UR nominal only when the cell ships no calibration.
    kinematics_params = _CELL_CALIBRATION or PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
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
            PathJoinSubstitution([FindPackageShare("tari_ur_description"), "urdf", "ur_cell.urdf.xacro"]),
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
            ur_type,
            " ",
            "tf_prefix:=",
            prefix,
            " ",
            "environment_file:=",
            _ENV_URDF,
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
            "azure_bracket:=",
            azure_bracket,
            " ",
            "launch_keyence:=",
            launch_keyence,
            " ",
            "launch_ut:=",
            "false",
            " ",
            "launch_zed:=",
            launch_zed,
        ]
    )


    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    return robot_description

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("tari_ur_description"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
            "environment_srdf:=",
            _ENV_SRDF,
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
            "azure_bracket:=",
            azure_bracket,
            " ",
            "launch_keyence:=",
            launch_keyence,
            " ",
            "launch_ut:=",
            "false",
            " ",
            "launch_zed:=",
            launch_zed,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }
    return robot_description_semantic


def generate_launch_description():
    # Octomap Configuration
    # octomap_config = {"sensors_3d": load_yaml("tari_ur_description", 'config/sensor_3d.yaml')}
    octomap_config = {'octomap_frame': 'azure_rgb', 
                      'octomap_resolution': 0.01,
                      'max_range': 5.0}

    octomap_updater_config = load_yaml('tari_ur_description', 'config/sensor_3d.yaml')
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    # The arm's solver, from tari_ur_description, NOT ur_moveit_config's stock
    # KDL -- see that file for the millimetre of error the stock one puts on
    # everything it solves. Flattened to dotted keys because a nested dict in a
    # launch parameter list gets its names mangled.
    #
    # The file is a full params file (/**: ros__parameters: ...), so the two
    # wrapper levels have to be stripped before flattening: flattening from the
    # top produces keys like "robot_description_kinematics./**.ros__parameters"
    # whose value is a dict, which ROS rejects.
    kinematics_yaml = load_yaml("tari_ur_description", "config/kinematics.yaml")
    for _wrapper in ("/**", "ros__parameters", "robot_description_kinematics"):
        if isinstance(kinematics_yaml, dict) and _wrapper in kinematics_yaml:
            kinematics_yaml = kinematics_yaml[_wrapper]
    robot_description_kinematics = {}
    for group_name, group_params in kinematics_yaml.items():
        for param_name, param_value in group_params.items():
            robot_description_kinematics[f"robot_description_kinematics.{group_name}.{param_name}"] = param_value

    demo_node = Node(
        package="dgpython_ros2_actionserver",
        executable="dgpython_action_server",
        name="dgpython_action_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            octomap_config,
            octomap_updater_config,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        #prefix=['xterm -e gdb -ex run --args']
    )
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )

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
            "azure_bracket",
            default_value="",
            description="Which bracket holds the Azure Kinect: ut, redo, none, or empty for the tooling's own default. See tari_ur_description/urdf/azure_bracket.urdf.xacro.",
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
        DeclareLaunchArgument(
            "launch_ut",
            default_value="false",
            description="Launch flex ut setup",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed, also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("tooling",
                              default_value="ec",
                              description="End of Arm Tooling Choice (old/new/newer)",
                              choices=["old", "new", "ec", "rightangle_contact_ut", "immersion_ut"])
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20e"],
        )
    )


    return launch.LaunchDescription(declared_arguments + [demo_node])
