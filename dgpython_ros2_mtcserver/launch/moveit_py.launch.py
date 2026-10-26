#########################################################################
# AdaptiveNDE Project                                                   #
# dgpython_ros2_mtcserver Package                                       #
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
# File:  ur_moveit.launch.py
# Authors: Denis Stogl (PickNik), Tyler Lesthaeghe, Liam Murray
# Created: 2023-04-21
# Description:
#   Launch File for MoveIt with UR Robot
# Derived From:
#   https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
#       Adapted from ur_moveit.launch.py
#       under BSD-3-Clause
#########################################################################
# Revision Log
#
# 2023-04-21:  Initial Commit
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

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
    # The arm's own measured geometry. None means the site does not ship one
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


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    tf_prefix = LaunchConfiguration("tf_prefix")
    ur_type = LaunchConfiguration("ur_type")
    headless_mode = LaunchConfiguration("headless_mode")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    launch_d415 = LaunchConfiguration("launch_d415")
    launch_azure = LaunchConfiguration("launch_azure")
    azure_bracket = LaunchConfiguration("azure_bracket")
    launch_keyence = LaunchConfiguration("launch_keyence")
    launch_ut = LaunchConfiguration("launch_ut")
    launch_zed = LaunchConfiguration("launch_zed")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_ignition = LaunchConfiguration("sim_ignition")
 
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tari_package = LaunchConfiguration("tari_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    tooling = LaunchConfiguration("tooling")
    sensor_sn = LaunchConfiguration("sensor_sn")
    

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    # THE SITE'S MEASURED CALIBRATION, and every node on the robot has to render
    # with the same one. Hardcoding UR nominal values here while
    # ur_moveit.launch.py picks the site's etc/calibration.yaml makes move_group
    # and this node describe two different arms: move_group's FK of the current
    # joint values and the TF tree then disagree by about 1 mm at probe_end, and
    # that is exactly the error ros.Move() accumulates per call -- it commands a
    # TF-reported pose that move_group's model cannot reach, lands a millimetre
    # away, and the next command is taken from there.
    #
    # Falls back to UR nominal only when the site ships no calibration.
    kinematics_params = _CELL_CALIBRATION or PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_client_library"), "resources", "external_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(tari_package), "urdf", description_file]),
            " ",
            "robot_ip:=",
            robot_ip,
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
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "use_tool_communication:=",
            use_tool_communication,
            " ",
            "tool_parity:=",
            tool_parity,
            " ",
            "tool_baud_rate:=",
            tool_baud_rate,
            " ",
            "tool_stop_bits:=",
            tool_stop_bits,
            " ",
            "tool_rx_idle_chars:=",
            tool_rx_idle_chars,
            " ",
            "tool_tx_idle_chars:=",
            tool_tx_idle_chars,
            " ",
            "tool_device_name:=",
            tool_device_name,
            " ",
            "tool_tcp_port:=",
            tool_tcp_port,
            " ",
            "tool_voltage:=",
            tool_voltage,
            " ",
            "reverse_ip:=",
            reverse_ip,
            " ",
            "script_command_port:=",
            script_command_port,
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
            launch_ut,
            " ",
            "launch_zed:=",
            launch_zed,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "sim_ignition:=",
            sim_ignition,

        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(tari_package), "srdf", moveit_config_file]
            ),
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
            launch_ut,
            " ",
            "launch_zed:=",
            launch_zed,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "sim_ignition:=",
            sim_ignition,
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # robot_description_kinematics = {
    #     "robot_description_kinematics": load_yaml(
    #         str(moveit_config_package.perform(context)), "config/kinematics.yaml"
    #     )
    # }
    
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # No octomap here: this node is not the occupancy map monitor and never
    # passed the config to anything. See ur_moveit.launch.py for why these
    # cells run without one at all.
    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("tari_ur_description", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    change_controllers = context.perform_substitution(use_fake_hardware)
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        # The bound MoveIt puts on getting the controller's RESULT back is
        # scaling*duration + margin, so on a near-zero-length move the scaling
        # term vanishes and the constant is the whole budget -- the SHORTER the
        # move, the tighter the deadline, which is backwards. At the stock 0.5 s
        # a scan's first approach fails with "Controller is taking too long to
        # execute trajectory (expected upper bound 0.500000 seconds)" while the
        # controller has already reported "Goal reached, success!" 7 ms in and
        # the result takes 815 ms to arrive. 5 s costs only a later report of a
        # genuinely stuck controller; a pass that goes wrong is already caught
        # by min_fraction and the scan's trace accounting.
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    mtc_node = Node(
        package="dgpython_ros2_mtcserver",
        executable="mtc_server",
        output="screen",
        # Pass the parameters from the UR moveit_config
        parameters=[
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        {"use_sim_time": use_sim_time},
        {"arm_group_name": "ur_manipulator"}
        ],
    )

    nodes_to_start = [
        mtc_node
        ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_array_handler",
            default_value="true",
            description="Enables the pose array handler if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20e"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
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
            "launch_ut",
            default_value="false",
            description="Launch flex ut setup",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_zed",
            default_value="false",
            description="Launch ZED v1 camera",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur_cell.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tari_package",
            default_value="tari_ur_description",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="tf_prefix of the joint names, useful for \
        multi-robot setup. If changed, also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sensor_sn",
            default_value="",
            description="Azure Kinect serial number",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if \
            use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, \
            if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, \
            if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. \
            The user has be be allowed to write to this location. \
            Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. \
            Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",
            description="Start Gazebo Classic Simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false",
            description="Start Gazebo Ignition Simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )
    declared_arguments.append(
        DeclareLaunchArgument("tooling",
                              default_value="ec",
                              description="End of Arm Tooling Choice (old/new/newer)",
                              choices=["old", "new", "ec", "rightangle_contact_ut", "immersion_ut"])
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
