import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the robot configuration
    # Replace 'my_robot_moveit_config' with your actual moveit config package
    moveit_config = (
        MoveItConfigsBuilder("my_robot")
        .robot_description(file_path="config/my_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 2. Start the MTC Action Server node
    mtc_node = Node(
        package="my_mtc_project",
        executable="mtc_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True} # Set to False if on real hardware
        ],
    )

    # 3. Optional: Static Transform Publisher 
    # (Only if your robot description doesn't define a world link)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0", "0", "0", "world", "base_link"],
    )

    return LaunchDescription([
        static_tf,
        mtc_node,
    ])