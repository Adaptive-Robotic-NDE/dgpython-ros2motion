import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    try:
        with open(os.path.join(package_path, file_path), 'r') as file:
            return yaml.safe_load(file)
    except Exception:
        return {}

def launch_setup(context, *args, **kwargs):
    # 1. Resolve all LaunchConfigurations into actual strings
    # This is the "secret sauce" to make them MoveItConfigsBuilder friendly
    ur_type = context.perform_substitution(LaunchConfiguration("ur_type"))
    tooling = context.perform_substitution(LaunchConfiguration("tooling"))
    prefix = context.perform_substitution(LaunchConfiguration("prefix"))
    
    # 2. Collect all your Boolean/String mappings into one dict
    # This replaces those massive 'Command' blocks
    render_mappings = {
        "ur_type": ur_type,
        "name": "ur",
        "tf_prefix": prefix,
        "tooling": tooling,
        "robot_ip": "172.17.0.2",
        "safety_limits": "true",
        "safety_pos_margin": "0.15",
        "safety_k_position": "20",
        "launch_d415": context.perform_substitution(LaunchConfiguration("launch_d415")),
        "launch_azure": context.perform_substitution(LaunchConfiguration("launch_azure")),
        "launch_keyence": context.perform_substitution(LaunchConfiguration("launch_keyence")),
        "launch_zed": context.perform_substitution(LaunchConfiguration("launch_zed")),
        "launch_ut": "false",
    }

    # 3. Resolve the specific UR YAML paths manually
    ur_desc_path = get_package_share_directory("ur_description")
    tari_path = get_package_share_directory("tari_ur_launch")
    
    render_mappings.update({
        "joint_limit_params": os.path.join(ur_desc_path, "config", ur_type, "joint_limits.yaml"),
        "kinematics_params": os.path.join(tari_path, "etc", "ex-ur10e-1_calibration.yaml"),
        "physical_params": os.path.join(ur_desc_path, "config", ur_type, "physical_parameters.yaml"),
        "visual_params": os.path.join(ur_desc_path, "config", ur_type, "visual_parameters.yaml"),
    })

    # 4. Use the Builder (this handles URDF, SRDF, and OMPL config in one go)
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=os.path.join(tari_path, "urdf", "ur_withenvironment.urdf.xacro"),
            mappings=render_mappings
        )
        .robot_description_semantic(
            file_path=os.path.join(tari_path, "srdf", "ur.srdf.xacro"),
            mappings=render_mappings
        )
        .trajectory_execution(file_path="config/controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 5. Define the Node with the built config
    moveit_py_node = Node(
        package="dgpython_ros2_planningserver",
        executable="moveit_planner.py",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return [moveit_py_node]

def generate_launch_description():
    # Define your arguments here as you did before
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur10e"),
        DeclareLaunchArgument("tooling", default_value="newer"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("launch_d415", default_value="true"),
        DeclareLaunchArgument("launch_azure", default_value="false"),
        DeclareLaunchArgument("launch_keyence", default_value="false"),
        DeclareLaunchArgument("launch_zed", default_value="false"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])