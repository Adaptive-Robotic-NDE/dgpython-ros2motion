from platform import node

from pandas import options

import rclpy
import rclcpp
from rclpy.node import Node
from rclpy.service import Service
from rclpy.action import ActionServer
from moveit.task_constructor import core, stages
from tari_interfaces.action import MtcTask
from std_srvs.srv import Trigger
from dgpython_ros2_mtcserver.tasks.move_to_state import get_plan_to_state_stage
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

def main():
    rclpy.init()
    rclcpp.init()
    options = rclcpp.NodeOptions()
    options.automatically_declare_parameters_from_overrides = True

    py_node = Node("mtc_py_node")
    cpp_node = rclcpp.Node("mtc_cpp_node", options=options)
    task = core.Task()
    py_node.declare_parameter("robot_description", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
    py_node.declare_parameter("robot_description_semantic", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

    has_param = py_node.has_parameter("robot_description")
    py_node.get_logger().info(f"Does node have robot_description? {has_param}")

    # Check for a specific joint's acceleration limit
    param_name = "robot_description_planning.joint_limits.shoulder_pan_joint.max_acceleration"

    if cpp_node.has_parameter(param_name):
        val = cpp_node.get_parameter(param_name).value
        py_node.get_logger().info(f"Verified: {param_name} = {val}")
    else:
        py_node.get_logger().warn(f"Parameter {param_name} NOT found on cpp_node!")

    task.loadRobotModel(cpp_node)
    py_node.get_logger().info("Task initialized successfully.")
        
    interpolation_planner = core.JointInterpolationPlanner()
    py_node.get_logger().info("Interpolation planner created successfully.")

    current_state = stages.CurrentState("current_state")
    task.add(current_state)

    stage_move_to_state = stages.MoveTo("over_table", interpolation_planner)
    stage_move_to_state.group = "ur_manipulator"
    stage_move_to_state.setGoal("over_table")
    task.add(stage_move_to_state)
    py_node.get_logger().info("Stage added successfully.")

    try:
        task.init()
        solution = task.plan()

        if solution:
            py_node.get_logger().info("Plan successful! Executing...")
            task.execute(solution)
            py_node.get_logger().info("Task executed successfully.")
        else:
            py_node.get_logger().error("Planning failed.")

    except Exception as e:
        py_node.get_logger().error(f"Error: {e}")

    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()