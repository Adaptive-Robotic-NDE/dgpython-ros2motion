from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

def main():
    # Initialize MoveItPy for the UR arm
    # Ensure your node has access to the URDF/SRDF parameters
    ur_arm = MoveItPy(node_name="moveit_py_ur_node")
    arm_group = ur_arm.get_planning_component("ur_manipulator")

    # Set a goal (can be a Pose or Joint State)
    arm_group.set_goal_state(configuration_name="home")

    # Plan and execute
    plan_result = arm_group.plan()
    if plan_result:
        ur_arm.execute(plan_result.trajectory, blocking=True)