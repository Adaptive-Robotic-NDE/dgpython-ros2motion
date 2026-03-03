import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import moveit_task_constructor_core
from dgpython_ros2_mtcserver.action import MtcTask
from dgpython_ros2_mtcserver.tasks.move_to_state import get_plan_to_state_stage

class MtcActionServer(Node):
    def __init__(self):
        super().__init__('mtc_action_server')
        self._action_server = ActionServer(
            self, MtcTask, 'mtc_task_action', self.execute_callback)
        self.get_logger().info("MTC Action Server Ready")

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        
        # 1. Initialize the Top-Level Task
        task = moveit_task_constructor_core.core.Task()
        task.load_robot_model(self)
        
        # 2. Add Mandatory Start Stage
        task.add(moveit_task_constructor_core.stages.CurrentState("start_state"))
        
        # 3. Add Stages based on the Request
        if request.task_type == "move_to_state":
            stage = get_plan_to_state_stage(self, request.group_name, request.target_state)
            task.add(stage)
        
        # 4. Plan and Execute
        result = MtcTask.Result()
        if task.plan():
            self.get_logger().info("Plan successful! Executing...")
            task.execute(task.solutions[0])
            goal_handle.succeed()
            result.success = True
        else:
            self.get_logger().error("Planning failed.")
            goal_handle.abort()
            result.success = False
            
        return result

def main():
    rclpy.init()
    node = MtcActionServer()
    rclpy.spin(node)