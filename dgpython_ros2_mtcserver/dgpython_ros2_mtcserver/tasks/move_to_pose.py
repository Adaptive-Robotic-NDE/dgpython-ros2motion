from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped

def create_detection_move_task(node, group_name, target_pose: PoseStamped):
    """
    Factory function to create a sub-task for moving to a detected object.
    """
    task = core.Task()
    task.load_robot_model(node)
    
    # 1. Start from where we are
    task.add(stages.CurrentState("current_state"))
    
    # 2. Define the Solver
    sampling_planner = core.PipelinePlanner(node)
    
    # 3. Move to the Pose (Propagator)
    # We use 'MoveTo' because we have a specific target goal
    move_to = stages.MoveTo("move_to_yolo_pose", sampling_planner)
    move_to.group = group_name
    move_to.set_goal(target_pose)
    
    task.add(move_to)
    
    return task