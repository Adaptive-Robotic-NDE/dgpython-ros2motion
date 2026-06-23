from moveit.task_constructor import core, stages

def get_plan_to_state_stage(node, group_name, state_name):
    """Returns a Stage to move the arm to a named SRDF state."""
    sampling_planner = core.PipelinePlanner(node)
    
    # We use MoveTo (a Propagator) to reach a specific destination
    stage = stages.MoveTo(f"move_to_{state_name}", sampling_planner)
    stage.group = group_name
    stage.set_goal(state_name)
    
    return stage