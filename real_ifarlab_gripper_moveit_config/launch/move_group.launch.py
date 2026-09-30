from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("whole_cell_hw", package_name="real_ifarlab_gripper_moveit_config").planning_pipelines(
        pipelines=["ompl", "pilz_industrial_motion_planner"],
        default_planning_pipeline="ompl",
    ).robot_description_semantic(file_path="config/whole_cell_hw.srdf").trajectory_execution(file_path="config/moveit_controllers.yaml").to_moveit_configs()
    return generate_move_group_launch(moveit_config)