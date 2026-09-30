from typing import List

MOVE_GROUP_ARM: str = "real_kawasaki"
MOVE_GROUP_GRIPPER: str = "gripper"

prefix: str = ""

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = prefix) -> List[str]:
    return [
        "world_to_agv",
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
    ]


def base_link_name(prefix: str = prefix) -> str:
    return prefix + "base_link"


def end_effector_name(prefix: str = prefix) -> str:
    return prefix + "joint6"


def gripper_joint_names(prefix: str = prefix) -> List[str]:
    return []
