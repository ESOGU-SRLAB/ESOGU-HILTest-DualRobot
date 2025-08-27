# combined_robots.py - İki robot için birleşik konfigürasyon
from typing import List

# Combined planning group tanımlaması
MOVE_GROUP_ARM: str = "combinedgroup"  # MoveIt config'inizdeki combined group adı
MOVE_GROUP_GRIPPER: str = "grippers"  # İki gripper'ı da içeren group (varsa)

# Robot prefixleri
ROBOT1_PREFIX: str = "robot1_ur10e_"
ROBOT2_PREFIX: str = "robot2_ur10e_"

# Gripper pozisyonları (her iki robot için)
OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04, 0.04, 0.04]  # robot1 + robot2
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0, 0.0, 0.0]  # robot1 + robot2

def joint_names() -> List[str]:
    """Combined group için tüm joint'ler (robot1 + robot2)"""
    robot1_joints = [
        ROBOT1_PREFIX + "shoulder_pan_joint",
        ROBOT1_PREFIX + "shoulder_lift_joint", 
        ROBOT1_PREFIX + "elbow_joint",
        ROBOT1_PREFIX + "wrist_1_joint",
        ROBOT1_PREFIX + "wrist_2_joint",
        ROBOT1_PREFIX + "wrist_3_joint",
    ]
    
    robot2_joints = [
        ROBOT2_PREFIX + "shoulder_pan_joint",
        ROBOT2_PREFIX + "shoulder_lift_joint",
        ROBOT2_PREFIX + "elbow_joint", 
        ROBOT2_PREFIX + "wrist_1_joint",
        ROBOT2_PREFIX + "wrist_2_joint",
        ROBOT2_PREFIX + "wrist_3_joint",
    ]
    
    # Combined group için sıralama MoveIt config'inizle eşleşmeli
    return robot1_joints + robot2_joints

def robot1_joint_names() -> List[str]:
    """Sadece robot1 joint'leri"""
    return [
        ROBOT1_PREFIX + "shoulder_pan_joint",
        ROBOT1_PREFIX + "shoulder_lift_joint",
        ROBOT1_PREFIX + "elbow_joint",
        ROBOT1_PREFIX + "wrist_1_joint", 
        ROBOT1_PREFIX + "wrist_2_joint",
        ROBOT1_PREFIX + "wrist_3_joint",
    ]

def robot2_joint_names() -> List[str]:
    """Sadece robot2 joint'leri"""
    return [
        ROBOT2_PREFIX + "shoulder_pan_joint",
        ROBOT2_PREFIX + "shoulder_lift_joint",
        ROBOT2_PREFIX + "elbow_joint",
        ROBOT2_PREFIX + "wrist_1_joint",
        ROBOT2_PREFIX + "wrist_2_joint", 
        ROBOT2_PREFIX + "wrist_3_joint",
    ]

def base_link_name() -> str:
    """Combined group için base link (genelde ilk robot'un base'i)"""
    return ROBOT1_PREFIX + "base_link"

def robot1_base_link_name() -> str:
    return ROBOT1_PREFIX + "base_link"

def robot2_base_link_name() -> str:
    return ROBOT2_PREFIX + "base_link"

def end_effector_name() -> str:
    """Combined için varsayılan end effector (robot1)"""
    return ROBOT1_PREFIX + "tool0"

def robot1_end_effector_name() -> str:
    return ROBOT1_PREFIX + "tool0"

def robot2_end_effector_name() -> str:
    return ROBOT2_PREFIX + "tool0"

def gripper_joint_names() -> List[str]:
    """Combined gripper joint'leri (varsa)"""
    # Gripper joint'leriniz varsa buraya ekleyin
    return []
