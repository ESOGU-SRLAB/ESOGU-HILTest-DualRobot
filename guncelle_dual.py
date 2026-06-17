#!/usr/bin/env python3
import json
import os

def process_ur_trajectories(input_file, output_file):
    if not os.path.exists(input_file):
        print(f"Skipping UR update: {input_file} not found.")
        return

    with open(input_file, 'r') as f:
        data = json.load(f)

    for traj_name, traj_info in data.items():
        original_joints = traj_info.get('joint_names', [])
        new_joints = []
        for name in original_joints:
            # e.g., sim_shoulder_pan_joint -> shoulder_pan_joint
            if name.startswith("sim_"):
                new_name = name[4:]
            else:
                new_name = name
            new_joints.append(new_name)
        traj_info['joint_names'] = new_joints
    
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Converted {input_file} -> {output_file}")


def process_kawa_trajectories(input_file, output_file):
    if not os.path.exists(input_file):
        print(f"Skipping Kawasaki update: {input_file} not found.")
        return

    with open(input_file, 'r') as f:
        data = json.load(f)

    for traj_name, traj_info in data.items():
        original_joints = traj_info.get('joint_names', [])
        new_joints = []
        for name in original_joints:
            if name == "world_to_agv":
                new_joints.append(name)
            elif name.startswith("sim_kawasaki_"):
                new_joints.append(name.replace("sim_kawasaki_", ""))
            else:
                new_joints.append(name)
        traj_info['joint_names'] = new_joints
    
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Converted {input_file} -> {output_file}")


def main():
    ur_in = "sim_ur_trajectories.json"
    ur_out = "real_ur_trajectories.json"
    
    kawa_in = "sim_kawa_trajectories.json"
    kawa_out = "real_kawa_trajectories.json"
    
    print("=== Processing Trajectories ===")
    process_ur_trajectories(ur_in, ur_out)
    process_kawa_trajectories(kawa_in, kawa_out)
    print("Done!")

if __name__ == "__main__":
    main()
