import numpy as np
from scipy.spatial.transform import Rotation as R

tip_words={
    "ROOT","JOINT","End"
}

def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
    """返回的是行列保持原样的数据"""
    joint_name = []
    joint_parent = []
    joint_offset = []
    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
        for i in range(len(lines)):
            if lines[i].startswith('Frame Time'):
                break
        motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            motion_data.append(np.array(data).reshape(1,-1))
        motion_data = np.concatenate(motion_data, axis=0)
    return motion_data


def part1_calculate_T_pose(bvh_file_path):
    """请填写以下内容
    输入： bvh 文件路径
    输出:
        joint_name: List[str]，字符串列表，包含着所有关节的名字
        joint_parent: List[int]，整数列表，包含着所有关节的父关节的索引,根节点的父关节索引为-1
        joint_offset: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的偏移量

    Tips:
        joint_name顺序应该和bvh一致
    """
    joint_names = []
    joint_parents = []
    joint_offsets = np.array([[0, 0, 0]])

    with open(bvh_file_path, 'r') as f:
        lines = f.readlines()
    # Find the ROOT joint
    root_idx = lines.index('ROOT RootJoint\n')
    # Parse the rest of the hierarchy structure
    joint_hierarchy = lines[root_idx+4:]

    # Initialize the stack with the ROOT joint
    stack = [('RootJoint', None)]
    # Pop the top joint from the stack
    joint_name, parent_name = stack.pop()

    # Add the joint to the dictionary
    joints = {}
    joints[joint_name] = parent_name

    joint_names.append('RootJoint')
    joint_parents.append(-1)

    # Find the child joints and add them to the stack
    for i in range(len(joint_hierarchy)):
        line = joint_hierarchy[i].strip()
        if (line.startswith("JOINT")) | (line.startswith("End Site")):
            if line.startswith("End Site"):
                child_name = f'{joint_name}_end'
            else:
                child_name = line.split()[1]

            joint_names.append(child_name)
            joint_parents.append(joint_names.index(joint_name))
            if line.startswith("JOINT"):
                stack.append((child_name, joint_name))
                joint_name=child_name
        elif line.startswith("}"):
            if len(stack)==0:
                joint_name='RootJoint'
            else:
                joint_name, parent_name=stack.pop()
        elif line.startswith("OFFSET"):
            # 使用字符串的 split() 方法将字符串按空格分割成多个部分
            parts = line.split()
            # 提取浮点数部分
            offset_values = [float(x) for x in parts[1:]]
            # 将偏移量添加到 joint_offset 中
            joint_offsets = np.append(joint_offsets, [offset_values], axis=0)
    return joint_names, joint_parents, joint_offsets


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    joint_positions = np.empty((len(joint_offset), 3))
    joint_orientations = np.empty((len(joint_offset), 4))
    idx_offset = 0
    
    for idx, offset in enumerate(joint_offset):
        cur_joint_name = joint_name[idx]
        parent_idx = joint_parent[idx]

        if cur_joint_name.startswith('RootJoint'):
            joint_positions[idx] = motion_data[frame_id, :3]

            frame_rot_euler = np.deg2rad(motion_data[frame_id, 3:6])
            rotation = R.from_euler('XYZ', frame_rot_euler)
            joint_orientations[idx] = rotation.as_quat()

        elif cur_joint_name.endswith('_end'):
            joint_positions[idx] = joint_positions[parent_idx] + np.array([joint_offset[idx]])

            frame_rot_euler = [0, 0, 0]
            rotation = R.from_euler('XYZ', frame_rot_euler)
            quaternion = rotation.as_quat()
            joint_orientations[idx] = quaternion * joint_orientations[parent_idx]

            idx_offset += 1

        else:
            joint_positions[idx] = joint_positions[parent_idx] + np.array([joint_offset[idx]])

            frame_rot_euler = np.deg2rad(motion_data[frame_id, 3*(idx-idx_offset):3*(idx-idx_offset+1)])
            rotation = R.from_euler('XYZ', frame_rot_euler)
            quaternion = rotation.as_quat()
            joint_orientations[idx] = quaternion * joint_orientations[parent_idx]

    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """
    motion_data = None
    return motion_data
