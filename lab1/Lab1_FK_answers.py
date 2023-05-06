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
        if (line.startswith("JOINT")) | (line.startswith("End")):
            if line.startswith("End"):
                child_name = f'{joint_name}_end'
            else:
                child_name = line.split()[1]

            joint_names.append(child_name)
            joint_parents.append(joint_names.index(joint_name))

            # if line.startswith("JOINT"):
            stack.append((child_name, joint_name))
            joint_name=child_name
        elif line.startswith("}"):
            if len(stack)==0:
                joint_name='RootJoint'
            else:
                tmp, joint_name=stack.pop()
        elif line.startswith("OFFSET"):
            # 使用字符串的 split() 方法将字符串按空格分割成多个部分
            parts = line.split()
            # 提取浮点数部分
            offset_values = [float(x) for x in parts[1:]]
            # 将偏移量添加到 joint_offset 中
            joint_offsets = np.append(joint_offsets, [offset_values], axis=0)
    return joint_names, joint_parents, joint_offsets


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    joint_positions = np.zeros((len(joint_offset), 3))
    joint_orientations = np.zeros((len(joint_offset), 4))
    idx_offset = 0
    
    for idx, offset in enumerate(joint_offset):
        cur_joint_name = joint_name[idx]
        parent_idx = joint_parent[idx]

        if cur_joint_name.startswith('RootJoint'):
            joint_positions[idx] = motion_data[frame_id, :3]
            joint_orientations[idx] = R.from_euler('XYZ', motion_data[frame_id, 3:6],degrees=True).as_quat()

            
        # elif (cur_joint_name.endswith('lHip') | cur_joint_name.endswith('lKnee')| cur_joint_name.endswith('lAnkle')| cur_joint_name.endswith('lToeJoint')):
        #     rotation = R.from_euler('XYZ', motion_data[frame_id, 3*(idx-idx_offset+1):3*(idx-idx_offset+2)],degrees=True).as_matrix()
        #     rot_matrix_p=R.from_quat(joint_orientations[parent_idx]).as_matrix()
        #     tmp = rot_matrix_p.dot(rotation)
        #     joint_orientations[idx]=R.from_matrix(tmp).as_quat()
        #     rot_matrix=R.from_quat(joint_orientations[idx]).as_matrix()
        #     joint_positions[idx] = joint_positions[parent_idx]+rot_matrix_p.dot(offset)



        elif cur_joint_name.endswith('_end'):
            q_result = joint_orientations[parent_idx] * np.concatenate(([0], offset)) * joint_orientations[parent_idx].conj()
            joint_positions[idx] = joint_positions[parent_idx]+q_result[1:]
            idx_offset += 1

        else:
            rotation = R.from_euler('XYZ', motion_data[frame_id, 3*(idx-idx_offset+1):3*(idx-idx_offset+2)],degrees=True).as_matrix()
            rot_matrix_p=R.from_quat(joint_orientations[parent_idx]).as_matrix()
            tmp = rot_matrix_p.dot(rotation)
            joint_orientations[idx]=R.from_matrix(tmp).as_quat()
            rot_matrix=R.from_quat(joint_orientations[idx]).as_matrix()
            joint_positions[idx] = joint_positions[parent_idx]+rot_matrix_p.dot(offset)


            

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
