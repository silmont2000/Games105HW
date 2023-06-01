import numpy as np
from scipy.spatial.transform import Rotation as R

tip_words={
    "ROOT","JOINT","End"
}

def load_single_frame_motion_data(bvh_file_path):
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
        single_frame_motion_data = []
        for line in lines[i+1:]:
            data = [float(x) for x in line.split()]
            if len(data) == 0:
                break
            single_frame_motion_data.append(np.array(data).reshape(1,-1))
        single_frame_motion_data = np.concatenate(single_frame_motion_data, axis=0)
    return single_frame_motion_data


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


def part2_forward_kinematics(joint_name, joint_parent, joint_offset, single_frame_motion_data, frame_id):
    """请填写以下内容
    输入: part1 获得的关节名字，父节点列表，偏移量列表
        single_frame_motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数
        frame_id: int，需要返回的帧的索引
    输出:
        joint_positions: np.ndarray，形状为(M, 3)的numpy数组，包含着所有关节的全局位置
        joint_orientations: np.ndarray，形状为(M, 4)的numpy数组，包含着所有关节的全局旋转(四元数)
    Tips:
        1. joint_orientations的四元数顺序为(x, y, z, w)
        2. from_euler时注意使用大写的XYZ
    """
    # 初始化
    joint_positions = np.zeros((len(joint_offset), 3))
    joint_orientations = np.zeros((len(joint_offset), 4))
    idx_offset = 0
    # 遍历offset，也就是便利所有节点
    for idx, offset in enumerate(joint_offset):
        cur_joint_name = joint_name[idx]
        parent_idx = joint_parent[idx]

        if cur_joint_name.startswith('RootJoint'):
            # 根节点，多三个position数据
            joint_positions[idx] = single_frame_motion_data[frame_id, :3]
            joint_orientations[idx] = R.from_euler('XYZ', single_frame_motion_data[frame_id, 3:6],degrees=True).as_quat()
        elif cur_joint_name.endswith('_end'):
            q_result = joint_orientations[parent_idx] * np.concatenate(([0], offset)) * joint_orientations[parent_idx].conj()
            joint_positions[idx] = joint_positions[parent_idx]+q_result[1:]
            idx_offset += 1
        else:
            # 普通节点
            # rotation是它自己的旋转
            rotation = R.from_euler('XYZ', single_frame_motion_data[frame_id, 3*(idx-idx_offset+1):3*(idx-idx_offset+2)],degrees=True).as_matrix()
            # rot_matrix_p是它父节点的朝向，因为取的已经是joint_orientations而不是single_frame_motion_data了
            rot_matrix_p=R.from_quat(joint_orientations[parent_idx]).as_matrix()
            # tmp是它自己的「朝向」
            tmp = rot_matrix_p.dot(rotation)
            # 存进朝向list
            joint_orientations[idx]=R.from_matrix(tmp).as_quat()
            # 位置，要在父节点位置的基础上，在父节点的坐标系下计算偏移
            joint_positions[idx] = joint_positions[parent_idx]+rot_matrix_p.dot(offset)

    return joint_positions, joint_orientations


def part3_retarget_func(T_pose_bvh_path, A_pose_bvh_path):
    """
    将 A-pose的bvh重定向到T-pose上
    输入: 两个bvh文件的路径
    输出: 
        single_frame_motion_data: np.ndarray，形状为(N,X)的numpy数组，其中N为帧数，X为Channel数。retarget后的运动数据
    Tips:
        两个bvh的joint name顺序可能不一致哦(
        as_euler时也需要大写的XYZ
    """
    A_data=load_single_frame_motion_data(A_pose_bvh_path)
    T_joint_names, _, _=part1_calculate_T_pose(T_pose_bvh_path)
    A_joint_names, _, _=part1_calculate_T_pose(A_pose_bvh_path)
    # 想了一下，感觉这里肯定是逆向乘回去，但为什么正负号和我理解的不一样，应该是因为定义顺逆时针时，模型是背面朝着我的（也就是和我同一个方向），或者这是逆矩阵刚好符号变了
    # 总之正负都试一试
    lShoulderMatrix= R.from_euler('XYZ', [0,0,-45],degrees=True).as_matrix()
    rShoulderMatrix= R.from_euler('XYZ', [0,0,45],degrees=True).as_matrix()
    # 因为是根据T的拿A的，中间需要掠过的end数不一定相等
    A_name_skip_cnt=[]
    ignore_cnt=0
    for joint_name in A_joint_names:
        if joint_name.endswith('_end'):
                ignore_cnt=ignore_cnt+1
        A_name_skip_cnt.append(ignore_cnt)
    motion_data = []
    # 不能用_data的shape呀！要播放的是A_data
    for frame_id in range(A_data.shape[0]):
        single_frame_motion_data = []
        for joint_name in T_joint_names:
            A_pose_index=A_joint_names.index(joint_name)
            if joint_name=="RootJoint":
                # rootJoint
                single_frame_motion_data.append(A_data[frame_id,:3])
                single_frame_motion_data.append(A_data[frame_id,3:6])
            elif joint_name=="lShoulder":
                original_data=R.from_euler('XYZ', A_data[frame_id,(A_pose_index-A_name_skip_cnt[A_pose_index]+1)*3:(A_pose_index-A_name_skip_cnt[A_pose_index]+2)*3],degrees=True).as_matrix()
                result_data=original_data.dot(lShoulderMatrix)
                single_frame_motion_data.append(R.from_matrix(result_data).as_euler('XYZ', degrees=True))
            elif joint_name=="rShoulder":
                original_data=R.from_euler('XYZ', A_data[frame_id,(A_pose_index-A_name_skip_cnt[A_pose_index]+1)*3:(A_pose_index-A_name_skip_cnt[A_pose_index]+2)*3],degrees=True).as_matrix()
                result_data=original_data.dot(rShoulderMatrix)
                single_frame_motion_data.append(R.from_matrix(result_data).as_euler('XYZ', degrees=True))
            elif joint_name.endswith('_end'):
                continue
            else:
                single_frame_motion_data.append(A_data[frame_id,(A_pose_index-A_name_skip_cnt[A_pose_index]+1)*3:(A_pose_index-A_name_skip_cnt[A_pose_index]+2)*3])
        single_frame_motion_data = np.concatenate(single_frame_motion_data, axis=0)
        motion_data.append(single_frame_motion_data)
    return np.array(motion_data)
