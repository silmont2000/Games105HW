import numpy as np
from scipy.spatial.transform import Rotation as R


def rotation_matrix(a, b):
    # 计算旋转轴
    n = np.cross(a, b)
    # 计算夹角
    cos_theta = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
    sin_theta = np.linalg.norm(n) / (np.linalg.norm(a) * np.linalg.norm(b))
    theta = np.arctan2(sin_theta, cos_theta)
    # 构造旋转矩阵
    c = np.cos(theta)
    s = np.sin(theta)
    v = 1 - c
    rotation_matrix = np.array([[n[0]*n[0]*v+c, n[0]*n[1]*v-n[2]*s, n[0]*n[2]*v+n[1]*s],
                                 [n[0]*n[1]*v+n[2]*s, n[1]*n[1]*v+c, n[1]*n[2]*v-n[0]*s],
                                 [n[0]*n[2]*v-n[1]*s, n[1]*n[2]*v+n[0]*s, n[2]*n[2]*v+c]])
    return rotation_matrix

def part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose):
    """
    完成函数，计算逆运动学
    输入: 
        meta_data: 为了方便，将一些固定信息进行了打包，见上面的meta_data类
        joint_positions: 当前的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 当前的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
        target_pose: 目标位置，是一个numpy数组，shape为(3,)
    输出:
        经过IK后的姿态
        joint_positions: 计算得到的关节位置，是一个numpy数组，shape为(M, 3)，M为关节数
        joint_orientations: 计算得到的关节朝向，是一个numpy数组，shape为(M, 4)，M为关节数
    """
    path,path_name,path1,path2=meta_data.get_path_from_root_to_end()
    path_end=path1[0]+1 ## lWrist_end
    for idx in range(0,len(path1)):
        path_joint_id=path1[idx]
        next_joint_id=0
        if idx==0:
            next_joint_id=path_end
        else:
            next_joint_id=path1[idx-1]

        vec_to_end=joint_positions[path_joint_id]-joint_positions[path_end]
        vec_to_target=joint_positions[path_joint_id]-target_pose
        rot_matrix=rotation_matrix(vec_to_target,vec_to_end)

        initial_orientation=R.from_quat(joint_orientations[path_joint_id]).as_matrix()
        rot_matrix_R=R.from_matrix(rotation_matrix(vec_to_end,vec_to_target)).as_matrix()
        calculated_orientation=rot_matrix_R.dot(initial_orientation)
        joint_orientations[path_joint_id]=R.from_matrix(calculated_orientation).as_quat()

        if idx==1:
            vec_to_next=joint_positions[next_joint_id]-joint_positions[path_joint_id]
            calculated_vec_to_next=rot_matrix.dot(vec_to_next)
            joint_positions[next_joint_id]=calculated_vec_to_next+joint_positions[path_joint_id]

    return joint_positions, joint_orientations

def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    """
    输入lWrist相对于RootJoint前进方向的xz偏移，以及目标高度，IK以外的部分与bvh一致
    """
    
    return joint_positions, joint_orientations

def bonus_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    """
    输入左手和右手的目标位置，固定左脚，完成函数，计算逆运动学
    """
    
    return joint_positions, joint_orientations