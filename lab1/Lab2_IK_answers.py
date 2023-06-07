import numpy as np
from scipy.spatial.transform import Rotation as R
import copy

def rotation_matrix(a, b):
    a=a/np.linalg.norm(a)
    b=b/np.linalg.norm(b)
    n = np.cross(a, b)
    # 旋转矩阵是正交矩阵，矩阵的每一行每一列的模，都为1；并且任意两个列向量或者任意两个行向量都是正交的。
    # n=n/np.linalg.norm(n)
    # 计算夹角
    cos_theta = np.dot(a, b)
    sin_theta = np.linalg.norm(n)
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
    parent_idx=meta_data.joint_parent

    # local_rotation是用于最后计算不在链上的节点
    no_caled_orientation=copy.deepcopy(joint_orientations)
    local_rotation = [R.from_quat(joint_orientations[parent_idx[i]]).inv() * R.from_quat(joint_orientations[i]) for i
                      in range(len(joint_orientations))]
    local_rotation[0] = R.from_quat(joint_orientations[0])
    local_position = [joint_positions[i]-joint_positions[parent_idx[i]] for i
                      in range(len(joint_orientations))]
    local_position[0] = joint_positions[0]

    path_end_id=path1[0] ## lWrist_end 就是手掌 只是加了end不叫hand而已
    for k in range(0,300):
        # k：循环次数
        for idx in range(0,len(path1)):
            # idx：路径上的第几个节点了，第0个是手，最后一个是root
            path_joint_id=path1[idx]

            vec_to_end=joint_positions[path_end_id]-joint_positions[path_joint_id]
            vec_to_target=target_pose-joint_positions[path_joint_id]
            # 获取end->target的旋转矩阵
            # debug
            # rot_matrix=rotation_matrix(np.array([1,0,0]),np.array([1,1,0]))
            rot_matrix=rotation_matrix(vec_to_end,vec_to_target)

            # 计算前的朝向。这个朝向实际上是累乘到父节点的
            initial_orientation=R.from_quat(joint_orientations[path_joint_id]).as_matrix()
            # 旋转矩阵，格式换算
            rot_matrix_R=R.from_matrix(rot_matrix).as_matrix()
            # 计算后的朝向
            calculated_orientation=rot_matrix_R.dot(initial_orientation)
            # 写回结果列表
            joint_orientations[path_joint_id]=R.from_matrix(calculated_orientation).as_quat()

            # 子节点的朝向也会有所变化
            # idx-1 就是当前节点的下一个更接近尾端的节点，一直向前迭代到1
            for i in range(idx-1,0,-1):
                path_joint_id=path1[i]
                # 遍历路径后的节点,都乘上旋转
                joint_orientations[path_joint_id]=R.from_matrix(rot_matrix_R.dot(R.from_quat(joint_orientations[path_joint_id]).as_matrix())).as_quat()

            path_joint_id=path1[idx]
            # 修改子节点的位置
            for i in range(idx-1,-1,-1):
                # path_joint_id=path1[i]
                # 节点id
                next_joint_id=path1[i]
                # 指向下个节点的向量
                vec_to_next=joint_positions[next_joint_id]-joint_positions[path_joint_id]
                # 左乘，改变向量
                calculated_vec_to_next_dir=rot_matrix.dot(vec_to_next)
                # 防止长度不对
                calculated_vec_to_next=calculated_vec_to_next_dir/np.linalg.norm(calculated_vec_to_next_dir)*np.linalg.norm(vec_to_next)
                # 还原回去
                joint_positions[next_joint_id]=calculated_vec_to_next+joint_positions[path_joint_id]
        joint_orientations[path_end_id]=joint_orientations[path1[1]]
        cur_dis=np.linalg.norm(joint_positions[path_end_id]-target_pose)
        if cur_dis<0.01:
            break
    print("距离",cur_dis,"迭代了",k,"次")
    for k in range(len(joint_orientations)):
        if k in  path:
            pass
        elif k==0:
            # 要单独处理，不然跟节点的-1就会变成从最后一个节点开始算
            pass
        else:
            local_rot_matrix=local_rotation[k].as_matrix()
            parent_rot_matrix=R.from_quat(joint_orientations[parent_idx[k]]).as_matrix()
            re=local_rot_matrix.dot(parent_rot_matrix)
            joint_orientations[k]=R.from_matrix(re).as_quat()

            initial_o=R.from_quat(no_caled_orientation[parent_idx[k]]).as_matrix()
            delta_orientation = np.dot(re, np.linalg.inv(initial_o))

            joint_positions[k]=joint_positions[parent_idx[k]]+delta_orientation.dot(local_position[k])

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
    