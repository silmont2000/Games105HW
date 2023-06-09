import numpy as np
from scipy.spatial.transform import Rotation as R
import copy


def load_motion_data(bvh_file_path):
    """part2 辅助函数，读取bvh文件"""
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

def inv_safe(data):
    # return R.from_quat(data).inv()
    if np.allclose(data, [0, 0, 0, 0]):
        return np.eye(3)
    else:
        return np.linalg.inv(R.from_quat(data).as_matrix())
    
def from_quat_safe(data):
    # return R.from_quat(data)
    if np.allclose(data, [0, 0, 0, 0]):
        return np.eye(3)
    else:
        return R.from_quat(data).as_matrix()

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
    local_rotation = [
        R.from_matrix(inv_safe(joint_orientations[parent_idx[i]]) * from_quat_safe(joint_orientations[i])).as_quat() for i
                      in range(len(joint_orientations))]
    local_rotation[0] = R.from_matrix(from_quat_safe(joint_orientations[0])).as_quat()
    local_position = [joint_positions[i]-joint_positions[parent_idx[i]] for i
                      in range(len(joint_orientations))]
    local_position[0] = joint_positions[0]

    path_end_id=path1[0] ## lWrist_end 就是手掌 只是加了end不叫hand而已
    for k in range(0,300):
        # k：循环次数
        # 正向的，path1是从手到root之前
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
            initial_orientation=from_quat_safe(joint_orientations[path_joint_id])
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
                joint_orientations[path_joint_id]=R.from_matrix(rot_matrix_R.dot(from_quat_safe(joint_orientations[path_joint_id]))).as_quat()

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
        
        # path2是从脚到root，所以要倒着
        # debug
        # for idx in range(len(path2)-1,len(path2)-3,-1): # len(path2)-1 --> 0
        for idx in range(len(path2)-1,0,-1): # len(path2)-1 --> 0
            path_joint_id=path2[idx]
            parient_joint_id=max(parent_idx[path_joint_id],0)

            vec_to_end=joint_positions[path_end_id]-joint_positions[path_joint_id]
            vec_to_target=target_pose-joint_positions[path_joint_id]
            # 获取end->target的旋转矩阵
            # debug
            # rot_matrix=rotation_matrix(np.array([0.72,0.35,0]),np.array([0.5,0.35,0]))
            # rot_matrix=np.linalg.inv(rot_matrix)
            rot_matrix=rotation_matrix(vec_to_end,vec_to_target)

            # 计算前的朝向。注意path2是反方向的，要改父节点才行
            initial_orientation=from_quat_safe(joint_orientations[path_joint_id])
            # 旋转矩阵，格式换算
            rot_matrix_R= R.from_matrix(rot_matrix).as_matrix()
            # 计算后的朝向
            calculated_orientation=rot_matrix_R.dot(initial_orientation)
            # 写回结果列表
            joint_orientations[path_joint_id]=R.from_matrix(calculated_orientation).as_quat()

            # 其他节点的朝向也会有所变化
            for i in range(idx+1,len(path2)):
                path_joint_id=path2[i] 
                joint_orientations[path_joint_id]=R.from_matrix(rot_matrix_R.dot(from_quat_safe(joint_orientations[path_joint_id]))).as_quat()

            # idx-1 就是当前节点的下一个更接近尾端的节点，一直向前迭代到1
            for i in range(len(path1)-1,0,-1):
                path_joint_id=path1[i]
                # 遍历路径后的节点,都乘上旋转
                joint_orientations[path_joint_id]=R.from_matrix(rot_matrix_R.dot(from_quat_safe(joint_orientations[path_joint_id]))).as_quat()

            path_joint_id=path2[max(idx-1,0)]
            # 修改父节点，或者说更靠近手的那些节点的位置
            # path2上的
            for i in range(idx,len(path2)):
                # path_joint_id=path1[i]
                # 节点id
                prev_joint_id=path2[i]
                # 指向上一个节点的向量
                vec_to_next=joint_positions[prev_joint_id]-joint_positions[path_joint_id]
                # 左乘，改变向量
                calculated_vec_to_next_dir=rot_matrix.dot(vec_to_next)
                # 防止长度不对
                calculated_vec_to_next=calculated_vec_to_next_dir/np.linalg.norm(calculated_vec_to_next_dir)*np.linalg.norm(vec_to_next)
                # 还原回去
                joint_positions[prev_joint_id]=joint_positions[path_joint_id]+calculated_vec_to_next
            # path1上的
            for i in range(len(path1)-1,-1,-1):
                # path_joint_id=path1[i]
                # 节点id
                prev_joint_id=path1[i]
                # 指向上一个节点的向量
                vec_to_next=joint_positions[prev_joint_id]-joint_positions[path_joint_id]
                # 左乘，改变向量
                calculated_vec_to_next_dir=rot_matrix.dot(vec_to_next)
                # 防止长度不对
                calculated_vec_to_next=calculated_vec_to_next_dir/np.linalg.norm(calculated_vec_to_next_dir)*np.linalg.norm(vec_to_next)
                # 还原回去
                joint_positions[prev_joint_id]=calculated_vec_to_next+joint_positions[path_joint_id]

        # debug
        # rot_matrix=rotation_matrix(np.array([1,0,0]),np.array([1,0,1]))
        # joint_orientations[0]=R.from_matrix(rot_matrix).as_quat()
        # joint_orientations[1]=R.from_matrix(rot_matrix).as_quat()
        joint_orientations[path_end_id]=joint_orientations[path1[1]]
        cur_dis=np.linalg.norm(joint_positions[path_end_id]-target_pose)
        if cur_dis<0.01:
            break
    print("距离",cur_dis,"迭代了",k,"次")
    # 更新不在链上的节点
    for k in range(len(joint_orientations)):
        if k in path:
            pass
        elif k==0:
            # 要单独处理，不然跟节点的-1就会变成从最后一个节点开始算
            pass
        else:
            # 先获取局部旋转
            # 这里如果直接存的就是矩阵就会有问题？
            local_rot_matrix=R.from_quat(local_rotation[k]).as_matrix()
            # 再获取我们已经计算了的父节点的旋转
            parent_rot_matrix=from_quat_safe(joint_orientations[parent_idx[k]])
            # 乘起来
            # re=local_rot_matrix.dot(parent_rot_matrix)
            re=parent_rot_matrix.dot(local_rot_matrix)
            joint_orientations[k]=R.from_matrix(re).as_quat()

            # 父节点没旋转的时候是：
            initial_o=from_quat_safe(no_caled_orientation[parent_idx[k]])
            # 父节点的旋转*delta_orientation=子节点旋转
            # 反求delta_orientation
            delta_orientation = np.dot(re, np.linalg.inv(initial_o))
            # 父节点的位置加原本基础上的旋转
            joint_positions[k]=joint_positions[parent_idx[k]]+delta_orientation.dot(local_position[k])

    return joint_positions, joint_orientations

def part2_inverse_kinematics(meta_data, joint_positions, joint_orientations, relative_x, relative_z, target_height):
    """
    输入lWrist相对于RootJoint前进方向的xz偏移，以及目标高度，IK以外的部分与bvh一致
    """
    # 根节点position好改，先把根节点position改了
    path, path_name, path1, path2=meta_data.get_path_from_root_to_end()
    target_pose=joint_positions[0]+np.array([relative_x,target_height-joint_positions[0][1],relative_z])
    IK_joint_positions, IK_joint_orientations=part1_inverse_kinematics(meta_data, joint_positions, joint_orientations, target_pose)
    for i in path:
        joint_positions[i]=IK_joint_positions[i]
        joint_orientations[i]=IK_joint_orientations[i]
    return joint_positions, joint_orientations

def bonus_inverse_kinematics(meta_data, joint_positions, joint_orientations, left_target_pose, right_target_pose):
    """
    输入左手和右手的目标位置，固定左脚，完成函数，计算逆运动学
    """
    
    return joint_positions, joint_orientations
    