# def calculate_angle_betweent_end_and_target(index, chain_list, in_end_chain, in_target):
#     ic = index + 1
#     icc = index + 2
#     cur_link = chain_list[index]

#     # draw_line(in_end_chain.global_location, cur_link.global_location, color='blue')
#     # draw_line(in_target, cur_link.global_location, color='blue')

#     # 到末端关节的向量
#     to_end = in_end_chain.global_location - cur_link.global_location
#     # 到目标点的向量
#     to_target = in_target - cur_link.global_location
#     to_end = get_nor(to_end)
#     to_target = get_nor(to_target)

#     # 两向量之间的夹角
#     rotation_radius = np.arccos(np.dot(to_end, to_target))

#     print('rotation_angle', rotation_radius, np.rad2deg(rotation_radius))

#     # 旋转轴
#     rotation_axis = np.cross(to_end, to_target)
#     rotation_axis = get_nor(rotation_axis)

#     print('rotation_axis', rotation_axis)

#     draw_line(cur_link.global_location, cur_link.global_location + (20 * rotation_axis), color='grey')

#     # 计算出轴和角度后，cur_link做旋转
#     # cur_link旋转后，更新子链的变换信息

#     chain_list[ic].local_location = rotate_with_axis_angle(chain_list[ic].local_location, rotation_axis, rotation_radius)
#     chain_list[ic].global_location = chain_list[i].global_location + chain_list[ic].local_location

#     for j in range(icc, tipbone_index + 1):
#         print('index j', j)
#         chain_list[j].global_location = chain_list[j - 1].global_location + chain_list[j].local_location

#     # draw_point(chain_list[i + 1].global_location, color='blue')


# # //bones[i].a表示骨骼起点，bones[i].b表示骨骼终点，bones[i].len表示骨骼长度，bones.Length表示骨骼数目，target表示目标点
# #     for (int n = 0; n<iterations; n++)//迭代次数
# # 		{
# # 			for (int i = arrows.Length - 1; i >= 0; i--)//遍历每根骨骼
# # 			{
# #                 bones[i].angle = (angle + Vector2.SignedAngle(bones[bones.Length - 1].b - a, target - a)) % 360f;//angle表示相对于父骨骼的角度，获取目标点到当前骨骼起点与末端位置到当前骨骼起点两条线间的夹角
 
# #                 int j = i;
 
# #                 Vector2 origin = bones[j].a;
# #                 if(j!=0)
# #                         Vector2 right = bones[i - 1].forward;
# #                 else
# #                         Vector2 right = Vector2.right; //（1,0）
# #                 for(; j<bones.Length ; j++)//当前骨骼旋转后，变更子骨骼的位置
# #                 {
# #                     bones[j].a = origin;
        
# #                     //变更骨骼位置
# #                     bones[j].b = bones[j].a + 
# # Vector2(Mathf.Cos(bones[j].angle* Math.Deg2Rad + Mathf.Atan2(right.y, right.x)),Mathf.Sin(bones[j].angle* Math.Deg2Rad + Mathf.Atan2(right.y, right.x)));
                    
# #                     //修改以下信息以便更改后续子骨骼的位置
# #                     origin = bones[j].b;  
# # 			        right = bones[j].forward;
# #                 }
# # 			}
# # 		}