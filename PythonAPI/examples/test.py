import numpy as np

# # 旋转矩阵计算函数
# def rotation_matrix(roll, pitch, yaw):
#     # 转换为弧度
#     r = np.radians(roll)
#     p = np.radians(pitch)
#     y = np.radians(yaw)
#     # 计算旋转矩阵
#     Rz = np.array([[np.cos(y), -np.sin(y), 0],
#                    [np.sin(y), np.cos(y), 0],
#                    [0, 0, 1]])
#     Ry = np.array([[np.cos(p), 0, np.sin(p)],
#                    [0, 1, 0],
#                    [-np.sin(p), 0, np.cos(p)]])
#     Rx = np.array([[1, 0, 0],
#                    [0, np.cos(r), -np.sin(r)],
#                    [0, np.sin(r), np.cos(r)]])
#     R = np.dot(np.dot(Rz, Ry), Rx)
#     return R

# # 将世界坐标系下的位置和朝向向量转换为机体坐标系下的位置和朝向向量
# def world_to_body(world_pos, world_ori, body_pos, body_ori):
#     # 计算旋转矩阵
#     R = rotation_matrix(*world_ori)
#     # 计算机体坐标系下的位置和朝向向量
#     body_pos = np.dot(R, body_pos - world_pos)
#     body_ori = np.dot(R, body_ori)
#     # 对机体坐标系下的位置向量进行处理
#     # 在x轴方向上加上一个偏差
#     body_pos[0] += 1.0
#     # 在y轴方向上乘以一个缩放因子
#     # body_pos[1] *= 0.8
#     # 将机体坐标系下的位置和朝向向量转换为世界坐标系下的向量
#     world_pos = np.dot(R.T, body_pos) + world_pos
#     world_ori = np.array([np.arctan2(R[2, 1], R[2, 2]), np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2)), np.arctan2(R[1, 0], R[0, 0])])
#     return world_pos, world_ori


def get_world_transform(world):
    # 获得个体在世界坐标系中的坐标和位姿
    world_transform = world.player.get_transform() 
    return world_transform

def update_world_transform(world_pos, world_ori, update_pos):
    # 输入更新的位置增量和角度, 输出
    body_pos = [0, 0, 0]
    new_world_pos, new_world_ori = world_to_body( np.array(world_pos),  np.array(world_ori), body_pos, np.array(update_pos))
    return new_world_pos, new_world_ori


# 旋转矩阵计算函数
def rotation_matrix(roll, pitch, yaw):
    # 转换为弧度
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)
    # 计算旋转矩阵
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0],
                   [0, 0, 1]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    R = np.dot(np.dot(Rz, Ry), Rx)
    return R

# 将世界坐标系下的位置和朝向向量转换为机体坐标系下的位置和朝向向量
def world_to_body(world_pos, world_ori, body_pos, update_pos):
    # 计算旋转矩阵
    R = rotation_matrix(*world_ori)
    # 计算机体坐标系下的位置和朝向向量
    body_pos = np.dot(R, body_pos - world_pos)
    # body_ori = np.dot(R, body_ori)

    # 更新机体坐标系下的坐标和位姿
    body_pos[0] += update_pos[0]
    body_pos[1] += update_pos[1]
    body_pos[2] += update_pos[2]

    # 将机体坐标系下的位置和朝向向量转换为世界坐标系下的向量
    world_pos = np.dot(R.T, body_pos) + world_pos
    world_ori = np.array([np.arctan2(R[2, 1], R[2, 2]), np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2)), np.arctan2(R[1, 0], R[0, 0])])
    return world_pos, world_ori



# 设置世界坐标系下的无人机位置和朝向向量
world_pos =[0, 0, 0]
world_ori = [0, 0, 0]

update_pos = [2, 0, 3]
update_ori = [0, 0, 0]

new_world_ori = [i + j for i, j in zip(world_ori, update_ori)]

# 设置机体坐标系下的位置和朝向向量
body_pos = np.array([0, 0, 0])
body_ori = np.array([0, 0, 0])



new_world_pos, new_world_ori = update_world_transform(world_pos, new_world_ori, update_pos)
print("处理后世界坐标系下的位置和朝向向量：", new_world_pos, new_world_ori*180/np.pi)





# # 将世界坐标系下的位置和朝向向量转换为机体坐标系下的位置和朝向向量，然后进行处理，
# # 最后将处理后的位置向量和朝向向量转换回世界坐标系下的向量
# new_world_pos, new_world_ori = world_to_body(world_pos, world_ori, body_pos, body_ori)

# print("原始世界坐标系下的位置和朝向向量：", world_pos, world_ori)
# print("机体坐标系下的位置和朝向向量：", body_pos, body_ori)
# print("处理后世界坐标系下的位置和朝向向量：", new_world_pos, new_world_ori)
# print(new_world_ori*180/np.pi)
