import numpy as np
from math import sin, cos

def homogeneous_transform(matrix, array):
  # 提取旋转矩阵和平移向量
  rotation_matrix = matrix[:3, :3]
  translation_vector = matrix[:3, 3]

  # 提取欧拉角
  euler_angles = array[3:]

  # 计算旋转矩阵
  rotation_x = np.array([[1, 0, 0],
               [0, cos(euler_angles[0]), -sin(euler_angles[0])],
               [0, sin(euler_angles[0]), cos(euler_angles[0])]])

  rotation_y = np.array([[cos(euler_angles[1]), 0, sin(euler_angles[1])],
               [0, 1, 0],
               [-sin(euler_angles[1]), 0, cos(euler_angles[1])]])

  rotation_z = np.array([[cos(euler_angles[2]), -sin(euler_angles[2]), 0],
               [sin(euler_angles[2]), cos(euler_angles[2]), 0],
               [0, 0, 1]])

  # 计算新的齐次变换矩阵
  new_matrix = np.eye(4)
  new_matrix[:3, :3] = rotation_x @ rotation_y @ rotation_z @ rotation_matrix
  new_matrix[:3, 3] = translation_vector + array[:3]

  # 计算新的六元数组
  rotation_matrix = new_matrix[:3, :3]
  euler_angles = np.array([np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2]),
                          np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2)),
                          np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])])
  translation_vector = new_matrix[:3, 3]
  new_array = np.concatenate((translation_vector, euler_angles))

  # 返回新的齐次变换矩阵和数组
  return new_matrix, new_array

# 示例用法
matrix = np.array([[0.11883352408431004  , -0.03399021025184509 , 0.9923298702499637   , 1.1625483840101642],
                [0.9887742685727328   , -0.08709800376758459 , -0.12139156734620705 , 0.03854127610434157],
                [0.0905564504668786   , 0.9956202031422988   , 0.023258606875406075 , 2.06323641538619],
                [0                    , 0                    , 0                    , 1]])

array = [0.05, 0, 0, np.pi/2, -5.3229, 0.1216]

new_matrix, new_array = homogeneous_transform(matrix, array)

print("New Homogeneous Transform Matrix:")
print(new_matrix)

print("New Array:")
print(new_array)