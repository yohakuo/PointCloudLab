import open3d as o3d
import numpy as np

path = r"data/raw/20260909_203617_pc.ply"  # 也可以是 .ply

pcd = o3d.io.read_point_cloud(path)

points = np.asarray(pcd.points)

min_xyz = points.min(axis=0)
max_xyz = points.max(axis=0)
size = max_xyz - min_xyz

print("最小坐标:", min_xyz)
print("最大坐标:", max_xyz)
print("点云尺寸:", size)