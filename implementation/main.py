import laspy
import open3d as o3d
import numpy as np

def load_lidar_data(path, num):
    print("Dataset " + str(num))
    print("---------")
    las = laspy.read(path)
    print(f"Point format:        {las.header.point_format}")
    print(f"Number of points:    {las.header.point_count}")
    print(f"Number of vlrs:      {las.vlrs}")
    print(list(las.point_format.dimension_names))
    print(f"Array X:      {las.X}")
    print(f"Array Y:      {las.Y}")
    print(f"Array Z:      {las.Z}")
    print(f"Intensity:    {las.intensity}")
    print(f"GPS time:     {las.gps_time}")
    print(f"Classes:      {set(list(las.classification))}")
    point_data = np.stack([las.X, las.Y, las.Z], axis=0).transpose((1, 0))
    print(f"3-dimensional array: {point_data}")

    geom = o3d.geometry.PointCloud()
    geom.points = o3d.utility.Vector3dVector(point_data)
    o3d.visualization.draw_geometries([geom])

if __name__ == "__main__":
    load_lidar_data('implementation/lidar_data/20011104_959.laz', 1)
    load_lidar_data('implementation/lidar_data/20011104_950.laz', 2)