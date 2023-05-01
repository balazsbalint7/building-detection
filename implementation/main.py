import laspy
import open3d as o3d
import numpy as np
from osgeo import gdal
import matplotlib.pyplot as plt
from PIL import Image

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

def create_dem(path, num):
    raster_dataset = gdal.Open(path)

    channel1 = raster_dataset.GetRasterBand(1)
    channel2 = raster_dataset.GetRasterBand(2)
    channel3 = raster_dataset.GetRasterBand(3)

    c1 = channel1.ReadAsArray()
    c2 = channel2.ReadAsArray()
    c3 = channel3.ReadAsArray()

    dem_img = np.dstack((c1, c2, c3))
    plt.imshow(dem_img)
    plt.savefig("dem_" + str(num) + ".png")
    plt.show()

def postProcessDem(path):
    pass

if __name__ == "__main__":
    #load_lidar_data('implementation/lidar_data/20011104_959.laz', 1)
    #load_lidar_data('implementation/lidar_data/20011104_950.laz', 2)
    create_dem('implementation/lidar_data/raster_1_with_trees.tif', 1)
    create_dem('implementation/lidar_data/raster_2_with_trees.tif', 2)
    # dataset = gdal.Open('implementation/lidar_data/raster_1.tif', gdal.GA_ReadOnly)
    # for x in range(1, dataset.RasterCount + 1):
    #     band = dataset.GetRasterBand(x).ReadAsArray()
    #     array = band.ReadAsArray()
