import laspy
import open3d as o3d
import numpy as np
from osgeo import gdal
import matplotlib.pyplot as plt
from PIL import Image
import os, shutil

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

def postProcessDem(path, newFile):
    if (os.path.exists(newFile)):
        os.remove(newFile)

    dataset = gdal.Open(path)
    band = dataset.GetRasterBand(1)
    arr = band.ReadAsArray()
    [rows, cols] = arr.shape
    arr_min = arr.min()
    arr_max = arr.max()
    for i in range(0, rows):
        for j in range(0, cols):
            if arr[i][j] == arr_min or arr[i][j] < 3:
                arr[i][j] = 0

    driver = gdal.GetDriverByName("GTiff")
    outdata = driver.Create(newFile, cols, rows, 1, gdal.GDT_Float32)
    outdata.SetGeoTransform(dataset.GetGeoTransform())##sets same geotransform as input
    outdata.SetProjection(dataset.GetProjection())##sets same projection as input
    outdata.GetRasterBand(1).WriteArray(arr)
    outdata.FlushCache() 
    
def _getValOrZero(arr, xInd, yInd):
    [rows, cols] = arr.shape
    if xInd < 0 or yInd < 0 or xInd > rows -1 or yInd > cols -1:
        return 0
    
    try:
        return arr[xInd][yInd]
    except:
        return 0
    
def isItCollinear(window):
    if window[0] == [0, 0, 0] or window[2] == [0, 0, 0]:
        return False
    else:
        ab = vectorSubtraction(window[0], window[1])
        ac = vectorSubtraction(window[0], window[2])
        abac = vectorCrossProduct(ab, ac)
        if abac == [0, 0, 0]:
            return True
        else: 
            return False

def vectorSubtraction(v1, v2):
    result = np.subtract(v1, v2)
    return result

def vectorCrossProduct(v1, v2):
    result = [v1[1]*v2[2] - v1[2]*v2[1],
            v1[2]*v2[0] - v1[0]*v2[2],
            v1[0]*v2[1] - v1[1]*v2[0]]
    return result

def removeNonPlanarPoints(path, newFile):
    if (not os.path.exists(path)):
        return

    if (os.path.exists(newFile)):
        os.remove(newFile)

    dataset = gdal.Open(path)
    band = dataset.GetRasterBand(1)
    arr = band.ReadAsArray()
    [rows, cols] = arr.shape
    arrMin = arr.min()
    arrMax = arr.max()
    collinearPoints = []

    outputArr = arr.copy()
    for i in range(0, rows):
        for j in range(0, cols):
            currPoint = arr[i][j]
            # create a 3*3 window for each point
            # if the point hasn't got 8 neighbor point, use 0 as padding
            window = [[_getValOrZero(arr, i-1, j-1), _getValOrZero(arr, i-1, j), _getValOrZero(arr, i-1, j+1)], 
                      [_getValOrZero(arr, i, j-1), currPoint, _getValOrZero(arr, i, j+1)], 
                      [_getValOrZero(arr, i+1, j-1), _getValOrZero(arr, i+1, j), _getValOrZero(arr, i+1, j+1)], (i, j)]

            # calculate normal vectors for detecting planar surfaces
            isThisWindowCollinear = isItCollinear(window)
            if isThisWindowCollinear == True:
                collinearPoints.append(window[3])
            # if the point fits to a plane then add its original value to 

if __name__ == "__main__":
    #load_lidar_data('implementation/lidar_data/20011104_959.laz', 1)
    #load_lidar_data('implementation/lidar_data/20011104_950.laz', 2)
    #create_dem('implementation/lidar_data/raster_1_with_trees.tif', 1)
    #create_dem('implementation/lidar_data/raster_2_with_trees.tif', 2)

    #postProcessDem('implementation/lidar_data/raster_1_with_trees.tif', 'implementation/lidar_data/modified_dem.tif')
    #postProcessDem('implementation/lidar_data/raster_2_with_trees.tif', 'implementation/lidar_data/modified_dem_2.tif')

    removeNonPlanarPoints('implementation/lidar_data/modified_dem.tif', 'implementation/lidar_data/noise_removed.tif')