import laspy
import open3d as o3d
import numpy as np
from osgeo import gdal
import matplotlib.pyplot as plt
from PIL import Image
import os, shutil, sys

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

def getValOrZero(arr, xVal, yVal):
    global rows, cols
    zVal = 0.0
    if not (xVal < 0 or yVal < 0 or xVal > rows -1 or yVal > cols -1):
        try:
            zVal = arr[xVal][yVal]
        except:
            pass

    return (xVal, yVal, zVal)
    
def isItCollinear(line):
    if line[0][2] == 0 or line[2][2] == 0: # when the neighbours are 0 then it doesn't need a check
        return False

    if line[0][2] == line[1][2] and line[1][2] == line[2][2]: # when the values are equal, then it is collinear
         return True
    else:
        ab = vectorSubtraction(line[0], line[1])
        ac = vectorSubtraction(line[0], line[2])
        abac = vectorCrossProduct(ab, ac)
        if abac == (0, 0, 0):
            return True
        else: 
            return False

def vectorSubtraction(v1, v2):
    result = np.subtract(v1, v2)
    return result

def vectorCrossProduct(v1, v2):
    result = np.cross(v1, v2)
    return (result[0], result[1], result[2])

rows, cols = 0, 0

def saveWindow(output, window):
    for i in range(len(window)):
        for j in range(len(window[i])):
            x, y, z = window[i][j]
            if not (x < 0 or y < 0 or x > rows -1 or y > cols -1):
                output[x, y] = z

def removeNonPlanarPoints(path, newFile):
    if (not os.path.exists(path)):
        print("input path not exists")
        return

    if (os.path.exists(newFile)):
        os.remove(newFile)

    dataset = gdal.Open(path)
    band = dataset.GetRasterBand(1)
    arr = np.array(band.ReadAsArray())

    global rows, cols
    [rows, cols] = arr.shape

    validPoints = {}
    for (i,j), value in np.ndenumerate(arr):
            currPoint = (i, j, value)

            if value == 0 or value < 16: # if the z value is 0, then we can go the next iteration
                continue
            else:
                if i in validPoints:
                    if j in validPoints[i]:
                        validPoints[i][j].append(value)
                    else:
                        validPoints[i][j] = value
                else:
                    validPoints[i] = {j : value}

    for x in validPoints:
        for y in validPoints[x]:
            point = (x, y, validPoints[x][y])
            # create a 3*3 window for each point
            # if the point hasn't got 8 neighbor point, use 0 as padding

            window = [[getValOrZero(validPoints, x-1, y-1), getValOrZero(validPoints, x-1, y), getValOrZero(validPoints, x-1, y+1)], 
                      [getValOrZero(validPoints, x, y-1), point, getValOrZero(validPoints, x, y+1)], 
                      [getValOrZero(validPoints, x+1, y-1), getValOrZero(validPoints, x+1, y), getValOrZero(validPoints, x+1, y+1)]]

            # We have to check these lines from the window:
            # | | | |    | |*| |   |*| | |   | | |*|
            # |*|*|*|    | |*| |   | |*| |   | |*| |
            # | | | |    | |*| |   | | |*|   |*| | |
            # calculate normal vectors for detecting planar surfaces
            horizontalLine = (window[1][0], window[1][1], window[1][2])
            verticalLine = (window[0][1], window[1][1], window[2][1])
            diagonalLineLeft = (window[0][0], window[1][1], window[2][2])
            diagonalLineRight = (window[0][2], window[1][1], window[2][0])

            hasCollinearLine = isItCollinear(horizontalLine) or isItCollinear(verticalLine) or isItCollinear(diagonalLineLeft) or isItCollinear(diagonalLineRight)
            # # if the point fits to a plane then add its original value to
            arr[x, y] = point[2] if hasCollinearLine else 0
            #saveWindow(arr, window)

    driver = gdal.GetDriverByName("GTiff")
    outdata = driver.Create(newFile, cols, rows, 1, gdal.GDT_Float32)
    outdata.SetGeoTransform(dataset.GetGeoTransform())##sets same geotransform as input
    outdata.SetProjection(dataset.GetProjection())##sets same projection as input
    outdata.GetRasterBand(1).WriteArray(arr)
    outdata.FlushCache()

if __name__ == "__main__":
    # load_lidar_data('lidar_data/20011104_950.laz', 1)
    # load_lidar_data('lidar_data/20011104_959.laz', 2)
    #create_dem('implementation/lidar_data/raster_1_with_trees.tif', 1)
    #create_dem('implementation/lidar_data/raster_2_with_trees.tif', 2)

    #postProcessDem('implementation/lidar_data/raster_1_with_trees.tif', 'implementation/lidar_data/modified_dem.tif')
    #postProcessDem('implementation/lidar_data/raster_2_with_trees.tif', 'implementation/lidar_data/modified_dem_2.tif')

    removeNonPlanarPoints('lidar_data/modified_dem.tif', 'lidar_data/noise_removed.tif')
    removeNonPlanarPoints('lidar_data/modified_dem_2.tif', 'lidar_data/noise_removed_2.tif')