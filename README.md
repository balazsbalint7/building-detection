# building-detection

# 2001 HCFCD Lidar: Harris County Point Cloud files
https://noaa-nos-coastal-lidar-pds.s3.amazonaws.com/laz/geoid18/102/index.html

# Requirements
To run the python scripts or pdal with the filters, you have to install some python libraries and pdal.

## Python libs
```pip install laspy, open3d, matplotlib, numpy, gdal```  

In some cases installing gdal on Windows may fail. If this happens with you, you can install it manually:
1. Download the correct wheel file from here: https://www.lfd.uci.edu/~gohlke/pythonlibs/#gdal  
**Note**: If you have python 3.8 installed, you have to download the one named as *GDAL‑3.4.3‑cp310‑cp310‑win_amd64.whl* (or win32).
2. Install it with these code:  
```python -m pip install path-to-wheel-file.whl```  

### Usage
You can run the main.py script without any cmd line argument.  
In the main function you can run the generator and postprocess functions. You might need to corrigate the path of the input and output files, depending on your execution folder.

## Pdal
To create and filter dem, we have to use pdal  

With conda you can install it like this:  
`conda install -c conda-forge pdal`

### Usage
Navigate to the **implementation/lidar_data** folder.

You can run a filter with this command in conda:  
`pdal pipeline filter.json`
