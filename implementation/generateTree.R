install.packages("lidR")
library(lidR)
library(terra)

# set 
las <- readLAS("lidar_data/20011104_950.las", select = "xyzr", filter = "-drop_z_below 0")

# Point-to-raster 2 resolutions
# chm_p2r_05 <- rasterize_canopy(las, 0.5, p2r(subcircle = 0.2), pkg = "terra")

# Post-processing median filter (not neccessary)
# kernel <- matrix(1,3,3)
# chm_p2r_05_smoothed <- terra::focal(chm_p2r_05, w = kernel, fun = median, na.rm = TRUE)
# ttops_chm_p2r_05_smoothed <- locate_trees(chm_p2r_05_smoothed, lmf(5))

# algo <- dalponte2016(chm_p2r_05_smoothed, ttops_chm_p2r_05_smoothed)
# las <- segment_trees(las, algo) # segment point cloud

las <- segment_trees(las, li2012(R = 3, speed_up = 5))

writeLas(las, "/lidar_data/20011104_950_trees.las")