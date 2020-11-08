# PCA and Voxel Grid Filter #

There are 2 parts in this chapter:
- PCA (Principle Component Analysis)
- Voxel Grid Filter

![image](image/pca_voxel.png)

## PCA: Principle Component Analysis
- PCA is to find the dominant directions of the point cloud.
- Applications:
  - Dimensionality Reduction
  - Surface Normal Estimation
  - Canonical Orientation
  - Keypoint Detection
  - Feature Description
- Core Formula:

![image](image/formula1.png)

- Kernel PCA

## Voxel Grid Filter (Downsampling)
1. Build a voxel grid
2. Select one point in each grid cell (Centroid/Random)

## Run
- Prerequisites
  - [**Eigen**](http://eigen.tuxfamily.org/): Linear algebra computing
  - [**PCL**](https://pointclouds.org/): Visualization
- Build and run
  - Build
  ```bash
  git clone https://github.com/zha0ming1e/ 
  cd 3D_Point_Cloud_Processing/ch1/ 
  mkdir build 
  cd build/ 
  cmake .. 
  make -j4 
  cd ../bin/   
  ```
  Then, there are two executable file in **bin/** directory: **pca_pointcloud**, **vgf_pointcloud**
  - Run
  ```bash
  ./pca_pointcloud ../data/POINT_CLOUD_DATA_FILE_TXT 
  ./vgf_pointcloud ../data/POINT_CLOUD_DATA_FILE_TXT 
  ```
- Results
  - PCA

  ![image](image/pca1.png)
  ![image](image/pca2.png)
  ![image](image/pca3.png)

  - Voxel Grid Filter

  ![image](image/vgf1.png)
  ![image](image/vgf2.png)
  ![image](image/vgf3.png)

