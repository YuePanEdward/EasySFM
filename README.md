# EasySFM
A simple Structure-from-motion project for ETH P3DV ([Photogrammetry and 3D Vision](https://prs.igp.ethz.ch/)) course

![alt text](assets/easySFM_demo_1_fountain.gif)

![alt text](assets/easySFM_demo_2_personhall.gif)

### C++ version passed

Environment: Linux (Ubuntu 16.04)

## How to use

1. Install dependent 3rd libraries 

[OpenCV(>=3)](https://github.com/opencv/opencv) with [contrib](https://github.com/opencv/opencv_contrib/tree/3.4),[PCL(>=1.7)](https://github.com/PointCloudLibrary/pcl), [Eigen3](https://eigen.tuxfamily.org/dox/), [Ceres](https://github.com/ceres-solver/ceres-solver)


2. Compile
```
cd cpp_code
mkdir build
cd build
cmake ..
make 
```

3. Run
```
cd ..
# Fountain Demo
sh script/run_fountain_small.sh
# If you'd like to use your own data or change some vital parameters, configure the run_xxx.sh file
```

4. Dataset preparation

A toy dataset (fountain) can be found in test_data folder.

You can also download some datasets from [SFM Datasets (COLMAP)](https://onedrive.live.com/?authkey=%21AAQumsDDwZBIW3w&id=C58A258D760E1B58%2146879&cid=C58A258D760E1B58).

More datasets are availiable at [Photogrammetry datasets](https://github.com/natowi/photogrammetry_datasets).

You can also take some photo yourself and then calibrate them.

The calibration file should have the following format:

K.txt
```
f1 0 c1
0 f2 c2
0 0 1
```

distort.txt (not neccessary)
```
k1 k2 p1 p2
```

5. Configure shell file

The following contents is an example of the shell file in script folder. You can play with different parameter settings here.

```
#run_fountain_small.sh

#data path
image_folder=../test_data/images;
image_list=../test_data/image_list.txt;
calib_k_file=../test_data/k/K.txt;
calib_distort_file=none; # not neccessary, if not provided, just set it as none
output_file=../test_data/output/sfm_sparse_point_cloud_fountain.ply;

#processing parameters
feature_type=S;                 # S for SURF an O for ORB
feature_parameter=300;          # For SURF, this is the minHessian, for ORB, this is the max number of features [int]
repro_dis_ransac=1.0;           # The initial value of reprojection distance threshold for RANSAC (Initialization 5points and PnP)
find_init_frames=1;             # Find best frame pairs for initialization or not (1: find pair, 0: just use the first two frames for initialization)
ba_calib_change_tolerance=0;    # How much can the calib matrix change when doing BA
ba_frequency=4;                 # Do Bundle Adjustment for each X frame

#display parameters
launch_viewer=1;                # Open the real-time viewer (2: display all the processing details, 1: display neccessary details, 0: only dispaly the final result)
view_sphere=0;                  # render the point as sphere during visualization (sphere 1, point 0)

# Debug mode
#gdb --args \  
./bin/sfm ${image_folder} ${image_list} ${calib_k_file} ${calib_distort_file} ${output_file} \
${feature_type} ${feature_parameter} ${repro_dis_ransac} ${find_init_frames} ${ba_calib_change_tolerance} ${ba_frequency} \
${launch_viewer} ${view_sphere}  
```

### TO DO List
- [ ] add sequential mode (visual slam)
- [ ] add multiple-camera mode (different intrinsic elements)
- [ ] add semantic label using deep learning
- [ ] add multi-view stereo dense reconstruction
- [ ] try more robust outlier filter
- [ ] improve the efficieny of bundle adjustment
- [ ] multithreading computing using CUDA


### About
Author: Yue Pan @ ETH Zurich D-BAUG