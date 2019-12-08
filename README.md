# P3DV
Structure-from-motion project for ETH P3DV ([Photogrammetry and 3D Vision](https://prs.igp.ethz.ch/)) course

![alt text](assets/demo_expected.png)

## C++ version under development 

### How to use

1. Install dependent 3rd libraries: 

[OpenCV(>=3)](https://github.com/opencv/opencv),[PCL(>=1.7)](https://github.com/PointCloudLibrary/pcl), [Eigen3](https://eigen.tuxfamily.org/dox/), [Ceres](https://github.com/ceres-solver/ceres-solver)


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
# Demo
sh script/run.sh
# If you'd like to use your own data or change some vital parameters, configure the run.sh file
```