# simulator-slam
This is slam code for BrambleBee

# Prerequisites
1. Ceres Solver

```
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev
# - However, if you want to build Ceres as a *shared* library, you must
#   add the following PPA:
# TODO: this appears to not be necessary on bionic. There is no "release" for this bugfix"
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev

git clone https://ceres-solver.googlesource.com/ceres-solver

cd ceres-solver
mkdir build
cd build
cmake ..
make -j8
sudo make install
```
# Launch SLAM code

```
roslaunch sensor_fusion sensor_fusion.launch
```

# Ports
Subscribers (topic name):
1. HDL-32: /velodyne_points
2. IMU: /imu/data
3. Wheel odometry: /husky_velocity_controller/odom

Publisher (topic name):
1. Pose estimates: /nav_filter/nav_filter/states
