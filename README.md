# CFSMotionPlanning_cpp

This package contains an example of c++ implementation with the Convex Feasible Set algorithm (CFS) in 2D planning. 
To run this package, the following steps are required:
1. Install cmake. (https://cmake.org/install/)
2. Download this repo.
3. Go to `CFSMotionPlanning_cpp/src/lib`, and `$ git clone https://github.com/lava/matplotlib-cpp.git`.
4. Go to `CFSMotionPlanning_cpp/build`, delete `CMakeCache.txt`.
5. Go to `CFSMotionPlanning_cpp`, in `CMakeList.txt` check and change the python version to the user's version.
6. Go to `CFSMotionPlanning_cpp/build`, then
```
$ cmake ..
$ make
$ ./cfs_2d
```

![GitHub Logo](/build/path.png)
