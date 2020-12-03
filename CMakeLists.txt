CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

#PROJECT(CFS_Knitro)
project(QuadProg++ CXX)

INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(src/lib)
INCLUDE_DIRECTORIES(src/lib/QP_lib)
#INCLUDE_DIRECTORIES(src/lib/gnuplot-iostream)

include_directories(/usr/include/python3.8)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(src)


add_executable(cfs_2d src/cfs_2d.cpp ${headers})
target_link_libraries(cfs_2d quadprog Eigen3::Eigen ${CMAKE_DL_LIBS})


target_link_libraries(cfs_2d python3.8)



