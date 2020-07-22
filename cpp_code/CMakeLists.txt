cmake_minimum_required(VERSION 2.8)

project(p3dv)

SET(CMAKE_CXX_FLAGS "-O3 -std=c++11 -w -fopenmp -pthread ")
SET(CMAKE_C_FLAGS "-O3 -std=c++11 -fopenmp -pthread ")

#SET(CMAKE_BUILD_TYPE "Debug") 
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Boost
#find_package(Boost 1.55.0 REQUIRED)
#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIRS})

#GLOG 
#find_package(glog REQUIRED)

#GFLAGS
#Require the FindGflags.cmake (reference: ceres_solver) in the /cmake folder
#find_package(Gflags)

#PCL
FIND_PACKAGE(PCL 1.7 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
#LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
#ADD_DEFINITIONS(${PCL_DEFINITIONS})
LIST(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
message("DEP: PCL [OK] ")

#OpenCV
#set(OpenCV_DIR /usr/local/opencv3.4.0/share/OpenCV)
FIND_PACKAGE(OpenCV 3.0 REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

message("DEP: OPENCV [OK] ")

# ceres
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
message("DEP: CERES [OK] ")

# head
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src SRC_LIST)
message("SRC_LIST is " ${SRC_LIST})

#SET(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBS}  /usr/local/lib/libproj.so glog::glog ${GFLAGS_LIBRARIES})
SET(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# SFM test
add_executable(sfm ${PROJECT_SOURCE_DIR}/test/sfm.cpp ${SRC_LIST})
target_link_libraries(sfm ${DEP_LIBS})

