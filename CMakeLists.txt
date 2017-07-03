cmake_minimum_required(VERSION 2.8.3)

project(light_mapping)
find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    dynamic_reconfigure
    geometry_msgs
    rosbag
    )

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -DNDEBUG -fopenmp -g -Wall -march=native -ffast-math -mfpmath=sse")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -DNDEBUG -fopenmp -g -Wall -march=native -ffast-math -mfpmath=sse")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(X11 REQUIRED)
find_package(CGAL REQUIRED)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS
)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${Pangolin_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    X11
    CGAL
    CGAL_Core
    gmp
    gmpxx
    mpfr
    lapack
)

FILE(GLOB SRC_FILES src/*.h* src/*.c*)
FILE(GLOB DELAUNAY_FILES src/delaunay/*.h* src/delaunay/*.c*)

add_library(3d_map SHARED ${SRC_FILES} ${DELAUNAY_FILES})
target_link_libraries(3d_map ${LIBS})

add_executable(ros_stereo_offline ros_stereo_offline.cc)
target_link_libraries(ros_stereo_offline 3d_map)

add_executable(ros_stereo_kitti ros_stereo_kitti.cc)
target_link_libraries(ros_stereo_kitti 3d_map)

