cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include(micro.cmake)
#include(init_micro.cmake)
#rosbuild_init1()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3")
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


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)
#find_package(Cholmod REQUIRED)
find_package(OpenGL REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)
find_package(Pangolin REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}
${Boost_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/Imu/include
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES ("/usr/include/eigen3"
"/usr/include/suitesparse"
${OPENGL_INCLUDE_DIR}
)
#include_directories("/usr/include/suitesparse")
#include_directories(
#${EIGEN3_INCLUDE_DIR}
#${CHOLMOD_INCLUDE_DIR}
#)
#rosbuild_add_executable1(${PROJECT_NAME}
#Build IMU
add_subdirectory(Imu)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}
src/main.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Optimizer2.cc
src/FrameDrawer.cc
src/MapDrawer.cc
src/Viewer.cc
)
#rosbuild_add_boost_directories1()
#rosbuild_link_boost1(${PROJECT_NAME} thread)
#target_link_libraries(${target} -lboost_thread)
message("cmake find root path ${CMAKE_FIND_ROOT_PATH}")
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Boost_LIBRARIES}
${Pangolin_LIBRARIES}
${OPENGL_LIBS}
/usr/lib/x86_64-linux-gnu/libcholmod.so
/usr/lib/libtbb.so.2
/usr/lib/libtbbmalloc.so.2
${PROJECT_SOURCE_DIR}/libMVSDK.so
#${PROJECT_SOURCE_DIR}
/usr/local/lib/libgtsam.so
${PROJECT_SOURCE_DIR}/build/Imu/libImu.so
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_cholmod.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_dense.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sba.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_sim3.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_slam3d.so
)

add_executable (pclViewer
src/pclViewer.cc
)

target_link_libraries (pclViewer ${PCL_LIBRARIES})

add_executable( viz
src/viz.cc
)

target_link_libraries( viz
${OpenCV_LIBS}
${PCL_LIBRARIES}
)

add_executable( stereo
src/stereoTest.cpp
)

target_link_libraries( stereo
${OpenCV_LIBS}
)

