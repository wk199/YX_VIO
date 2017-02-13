cmake_minimum_required(VERSION 2.4.6)
#IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
#ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
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

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package (Threads REQUIRED)
find_package (Boost REQUIRED COMPONENTS thread system)
find_package(OpenCV REQUIRED)
include_directories(
${PROJECT_SOURCE_DIR}/Imu/include
)

add_library(Imu SHARED
src/hidcomm.cpp
src/usb2io.cpp
src/MPU9250.cpp
src/IMUMeasurement.cpp
src/Sensor.cpp
src/SensorFusion.cpp
)

target_link_libraries(Imu
${PROJECT_SOURCE_DIR}/Imu/lib/libusb-1.0.so
#/home/haoliliang/Work/Project/GTSAM/gtsam/build/gtsam/libgtsam.so
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Boost_LIBRARIES}
${CMAKE_THREAD_LIBS_INIT}
)










