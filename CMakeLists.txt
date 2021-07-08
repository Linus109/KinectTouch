cmake_minimum_required(VERSION 3.5)

project(KinectTouch)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB OSC
  "src/oscpack/osc/*.h"
  "src/oscpack/osc/*.cpp"
  "src/oscpack/ip/*.h"
  "src/oscpack/ip/*.cpp"
  "src/oscpack/ip/posix/*.h"
  "src/oscpack/ip/posix/*.cpp"
)

file(GLOB TUIO
  "src/TUIO/*.h"
  "src/TUIO/*.cpp"
)

file(GLOB freenectToOpencv
    "src/Libfreenect2OpenCV.cpp"
    "src/Libfreenect2OpenCV.h"
)

file(GLOB shapefilters
    "src/ShapeFilterStrategy.h"
    "src/ClusterFilter.cpp"
    "src/ClusterFilter.h"
    "src/DensityFilter.cpp"
    "src/DensityFilter.h"
)

find_package(OpenCV REQUIRED)
find_package(freenect2 REQUIRED)
find_package(Threads REQUIRED)

add_executable( KinectTouch src/KinectTouch.cpp ${OSC} ${TUIO} ${freenectToOpencv} ${shapefilters} src/Utils.h src/Utils.cpp)
# add_executable(KinectTouch src/KinectTouch.cpp)

include_directories(
  "/usr/include/libfreenect2"
  "src/TUIO"
  "src/oscpack"
  "src/oscpack/osc"
  "src/oscpack/ip"
  "src/oscpack/ip/posix"
)

target_link_libraries( KinectTouch ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} freenect2 )
