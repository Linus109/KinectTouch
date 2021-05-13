cmake_minimum_required(VERSION 2.8)

project(KinectTouch)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# file(GLOB OSC
#   "src/oscpack/osc/*.h"
#   "src/oscpack/osc/*.cpp"
# )

# file(GLOB TUIO
#   "src/TUIO/*.h"
#   "src/TUIO/*.cpp"
# )

find_package(OpenCV REQUIRED)
find_package(freenect2 REQUIRED)
find_package(Threads REQUIRED)

#add_executable( KinectTouch src/KinectTouch.cpp ${OSC} ${TUIO})
add_executable(KinectTouch src/KinectTouch.cpp)

include_directories(
  "/usr/include/libfreenect2"
  # "src/TUIO"
  # "src/oscpack"
)

target_link_libraries( KinectTouch ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} freenect2 )
