KinectTouch
==
Turns any surface into a giant touchpad using kinect

[![kinect - build a TUIO multitouch pad on any surface](http://img.youtube.com/vi/4zXtV66cFDY/0.jpg)](http://www.youtube.com/watch?v=4zXtV66cFDY)

Prerequisites
==
  - [Libfreenect2](https://github.com/OpenKinect/libfreenect2)
    - Arch installation
```bash
yay -Syu libfreenect2
```

  - [OpenCV](http://opencv.org/)
    - Ubuntu installation 
```bash 
sudo apt-get install libopencv-dev
```
  - [CMake](https://cmake.org/)
    - Ubuntu installation
```bash
sudo apt-get install cmake
```

Building and Running
==
```bash
mkdir build
cd build
cmake ..
make
```

```bash
./KinectTouch
```

TODOs
==
 - Integrate [TUIO](https://github.com/mkalten/TUIO11_CPP) as a submodule
 - Tracking / filtering of touch points 


