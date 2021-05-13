//============================================================================
// Name        : KinectTouch.cpp
// Author      : github.com/robbeofficial
// Version     : 0.something
// Description : recognizes touch points on arbitrary surfaces using kinect
// 				 and maps them to TUIO cursors
// 				 (turns any surface into a touchpad)
//============================================================================

/*
 * 1. point your kinect from a higher place down to your table
 * 2. start the program (keep your hands off the table for the beginning)
 * 3. use your table as a giant touchpad
 */

#include <iostream>
#include <vector>
#include <map>
using namespace std;

// openCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

#define CV_RETR_LIST cv::RETR_LIST
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE 
#define CV_GRAY2BGR cv::COLOR_GRAY2BGR 
#define CV_FILLED cv::FILLED

// libfreenect2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <string>
#include <iostream>

// TUIO
/*
#include "TuioServer.h"
using namespace TUIO;
*/

// TODO smoothing using kalman filter

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------

// libfreenect2
std::string serial;
libfreenect2::Freenect2 freenect2;
libfreenect2::Freenect2Device *dev = 0;
libfreenect2::PacketPipeline *pipeline = 0;
libfreenect2::FrameMap frames;
libfreenect2::SyncMultiFrameListener *listener;

bool mousePressed = false;

//---------------------------------------------------------------------------
// Functions
//---------------------------------------------------------------------------

int initLibfreenect2() {
    if(freenect2.enumerateDevices() == 0) {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    if (serial == "") {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    pipeline = new libfreenect2::CpuPacketPipeline();
    dev = freenect2.openDevice(serial, pipeline);

    int types = 0;
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth | libfreenect2::Frame::Color;
    listener = new libfreenect2::SyncMultiFrameListener(types);
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);

    if (!dev->start()) {
        std::cout << "could not start device" << std::endl;
        return -1;
    }

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Registration* registration = new libfreenect2::Registration(
            dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

	return 0;
}

void average(vector<Mat1s>& frames, Mat1s& mean) {
	Mat1d acc(mean.size());
	Mat1d frame(mean.size());

	for (unsigned int i=0; i<frames.size(); i++) {
		frames[i].convertTo(frame, CV_64FC1);
		acc = acc + frame;
	}

	acc = acc / frames.size();

	acc.convertTo(mean, CV_16SC1);
}



int main() {

	const unsigned int nBackgroundTrain = 30;
	const unsigned short touchDepthMin = 10;
	const unsigned short touchDepthMax = 20;
	const unsigned int touchMinArea = 50;

	const bool localClientMode = true; 					// connect to a local client

	const double debugFrameMaxDepth = 4000; // maximal distance (in millimeters) for 8 bit debug depth frame quantization
	const char* windowName = "Debug";
	const Scalar debugColor0(0,0,128);
	const Scalar debugColor1(255,0,0);
	const Scalar debugColor2(255,255,255);

	int xMin = 110;
	int xMax = 560;
	int yMin = 120;
	int yMax = 320;

	Mat1s depth(1080, 1920); // 16 bit depth (in millimeters)
	Mat1b depth8(1080, 1920); // 8 bit depth
	Mat3b rgb(1080, 1920); // 8 bit depth

	Mat3b debug(1080, 1920); // debug visualization

	Mat1s foreground(1920, 1080);
	Mat1b foreground8(1920, 1080);

	Mat1b touch(1920, 1080); // touch mask

	Mat1s background(1080, 1920);
	vector<Mat1s> buffer(nBackgroundTrain);

	if (initLibfreenect2() == -1) {
        std::cout << "failed to initialize libfreenect2" << std::endl;
        exit(-1);
    }

	// TUIO server object
	/*
	TuioServer* tuio;
	if (localClientMode) {
		tuio = new TuioServer();
	} else {
		tuio = new TuioServer("192.168.0.2",3333,false);
	}
	TuioTime time;
	*/

	// create some sliders
	namedWindow(windowName);
	createTrackbar("xMin", windowName, &xMin, 1920);
	createTrackbar("xMax", windowName, &xMax, 1920);
	createTrackbar("yMin", windowName, &yMin, 1080);
	createTrackbar("yMax", windowName, &yMax, 1080);

	// create background model (average depth)
	for (unsigned int i=0; i<nBackgroundTrain; i++) {
		listener->waitForNewFrame(frames, 10*1000);
		depth.data = (uchar*) frames[libfreenect2::Frame::Depth];
		buffer[i] = depth;
        listener->release(frames);
	}
	average(buffer, background);

	while ( (char) waitKey(1) != (char) 27 ) {
		// read available data
		listener->waitForNewFrame(frames, 10*1000);

		// update 16 bit depth matrix

        libfreenect2::Frame *depthFrame = frames[libfreenect2::Frame::Depth];
		depth.data = (uchar*) depthFrame;

		// update rgb image
		/* rgb.data = (uchar*) frames[libfreenect2::Frame::Color]; // segmentation fault here
		cvtColor(rgb, rgb, COLOR_RGB2BGR); */

		// extract foreground by simple subtraction of very basic background model
		foreground = background - depth;

		// find touch mask by thresholding (points that are close to background = touch points)
		touch = (foreground > touchDepthMin) & (foreground < touchDepthMax);

		// extract ROI
		Rect roi(xMin, yMin, xMax - xMin, yMax - yMin);
		Mat touchRoi = touch(roi);

		// find touch points
		vector< vector<Point2i> > contours;
		vector<Point2f> touchPoints;
		findContours(touchRoi, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point2i(xMin, yMin));
		for (unsigned int i=0; i<contours.size(); i++) {
			Mat contourMat(contours[i]);
			// find touch points by area thresholding
			if ( contourArea(contourMat) > touchMinArea ) {
				Scalar center = mean(contourMat);
				Point2i touchPoint(center[0], center[1]);
				touchPoints.push_back(touchPoint);
			}
		}

		// send TUIO cursors
		
		/* time = TuioTime::getSessionTime();
		tuio->initFrame(time);

		for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			float cursorX = (touchPoints[i].x - xMin) / (xMax - xMin);
			float cursorY = 1 - (touchPoints[i].y - yMin)/(yMax - yMin);
			TuioCursor* cursor = tuio->getClosestTuioCursor(cursorX,cursorY);
			// TODO improve tracking (don't move cursors away, that might be closer to another touch point)
			if (cursor == NULL || cursor->getTuioTime() == time) {
				tuio->addTuioCursor(cursorX,cursorY);
			} else {
				tuio->updateTuioCursor(cursor, cursorX, cursorY);
			}
		}

		tuio->stopUntouchedMovingCursors();
		tuio->removeUntouchedStoppedCursors();
		tuio->commitFrame(); */
	

		// draw debug frame
		depth.convertTo(depth8, CV_8U, 255 / debugFrameMaxDepth); // render depth to debug frame
		cvtColor(depth8, debug, CV_GRAY2BGR);
		// debug.setTo(debugColor0, touch);  // touch mask
		rectangle(debug, roi, debugColor1, 2); // surface boundaries
		/* for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			circle(debug, touchPoints[i], 5, debugColor2, CV_FILLED);
		} */

		// render debug frame (with sliders)
		imshow(windowName, debug);
		// imshow(windowName, depthFrame->Float);
		// imshow("image", rgb);
        listener->release(frames);

	}
    dev->stop();
    dev->close();
    delete listener;
    delete dev;
    delete pipeline;

	return 0;
}
