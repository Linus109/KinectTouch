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
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <vector>
#include <map>
#include <fstream>
#include <math.h>
using namespace std;

// openCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
using namespace cv;

#define CV_RETR_LIST cv::RETR_LIST
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE 
#define CV_CHAIN_APPROX_NONE cv::CHAIN_APPROX_NONE 
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
#include "TuioServer.h"
#include "Libfreenect2OpenCV.h"
using namespace TUIO;

// TODO smoothing using kalman filter

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------

bool mousePressed = false;

//---------------------------------------------------------------------------
// Functions
//---------------------------------------------------------------------------

void average(vector<Mat>& frames, Mat& mean) {
    Mat acc(mean.size(), mean.type());
	Mat frame(mean.size(), mean.type());
    frames[0].copyTo(acc);

	for (unsigned int i=1; i<frames.size(); i++) {
		frames[i].copyTo(frame);
		acc = acc + frame;
	}
	acc = acc / frames.size();
    acc.copyTo(mean);
}

int main() {
    RNG rng(12345);
	const unsigned int nBackgroundTrain = 10;
	int touchDepthMin = 7;
	int touchDepthMax = 20;
	int touchMinArea = 5;
	int touchMaxArea = 10;

	const bool localClientMode = true; 					// connect to a local client

	const double debugFrameMaxDepth = 4000; // maximal distance (in millimeters) for 8 bit debug depth frame quantization
	const char* windowName = "Debug";
	const Scalar debugColor0(0,0,128);
	const Scalar debugColor1(255,0,0);
	const Scalar debugColor2(255,255,255);
	const Scalar debugColor3(0,255,0);

	int xMin = 64;
	int xMax = 428;
	int yMin = 6;
	int yMax = 285;
    
	Mat depth;
	Mat1b depth8(424, 512); // 8 bit depth
	Mat rgb; // 8 bit depth

	Mat3b debug(424, 512); // debug visualization

	Mat foreground;
	Mat1b foreground8(424, 512);

	Mat touch;

	Mat background;
	vector<Mat> buffer(nBackgroundTrain);

    int multi = 2;

    libfreenect2opencv::Libfreenect2OpenCV libfreenect2OpenCV;
    libfreenect2OpenCV.start();

	// {{{ TUIO server object
	TuioServer* tuio;
	if (localClientMode) {
		tuio = new TuioServer();
        // std::cout << "--- local ---" << std::endl;
	} else {
		tuio = new TuioServer("192.168.0.2",3333,false);
        // std::cout << "--- NOOOOOT local ---" << std::endl;
	}
	TuioTime time;
    // }}}

	// create some sliders {{{
	namedWindow(windowName);
	createTrackbar("xMin", windowName, &xMin, 512);
	createTrackbar("xMax", windowName, &xMax, 512);
	createTrackbar("yMin", windowName, &yMin, 424);
	createTrackbar("yMax", windowName, &yMax, 424);
	createTrackbar("touchDepthMin", windowName, &touchDepthMin, 100);
	createTrackbar("touchDepthMax", windowName, &touchDepthMax, 200);
	createTrackbar("touchMinArea", windowName, &touchMinArea, 100);
	createTrackbar("touchMaxArea", windowName, &touchMaxArea, 100);
    // }}}

	// create background model (average depth)
	for (unsigned int i=0; i<nBackgroundTrain; i++) {
        depth = libfreenect2OpenCV.getDepthMat();
		buffer[i] = depth;
	}
	average(buffer, background);
    
    int countup = 0;
	while ( (char) waitKey(1) != (char) 27 ) {
		// read available data
		// update 16 bit depth matrix
        depth = libfreenect2OpenCV.getDepthMat();
        
		// update rgb image
		/* rgb.data = (uchar*) frames[libfreenect2::Frame::Color]; // segmentation fault here
		cvtColor(rgb, rgb, COLOR_RGB2BGR); */
        // rgb = libfreenect2OpenCV.getRGBMat();
        // cvtColor(rgb, rgb, COLOR_BGR2GRAY);

		// extract foreground by simple subtraction of very basic background model
		foreground = background - depth;

		// find touch mask by thresholding (points that are close to background = touch points)
		touch = (foreground > touchDepthMin) & (foreground < touchDepthMax);

        // {{{ print mats
        // if (countup >= 30) {
        //     cout << "depth mat: " << endl;
        //     cout << depth << endl;
        //     cout << "***********" << endl;
        //     cout << "foreground mat: " << endl;
        //     cout << foreground << endl;
        //     cout << "***********" << endl;
        //     cout << "touch mat: " << endl;
        //     cout << touch << endl;
            
        //     cout << "is background == depth?" << endl;
        //     cv::Mat diff = background != depth;
        //     // Equal if no elements disagree
        //     bool eq = cv::countNonZero(diff) == 0;
        //     cout << eq << endl;
            
        //     countup = 0;
        // }
        // countup++;
        // }}}

		// extract ROI
		Rect roi(xMin, yMin, xMax - xMin, yMax - yMin);
		Mat touchRoi = touch(roi);

		// {{{ find touch points
		vector< vector<Point2i> > contours;
		// vector< vector<Point2i> > contoursRGB;
		vector<Point2f> touchPoints;
		findContours(touchRoi, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point2i(xMin, yMin));
		// findContours(rgb, contoursRGB, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        // vector< vector<Point2i> > hull(contoursRGB.size());
		for (unsigned int i=0; i<contours.size(); i++) {
			Mat contourMat(contours[i]);

            double cArea = contourArea(contourMat);
			if ( cArea > touchMinArea && cArea <= touchMaxArea ) {
				Scalar center = mean(contourMat);
                // cout << "found contours" << endl;
                // vector<Point2i> newHull(1);
                // convexHull(contours[i], newHull);
                // hull.insert(hull.end(), newHull);
				Point2i touchPoint(center[0], center[1]);
				touchPoints.push_back(touchPoint);
			}
		}
        // for (int i = 0; i < contoursRGB.size(); i++) {
        //     convexHull(contoursRGB[i], hull[i]);
        // }
        // }}}

		// {{{ send TUIO cursors
		time = TuioTime::getSessionTime();
		tuio->initFrame(time);

		for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			float cursorX = (touchPoints[i].x - xMin) / (xMax - xMin);
			float cursorY = 1 - (touchPoints[i].y - yMin)/(yMax - yMin);
            // std::cout << "touchpoint[" << i << "]: x=" << cursorX << ", y=" << cursorY << std::endl; 
			TuioCursor* cursor = tuio->getClosestTuioCursor(cursorX,cursorY);
			// TODO improve tracking (don't move cursors away, that might be closer to another touch point)
			if (cursor == NULL) {
				tuio->addTuioCursor(cursorX,cursorY);
                // std::cout << "addTuioCursor[" << i << "]: x=" << cursorX << ", y=" << cursorY << std::endl; 
			} else {
				tuio->updateTuioCursor(cursor, cursorX, cursorY);
                // std::cout << "updateTuioCursor[" << i << "]: x=" << cursorX << ", y=" << cursorY << std::endl; 
			}
		}

		tuio->stopUntouchedMovingCursors();
		tuio->removeUntouchedStoppedCursors();
		tuio->commitFrame();

        // }}}
	
		// draw debug frame {{{
		depth.convertTo(depth8, CV_8U, 255 / debugFrameMaxDepth); // render depth to debug frame
		// foreground.convertTo(foreground8, CV_8U, 255 / debugFrameMaxDepth); // render depth to debug frame
		cvtColor(depth8, debug, CV_GRAY2BGR);
		// cvtColor(foreground8, debug, CV_GRAY2BGR);
		debug.setTo(debugColor0, touch);  // touch mask
		rectangle(debug, roi, debugColor1, 2); // surface boundaries
		for (unsigned int i=0; i<touchPoints.size(); i++) { // touch points
			circle(debug, touchPoints[i], 5, debugColor1, CV_FILLED);
		}

		// render debug frame (with sliders)
		imshow(windowName, debug);
		// imshow(windowName, rawDepth);
		// imshow(windowName, depthFrame->Float);
		// imshow("image", drawing);

        // }}}

	}

    libfreenect2OpenCV.stop();
	return 0;
}
