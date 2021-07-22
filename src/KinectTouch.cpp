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
#include <cmath>
#include <list>;

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
//#include "ShapeFilterStrategy.h"
#include "ClusterFilter.h"
#include "DensityFilter.h"

using namespace TUIO;

// TODO smoothing using kalman filter

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
const  int framesForAverage = 50000;
list<Mat> frames;
int oldestFrameIndex = 0;
bool mousePressed = false;
bool useFilter = false;
Mat frameAcc = Mat::zeros(424, 512, CV_32FC1);

//---------------------------------------------------------------------------
// Functions
//---------------------------------------------------------------------------

void changeBlur(int, void *strength) {
    int *st = (int *) strength;
    if (*st <= 1) {
        *st = 1;
    }
}

void toggleFilter(int, void*) {
    useFilter = !useFilter;
}

Mat calculateAverageFrame(list<Mat> *frames_, libfreenect2opencv::Libfreenect2OpenCV &libfreenect2OpenCV) {
    Mat old = *frames_->begin();
    Mat newFrame = libfreenect2OpenCV.getDepthMatUndistorted();
    imshow("diff", old - newFrame);
    frames_->pop_front();
    cv::subtract(frameAcc, old, frameAcc);
//    frameAcc = frameAcc - old;
    cv::add(frameAcc, newFrame, frameAcc);
//    frameAcc = frameAcc + newFrame;
    int framesSize = frames_->size();
    frames_->push_back(newFrame);
    assert(frames_->size() - framesSize == 1);
    imshow("avgDebug", frameAcc);
    imshow("front", *frames_->begin());
    imshow("back", frames_->back());
    return frameAcc / (double) frames_->size();
}

void test() {
    list<Mat> matrices;
    for (int i = 0; i < 5; i++) {
        matrices.push_back(Mat(5, 5, CV_32FC1, i));
    }

    cout << ( std::next(matrices.begin(), 1) - std::next(matrices.begin(), matrices.size()-1) );
}

int main() {
    Mat background;
    Mat touch;
    Mat binarized;
    Mat blur;
    Mat touchRoi;
    Mat depth;
    Mat mult;
    Mat contoursMat;
    ShapeFilterStrategy *filter = new DensityFilter();

    int touchMinArea = 13;
    int touchMaxArea = 23;
    int debugthresh = 1;
    int blurStrength = 5;
    int blurStrength2 = 3;
    int xMin = 0;
    int xMax = 512;
    int yMin = 46;
    int yMax = 357;
    int threshFG = 1518;
    int threshBG = 1600;
    int filterRadius = 20;

    const bool localClientMode = true;                    // connect to a local client
    const char *windowName = "Debug";
    const Scalar green(0, 255, 0);
    const Scalar white(255, 255, 255);
    const Scalar red(0, 0, 255);
    const Scalar blue(255, 0, 0);
    const Scalar magenta(255, 0, 255);

    // {{{ TUIO server object
    TuioServer *tuio;
    if (localClientMode) {
        tuio = new TuioServer();
        // std::cout << "--- local ---" << std::endl;
    } else {
        tuio = new TuioServer("192.168.0.2", 3333, false);
        // std::cout << "--- NOOOOOT local ---" << std::endl;
    }
    TuioTime time;
    // }}}

    // create some sliders {{{
    namedWindow(windowName);
    createTrackbar("blur strength", windowName, &blurStrength, 50, changeBlur, &blurStrength);
//    createTrackbar("blur strength 2", windowName, &blurStrength2, 50, changeBlur, &blurStrength2);
    createTrackbar("threshFG", windowName, &threshFG, 2000);
    createTrackbar("threshBG", windowName, &threshBG, 2000);
    createTrackbar("xMin", windowName, &xMin, 512);
    createTrackbar("xMax", windowName, &xMax, 512);
    createTrackbar("yMin", windowName, &yMin, 424);
    createTrackbar("yMax", windowName, &yMax, 424);
    createTrackbar("touchMinArea", windowName, &touchMinArea, 10000);
    createTrackbar("touchMaxArea", windowName, &touchMaxArea, 10000);
    createTrackbar("FilterThreshold", windowName, filter->getDensityThresholdReference(), 50);
    createTrackbar("min shape size", windowName, filter->getMinShapeSizeReference(), 2000);
    createButton("toggleFilter", toggleFilter);
    // }}}

    libfreenect2opencv::Libfreenect2OpenCV libfreenect2OpenCV;
    libfreenect2OpenCV.start();

    for(int i = 0; i < framesForAverage; i++) {
        depth = libfreenect2OpenCV.getDepthMatUndistorted();
        frames.push_back(depth);
//        cout << frameAcc.row(0).at<float>(0) << endl;
        frameAcc += depth;
//        cout << frameAcc.row(0).at<float>(0) << endl;
    }

    int timeCount = 0, shotCount = 0;
    while ((char) waitKey(1) != (char) 27) {
//        depth = libfreenect2OpenCV.getDepthMatUndistorted();

        depth = calculateAverageFrame(&frames, libfreenect2OpenCV);

//        Mat depthDebug = (threshFG < depth) & (threshBG > depth);
//
//        Mat ir = libfreenect2OpenCV.getIRMat();
//        Mat rgb = libfreenect2OpenCV.getRGBMat();
//        cv::blur(depth, blur, Size(blurStrength, blurStrength));
////        cv::threshold(blur, binarized, threshFG, 255, CV_8UC1);
//        binarized = (threshFG < blur) & (threshBG > blur);
////        cv::blur(binarized, binarized, Size(blurStrength, blurStrength));
////        cv::threshold(binarized, binarized, 1, 255, CV_8UC1);
//        binarized.convertTo(binarized, CV_8UC1);
//
//        // extract ROI
//        Rect roi(xMin, yMin, xMax - xMin, yMax - yMin);
//        touchRoi = binarized(roi);
//        // cout << touchRoi << endl;
//
//        // {{{ find touch points
//        vector<vector<Point2i> > contours;
//        vector<Point2f> touchPoints;
//        findContours(touchRoi, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point2i(xMin, yMin));
//        vector<vector<Point> > hull(contours.size());
//        for (unsigned int i = 0; i < contours.size(); i++) {
//            Mat contourMat(contours[i]);
//            convexHull(Mat(contours[i]), hull[i], false);
//
//            //          double cArea = contourArea(contourMat);
//            // if ( cArea > touchMinArea && cArea <= touchMaxArea ) {
//            // 	Scalar center = mean(contourMat);
//            // 	Point2i touchPoint(center[0], center[1]);
//            // 	touchPoints.push_back(touchPoint);
//            // }
////            vector <double> hullArea;
////            double hArea = contourArea(hull[i]);
////			if ( hArea > touchMinArea && hArea < touchMaxArea ) {
////                Point2i center = getCenter(hull[i]);
////				Point2i touchPoint(center.x, center.y);
////				touchPoints.push_back(touchPoint);
////            }
//
//        }
//        // }}}
//        // {{{ send TUIO cursors
//        time = TuioTime::getSessionTime();
//        tuio->initFrame(time);
//
//        for (unsigned int i = 0; i < touchPoints.size(); i++) { // touch points
//            float cursorX = (touchPoints[i].x - xMin) / (xMax - xMin);
//            float cursorY = 1 - (touchPoints[i].y - yMin) / (yMax - yMin);
//            // std::cout << "touchpoint[" << i << "]: x=" << cursorX << ", y=" << cursorY << std::endl;
//            TuioCursor *cursor = tuio->getClosestTuioCursor(cursorX, cursorY);
//            // TODO improve tracking (don't move cursors away, that might be closer to another touch point)
//            if (cursor == NULL) {
//                tuio->addTuioCursor(cursorX, cursorY);
//                // std::cout << "addTuioCursor[" << i << "]: x=" << cursorX << ", y=" << cursorY << std::endl;
//            } else {
//                tuio->updateTuioCursor(cursor, cursorX, cursorY);
//                // std::cout << "updateTuioCursor[" << i << "]: x=" << cursorX << ", y=" << cursorY << std::endl;
//            }
//        }
//
//        tuio->stopUntouchedMovingCursors();
//        tuio->removeUntouchedStoppedCursors();
//        tuio->commitFrame();
//
//        // }}}
//        vector<vector<Point2i> > filteredHulls;
//        if (useFilter) {
//            //hulls with a certain min size and filtered noise
//            filteredHulls = filter->filter(hull, filterRadius + 100);
//        } else {
//            filteredHulls = hull;
//        }
//        cout << "------------------------------------------------" << endl;
//        cout << "Contours: " << contours.size() << endl << "Hulls: " << hull.size() << endl;
//
//        for (vector<Point2i> hull : filteredHulls) {
//            Point2i center = getCenter(hull);
//            Point2i touchPoint(center.x, center.y);
//            touchPoints.push_back(touchPoint);
//        }
//
//        // draw debug frame {{{
//        cvtColor(binarized, contoursMat, COLOR_GRAY2BGR);
//
//        rectangle(contoursMat, roi, blue, 2); // surface boundaries
//
//        for (int i = 0; i < contours.size(); i++) {
//            drawContours(contoursMat, contours, i, green);
//        }
//
//        for (int i = 0; i < filteredHulls.size(); i++) {
//            drawContours(contoursMat, filteredHulls, i, magenta);
//            circle(contoursMat, touchPoints[i], 3, red, 3);
//        }
//
////        for (Point2f touchPoint : touchPoints) {
////            circle(contoursMat, touchPoint, 5, red, -1);
////        }
//
//        imshow(windowName, contoursMat);
//
        //show orig depth image
//        Mat depth8;
//        depth.convertTo(depth8, CV_8UC1);
//        imshow("orig depth", depth8);
        imshow("orig depth", depth);
//
////        Mat ir8;
////        ir.convertTo(ir8, CV_8UC1);
////        imshow("ir image", ir8);
////
////        imshow("rgb", rgb);
//
//        imshow("depth debug", depthDebug);
//        // }}}

    }

    libfreenect2OpenCV.stop();
    return 0;
}
