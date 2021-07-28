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
#include <ostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <list>

using namespace std;

// openCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/ocl.hpp>

using namespace cv;

#define CV_RETR_LIST cv::RETR_LIST
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE
#define CV_CHAIN_APPROX_NONE cv::CHAIN_APPROX_NONE
#define CV_GRAY2BGR cv::COLOR_GRAY2BGR
#define CV_FILLED cv::FILLED

// libfreenect2

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
void changeBlur(int, void*);
void toggleFilter(int, void*);
Mat calculateAverageFrame(list<Mat> *frames_, libfreenect2opencv::Libfreenect2OpenCV &libfreenect2OpenCV);
void updateDistMatrix(int, void*);
vector<Point2f> aggregateTouchPoints(vector<Point2f> &touchPoints, int radius);
Mat createDistanceMatrix(int rows, int cols, Point2i originPoint, float minDistance, float pixelToMmConversionRatio);


const  int framesForAverage = 40;
list<Mat> frames;
int oldestFrameIndex = 0;
bool mousePressed = false;
bool useFilter = false;
Mat frameAcc = Mat::zeros(424, 512, CV_32FC1);
const int frameHeight = 424;
const int frameWidth = 512;
const float mmPerPixel = 4.29;
int threshFG = 1560;
int threshBG = 1600;
Mat distanceMat = createDistanceMatrix(frameHeight, frameWidth, Point(frameWidth / 2, frameHeight / 2), threshFG, mmPerPixel);

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
    cerr << "filter is: " << useFilter << endl;
}

Mat calculateAverageFrame(list<Mat> *frames_, libfreenect2opencv::Libfreenect2OpenCV &libfreenect2OpenCV) {
    Mat old = frames_->front();
    Mat newFrame;
    libfreenect2OpenCV.getDepthMatUndistorted().copyTo(newFrame);
    frameAcc = frameAcc - old;
    frameAcc = frameAcc + newFrame;
    frames_->push_back(newFrame);
    frames_->pop_front();

    return frameAcc / (double) frames_->size();
}

void updateDistMatrix(int, void*) {
    distanceMat = createDistanceMatrix(frameHeight, frameWidth, Point2i(frameWidth / 2, frameHeight / 2), threshFG, mmPerPixel);
}

vector<Point2f> aggregateTouchPoints(vector<Point2f> &touchPoints, int radius) {
    vector<Point2f> aggregatedTouchPoints;
    for (Point2f referenceTouchPoint : touchPoints) {
        vector<Point2f> toBeAggregated;
        toBeAggregated.push_back(referenceTouchPoint);
        for (Point2f compareTouchPoint : touchPoints) {
            if (distance_(referenceTouchPoint, compareTouchPoint) < radius) {
                toBeAggregated.push_back(compareTouchPoint);
            }
        }

        Point2f sum(0, 0);
        for (Point2f touchPoint : toBeAggregated) {
            sum += touchPoint;
        }
        aggregatedTouchPoints.push_back(sum / (float) toBeAggregated.size());
    }
    return aggregatedTouchPoints;
}

/**
 * Creates a distance matrix for every point of the canvas/frame to the camera using pythagoras
 * @param destination a reference to the matrix that is to be filled
 * @param originPoint The point where the distance to the camera is minDistance, most likely the middle of the frame
 * @param minDistance the distance from the originPoint to the camera
 */
Mat createDistanceMatrix(int rows, int cols, Point2i originPoint, float minDistance, float pixelToMmConversionRatio) {
    Mat matrix = Mat::zeros(rows, cols, CV_32FC1);
    for(int y = 0; y < rows; y++) {
        for(int x = 0; x < cols; x++) {
            float dist = distance_(originPoint, Point2i(x, y)) * pixelToMmConversionRatio;
            matrix.row(y).col(x) = sqrt(pow(minDistance, 2) + pow(dist, 2));
        }
    }
    return matrix;
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
    vector<Point2f> touchPoints;
    int sendToTuio = 2; //send found touchpoints to tuio server every x loops.
    ShapeFilterStrategy *filter = new DensityFilter();
//    ShapeFilterStrategy *filter = new ClusterFilter();

    int touchMinArea = 13;
    int touchMaxArea = 23;
    int debugthresh = 1;
    int blurStrength = 5;
    int blurStrength2 = 3;
    int xMin = 105;
    int xMax = 512;
    int yMin = 120;
    int yMax = 357;
    int filterRadius = 20;

    const bool localClientMode = true;                    // connect to a local client
    const char *windowName = "results";
    const Scalar green(0, 255, 0);
    const Scalar white(255, 255, 255);
    const Scalar red(0, 0, 255);
    const Scalar blue(255, 0, 0);
    const Scalar magenta(255, 0, 255);

    // {{{ TUIO server object
    TuioServer *tuio;
    if (localClientMode) {
        tuio = new TuioServer();
    } else {
        tuio = new TuioServer("192.168.0.2", 3333, false);
    }
    TuioTime time;
    // }}}

    // create some sliders {{{
    namedWindow(windowName);
    createTrackbar("blur strength", windowName, &blurStrength, 50, changeBlur, &blurStrength);
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
    createTrackbar("filterRadius", windowName, &filterRadius, 100);
    createButton("toggleFilter", toggleFilter);
    createButton("update Distance Matrix", updateDistMatrix);
    // }}}

    libfreenect2opencv::Libfreenect2OpenCV libfreenect2OpenCV;
    libfreenect2OpenCV.start();

    Mat tmpMat;
    for(int i = 0; i < framesForAverage; i++) {
        libfreenect2OpenCV.getDepthMatUndistorted().copyTo(tmpMat);
        frames.push_back(tmpMat);
        frameAcc += tmpMat;
    }

    while ((char) waitKey(1) != (char) 27) {
        auto start = std::chrono::high_resolution_clock::now();
//        updateDistMatrix(4, &distanceMat);

        depth = calculateAverageFrame(&frames, libfreenect2OpenCV);

        Mat ir = libfreenect2OpenCV.getIRMat();
        Mat rgb = libfreenect2OpenCV.getRGBMat();
        cv::blur(depth, blur, Size(blurStrength, blurStrength));

//        binarized = (blur > threshFG) & (blur < threshBG);

        binarized = Mat::zeros(blur.rows, blur.cols, CV_8UC1);

        for(int y = 0; y < binarized.rows; y++) {
            for(int x = 0; x < binarized.cols; x++) {
                float value = blur.at<float>(y, x);
                bool isRelevant = value > threshFG && value < threshBG;
                if(!isRelevant) {
                    binarized.at<uchar>(y,x) = 255;
                }
            }
        }

//        binarized = (distanceMat > blur);
//        binarized = (threshFG > blur);
//        bitwise_not(binarized, binarized);

        // extract ROI
        Rect roi(xMin, yMin, xMax - xMin, yMax - yMin);
        touchRoi = binarized(roi);

        // {{{ find touch points
        vector<vector<Point2i> > contours;

        findContours(touchRoi, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point2i(xMin, yMin));
        vector<vector<Point> > hull(contours.size());
        for (unsigned int i = 0; i < contours.size(); i++) {
            convexHull(Mat(contours[i]), hull[i], false);
        }
        // }}}

        // }}}
        vector<vector<Point2i> > filteredHulls;
        if (useFilter) {
            //hulls with a certain min size and filtered noise
            filteredHulls = filter->filter(hull, filterRadius + 100);
        } else {
            filteredHulls = hull;
        }

        for (vector<Point2i> hull : filteredHulls) {
            Point2i center = getCenter(hull);
            Point2i touchPoint(center.x, center.y);
            touchPoints.push_back(touchPoint);
        }

        touchPoints = aggregateTouchPoints(touchPoints, filterRadius);

        // {{{ send TUIO cursors
        time = TuioTime::getSessionTime();
        tuio->initFrame(time);

        for (unsigned int i = 0; i < touchPoints.size(); i++) { // touch points
            float cursorX = (touchPoints[i].x - xMin) / (xMax - xMin);
            float cursorY = 1 - (touchPoints[i].y - yMin) / (yMax - yMin);
            TuioCursor *cursor = tuio->getClosestTuioCursor(cursorX, cursorY);

            // TODO improve tracking (don't move cursors away, that might be closer to another touch point)
            if (cursor == NULL) {
                tuio->addTuioCursor(cursorX, cursorY);
            } else {
                tuio->updateTuioCursor(cursor, cursorX, cursorY);
            }
        }

        tuio->stopUntouchedMovingCursors();
        tuio->removeUntouchedStoppedCursors();
        tuio->commitFrame();

        // draw debug frame {{{
        cvtColor(binarized, contoursMat, COLOR_GRAY2BGR);

        rectangle(contoursMat, roi, blue, 2); // surface boundaries
        circle(contoursMat, Point2i(frameWidth/2, frameHeight/2), 8, magenta, -1);

        for (int i = 0; i < contours.size(); i++) {
            drawContours(contoursMat, contours, i, green);
        }

        for (int i = 0; i < filteredHulls.size(); i++) {
            drawContours(contoursMat, filteredHulls, i, magenta);
            circle(contoursMat, touchPoints[i], 3, red, 3);
        }

        for (Point2f touchPoint : touchPoints) {
            circle(contoursMat, touchPoint, 5, red, -1);
        }

        imshow(windowName, contoursMat);
//        imshow("avg depth", depth);
        // }}}

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        cout << "Loop duration: " << duration.count() << endl;

    }

    delete filter;
    libfreenect2OpenCV.stop();
    return 0;
}
