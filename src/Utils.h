//
// Created by linus on 07.07.21.
//


//#pragma once
#ifndef KINECTTOUCH_UTILS_H
#define KINECTTOUCH_UTILS_H
#include <opencv2/imgproc.hpp>
#include <vector>

using namespace cv;
using namespace std;

Point2i getCenter(vector<Point2i> shape);

double distance_(const Point2i& point1, const Point2i& point2);

double distance(vector<Point2i> shape1, vector<Point2i> shape2);


#endif //KINECTTOUCH_UTILS_H
