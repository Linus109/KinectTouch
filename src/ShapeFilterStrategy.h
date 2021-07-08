//
// Created by linus on 07.07.21.
//

#ifndef KINECTTOUCH_SHAPEFILTERSTRATEGY_H
#define KINECTTOUCH_SHAPEFILTERSTRATEGY_H
#include <opencv2/imgproc.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "Utils.h"

using namespace std;
using namespace cv;

class ShapeFilterStrategy {
    protected:
        int densityThreshold = 7;

    public:
        virtual vector< vector<Point2i> > filter(vector< vector< Point2i> > hulls, int radius) = 0;
        virtual ~ShapeFilterStrategy() {};
        void setDensityThreshold(int densityThreshold) {
            this->densityThreshold = densityThreshold;
        }
        int* getDensityThresholdReference() { return &densityThreshold; };

};


#endif //KINECTTOUCH_SHAPEFILTERSTRATEGY_H
