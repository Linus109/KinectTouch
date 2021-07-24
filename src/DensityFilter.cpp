//
// Created by linus on 07.07.21.
//

#ifndef DENSITIY_FILTER_CPP
#define DENSITY_FILTER_CPP
#include <thread>
#include "DensityFilter.h"

vector<vector<Point2i> > DensityFilter::filter(vector<vector<Point2i>> hulls, int radius) {
    int* densityScore = new int[hulls.size()] { 0 };
    if (hulls.size() == 0)
        return hulls;

    unsigned int supportedThreads = std::thread::hardware_concurrency() * 3;
    int from = 0;
    int to = 0;
    std::mutex lock;
    int stepSize = hulls.size() / supportedThreads;
    if(hulls.size() < supportedThreads)
        filterOnRange(hulls, radius, 0, hulls.size(), densityScore, &lock);

    else {
        std::thread threads[supportedThreads];

        for (int i = 0; i < supportedThreads; i++) {
            if(i == supportedThreads - 1) {
                to = hulls.size();
            }
            else
                to = from + stepSize;

            threads[i] = std::thread(&DensityFilter::filterOnRange, this, hulls, radius, from, to, densityScore, &lock);
            from = to;
        }


        for (std::thread &thread_ : threads) {
            thread_.join();
        }

        for(int i = hulls.size(); i > 0; i--) {
            if(densityScore[i] > densityThreshold)  {
                hulls.erase(hulls.begin() + i);
            }
        }
    }
    delete[] densityScore;
    return hulls;
}

void DensityFilter::filterOnRange(vector < vector<Point2i> > hulls, int radius, int from, int to, int* densityScore, std::mutex *lock) {

    for(int i = from; i < to; i++) {

        auto shape = hulls[i];
        if(shape.size() <= 0)
            continue;
        if (cv::contourArea(shape) > minShapeSize)
            continue;

        for(int n = 0; n < hulls.size(); n++) {
            auto otherShape = hulls[n];

            if(shape == otherShape)
                continue;

            if(distance(shape, otherShape) < radius) {
//                densityScore[i]++;
                lock->lock();
                densityScore[n]++;
                lock->unlock();
            }
        }
    }



}

#endif