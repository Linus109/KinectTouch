//
// Created by linus on 07.07.21.
//

#ifndef DENSITIY_FILTER_CPP
#define DENSITY_FILTER_CPP

#include "DensityFilter.h"

vector<vector<Point2i> > DensityFilter::filter(vector<vector<Point2i>> hulls, int radius) {
    int* densityScore = new int[hulls.size()] { 0 };
    for(int i = 0; i < hulls.size(); i++) {
        auto shape = hulls[i];

        for(int n = 0; n < hulls.size(); n++) {
            auto otherShape = hulls[n];

            if(shape == otherShape)
                continue;

            if(distance(shape, otherShape) < radius) {
//                densityScore[i]++;
                densityScore[n]++;
            }
        }
    }

    for(int i = hulls.size(); i > 0; i--) {
        if(densityScore[i] > densityThreshold)  {
            hulls.erase(hulls.begin() + i);
        }
    }

    delete[] densityScore;
    return hulls;
}

#endif