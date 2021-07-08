//
// Created by linus on 07.07.21.
//

#ifndef KINECTTOUCH_CLUSTERFILTER_H
#define KINECTTOUCH_CLUSTERFILTER_H
#include "ShapeFilterStrategy.h"

class ClusterFilter : public ShapeFilterStrategy {
    public:
        vector< vector<Point2i> > filter(vector< vector< Point2i> > hulls, int radius);
};


#endif //KINECTTOUCH_CLUSTERFILTER_H
