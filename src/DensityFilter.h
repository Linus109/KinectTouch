//
// Created by linus on 07.07.21.
//

#ifndef KINECTTOUCH_DENSITYFILTER_H
#define KINECTTOUCH_DENSITYFILTER_H
#include "ShapeFilterStrategy.h"

class DensityFilter : public ShapeFilterStrategy {

    public:
        vector< vector<Point2i> > filter(vector< vector< Point2i> > hulls, int radius);

};


#endif //KINECTTOUCH_DENSITYFILTER_H
