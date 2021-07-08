//
// Created by linus on 07.07.21.
//

#include "ClusterFilter.h"

/**
 * Detects clusters of small shapes (noise) and removes them
 * @param hulls vector of shapes
 * @param radius
 * @return
 */
vector< vector<Point2i> > ClusterFilter::filter(vector< vector<Point2i> > hulls, int radius) {

    int innerClusterRadius = radius * 0.66;
    vector<int> cluster;
    vector< vector<Point2i> > result;

    for(int i = 0; i < hulls.size(); i++ ) {
        double area = cv::contourArea(hulls[i]);
        cout << "Area: " << area << endl;
        if(area < 20) {
            if (hulls[i].size() != 0) {
                result.push_back(hulls[i]);
            } else {
                cout << "hulls size is 0" << endl;
            }
        }
    }
    int smallShapes = result.size();

    //all small shapes...
    for(auto shape : result) {
        if (shape.empty())
            continue;
        Point2i center = getCenter(shape);
        // outer radius
        for (int i = 0; i < result.size(); i++) {
            Point2i compareCenter = getCenter(result[i]);
            if (center != compareCenter && distance_(center, compareCenter) < radius) {   //check if center of shape is within radius
                cluster.push_back(i);
            }
        }

        //inner radius
        if(cluster.size() >= 10) {
            for(int i = cluster.size(); i > 0; i--) {
                Point2i compareCenter = getCenter(result.at(i)); //gg
                if(distance_(center, compareCenter) < innerClusterRadius) {
                    result.erase(result.begin() + i);// ¯\_(ツ)_/¯
                }
            }
        }
        cluster.clear();
        cout << "Small shapes: " << smallShapes << endl << "filtered: " << smallShapes - result.size() << endl;
    }
    return result;
}

