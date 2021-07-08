//
// Created by linus on 08.07.21.
//
#include "Utils.h"

Point2i getCenter(vector<Point2i> shape) {
    Point center(0, 0);
    for (const Point& hullPoint : shape) {
        center += hullPoint;
    }

    center.x /= shape.size();
    center.y /= shape.size();
    return center;
}

double distance_(const Point2i& point1, const Point2i& point2) {
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

double distance(vector<Point2i> shape1, vector<Point2i> shape2) {
    return distance_(getCenter(shape1), getCenter(shape2));
}
