#ifndef C___CENTROIDTRACKER_H
#define C___CENTROIDTRACKER_H

#endif //C___CENTROIDTRACKER_H

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <set>
#include <algorithm>
#include <darknet_ros_msgs/BoundingBoxes.h>
class CentroidTracker {
public:
    explicit CentroidTracker();

    void register_Object(int cX, int cY);

    std::vector<std::pair<int, std::pair<int, int>>> update(std::vector<darknet_ros_msgs::BoundingBox> boxes);

    // <ID, centroids>
    std::vector<std::pair<int, std::pair<int, int>>> objects;


private:
    int maxDisappeared;

    int nextObjectID;

    static double calcDistance(double x1, double y1, double x2, double y2);

    // <ID, count>
    std::map<int, int> disappeared;
};