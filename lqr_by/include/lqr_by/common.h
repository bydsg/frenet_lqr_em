#ifndef common.h
#define common.h

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <fstream>

struct Point {
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};
struct Road
{
    int id;
    std::vector<int> pre;
    std::vector<int> beh;
    double distance;
    std::vector<Point> road_points;

};

#endif