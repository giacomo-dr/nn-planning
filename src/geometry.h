// geometry.h
// Author: Giacomo Del Rio
// Date: 14 Apr 2017

#ifndef GEOMETRY_H
#define GEOMETRY_H


#include <vector>
#include <Eigen/Dense>


typedef Eigen::Vector2d Point2D;
typedef Eigen::Vector3d Point3D;

struct WaypointPath2D {
    std::vector<Point2D> waypoints;

    WaypointPath2D(){}
    WaypointPath2D( std::vector<Point2D> waypoints ) :
            waypoints(waypoints){}
    void clear(){ waypoints.clear(); }
};

#endif //GEOMETRY_H
