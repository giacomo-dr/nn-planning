// path_planner.h
// Author: Giacomo Del Rio
// Date: 17 Apr 2017

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include "geometry.h"
#include "height_map.h"

BOOST_GEOMETRY_REGISTER_POINT_2D( Point2D, double, cs::cartesian, x(), y() )

namespace bgi = boost::geometry::index;


struct RRTNode {
    Point2D vertex;
    int parent;
    double probability;

    RRTNode() : parent(-1), probability(0) {}
    RRTNode( Point2D vertex, int parent ) :
            vertex(vertex), parent(parent), probability(0) {}
    RRTNode( Point2D vertex, int parent, double probability ) :
            vertex(vertex), parent(parent), probability(probability) {}
};

struct RRTPlan {
    long root = -1;
    std::vector<RRTNode> nodes;

    void clear(){ root = -1; nodes.clear(); }
    long add_node( RRTNode n ){ nodes.push_back(n); return nodes.size() -1; }
};

class RRTPlanner {
public:
    RRTPlanner();
    RRTPlanner( HeightMap* map, double growth_factor,
                unsigned int greediness, unsigned int max_iterations,
                double max_segment_angle, double traversability_threshold );
    void set_parameters( double growth_factor, unsigned int greediness,
                         unsigned int max_iterations, double max_segment_angle,
                         double traversability_threshold );
    void set_map( HeightMap* map );
    const RRTPlan& get_plan() const;
    const WaypointPath2D& get_path() const;
    int build_plan( Point2D start, double start_yaw,
                    Point2D target, double target_yaw );

private:
    HeightMap* map;
    double growth_factor;
    unsigned int greediness;
    unsigned int max_iterations;
    double max_segment_angle;
    double traversability_threshold;

    RRTPlan rrt;
    typedef std::pair<Point2D, int> RTreeValue;
    bgi::rtree< RTreeValue, bgi::rstar<16> > nodes_index;
    WaypointPath2D shortest_path;

    Point2D start_point, target_point;
    double start_yaw, target_yaw;

private:
    void reset();
    long expand_to_target();
    void expand_to_point( Point2D p );
    void build_shortest_path( long final_node_idx );
    Point2D compute_step( const Point2D& p1, const Point2D& p2 ) const;
    double step_probability( const Point2D& p1, const Point2D& p2 ) const;
    double abs_angle( const RTreeValue& n, const Point2D& p2 ) const;
    bool is_traversable( const Point2D& p1, const Point2D& p2 ) const;
    bool in_bounds( const Point2D& p ) const;
    static double angle_between( const Point2D& from, const Point2D& to );
    static double angle_difference( double alpha_1, double alpha_2 );
    static double get_yaw( const Point2D& p );
};


#endif //PATH_PLANNER_H
