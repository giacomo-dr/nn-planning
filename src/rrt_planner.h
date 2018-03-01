// rrt_planner.h
// Author: Giacomo Del Rio
// Date: 17 Apr 2017

#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <vector>
#include <boost/geometry/index/rtree.hpp>
#include "geometry.h"
#include "height_map.h"


namespace bgi = boost::geometry::index;


struct RRTNode {
    Point2D vertex;
    int parent;
    std::vector<int> children;
    double probability;

    RRTNode() : parent(-1), probability(0) {}
    RRTNode( Point2D vertex, int parent, double probability ) :
            vertex(vertex), parent(parent), probability(probability) {}
};

struct RRTPlan {
    long root = -1;
    std::vector<RRTNode> nodes;

    void clear(){ root = -1; nodes.clear(); }
    long add_node( RRTNode n );
};

class RRTPlanner {
public:
    struct Parameters{
        double growth_factor;               // Branch segment size
        double max_segment_angle;           // Maximum angle between two adjacent segments
        unsigned int greediness;            // Select target as growing point each greediness iteration
        unsigned int max_iterations;        // Maximum number of growing attempt
        double traversability_threshold;    // Allow a segment only if traversable with a prob greater than this
        int grow_to_target_neighbors;       // Number of neighbors considered when growing to target
        int grow_to_point_neighbors;        // Number of neighbors considered when growing to a point
    };

public:
    RRTPlanner();
    RRTPlanner( HeightMap* map, const Parameters& params );
    void set_parameters( const Parameters& params );
    const Parameters& get_parameters() const;
    void set_map( HeightMap* map );
    const RRTPlan& get_plan() const;
    const WaypointPath2D& get_path() const;
    Point2D get_start_point() const;
    Point2D get_target_point() const;
    int build_plan( const Point2D& start, double start_yaw,
                    const Point2D& target, double target_yaw );
    bool is_traversable( const Point2D& p1, const Point2D& p2 ) const;

private:
    Parameters params = {            // Default vanilla parameters
            .growth_factor = 1.0,
            .max_segment_angle = M_PI_2,
            .greediness = 10,
            .max_iterations = 10000,
            .traversability_threshold = 0.95,
            .grow_to_target_neighbors = 10,
            .grow_to_point_neighbors = 1
    };
    HeightMap* map;
    Point2D start_point, target_point;
    double start_yaw, target_yaw;
    double traversability_threshold_log;

    RRTPlan rrt;
    typedef std::pair<Point2D, int> RTreeValue;
    bgi::rtree< RTreeValue, bgi::rstar<16> > nodes_index;
    WaypointPath2D shortest_path;

private:
    void reset();
    long expand_to_target();
    void expand_to_point( const Point2D& to );
    void build_shortest_path( long final_node_idx );
    Point2D compute_step( const Point2D& p1, const Point2D& p2 ) const;
    double step_probability( const Point2D& p1, const Point2D& p2 ) const;
    bool has_similar_sibling( const RRTNode& n, const Point2D& to ) const;
    double abs_angle( const RTreeValue& n, const Point2D& p2 ) const;
    bool in_bounds( const Point2D& p ) const;
    static double angle_between( const Point2D& from, const Point2D& to );
    static double angle_difference( double alpha_1, double alpha_2 );
    static double get_yaw( const Point2D& p );
};

#endif //RRT_PLANNER_H
