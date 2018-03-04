// rrt_star_planner.h
// Author: Giacomo Del Rio
// Date: 26 Feb 2018

#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include <vector>
#include <boost/geometry/index/rtree.hpp>
#include "geometry.h"
#include "height_map.h"


namespace bgi = boost::geometry::index;
typedef boost::geometry::model::box<Point2D> Box;

struct RRTStarNode {
    Point2D vertex;
    int parent;
    std::vector<int> children;
    double probability;
    double cost;

    RRTStarNode() : parent(-1), probability(0), cost(0) {}
    RRTStarNode( Point2D vertex, int parent, double probability, double cost ) :
                 vertex(vertex), parent(parent), probability(probability), cost(cost) {}
};

struct RRTStarPlan {
    long root = -1;
    std::vector<RRTStarNode> nodes;

    void clear(){ root = -1; nodes.clear(); }
    long add_node( RRTStarNode n );
};

class RRTStarPlanner{
public:
    struct Parameters{
        double growth_factor;               // Branch segment size
        double max_segment_angle;           // Maximum angle between two adjacent segments
        unsigned int greediness;            // Select target as growing point each greediness iteration
        unsigned int max_iterations;        // Maximum number of growing attempt
        double traversability_threshold;    // Allow a segment only if traversable with a prob greater than this
        double neighbors_factor;            // Adjust the size of the area when looking for neighbors
    };

public:
    RRTStarPlanner();
    RRTStarPlanner( HeightMap* map, const Parameters& params );
    void set_parameters( const Parameters& params );
    const Parameters& get_parameters() const;
    void set_map( HeightMap* map );
    const RRTStarPlan& get_plan() const;
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
            .neighbors_factor = 2.0
    };
    HeightMap* map;
    RRTStarPlan rrt;
    typedef std::pair<Point2D, int> RTreeStarValue;
    bgi::rtree< RTreeStarValue, bgi::rstar<16> > nodes_index;
    WaypointPath2D shortest_path;

    Point2D start_point, target_point;
    double start_yaw, target_yaw;
    double traversability_threshold_log;

private:
    void reset();
    long expand_to_target();
    void expand_to_point( const Point2D& to );
    void build_shortest_path( long final_node_idx );
    Point2D compute_step( const Point2D& p1, const Point2D& p2 ) const;
    double neighbors_radius() const;
    double step_probability( const Point2D& p1, const Point2D& p2 ) const;
    bool has_similar_sibling( const RRTStarNode& n, const Point2D& to ) const;
    const RTreeStarValue& nearest( const Point2D& to ) const;
    double abs_angle( const RTreeStarValue& n, const Point2D& p2 ) const;
    bool in_bounds( const Point2D& p ) const;
    static double angle_between( const Point2D& from, const Point2D& to );
    static double angle_difference( double alpha_1, double alpha_2 );
    static double get_yaw( const Point2D& p );
    static double distance( const Point2D& p1, const Point2D& p2 );
    static double lin_combination( double a, double b, double weight );
};

#endif //RRT_STAR_PLANNER_H
