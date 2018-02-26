// rrt_star_planner.h
// Author: Giacomo Del Rio
// Date: 26 Feb 2018

#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include <vector>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include "geometry.h"
#include "height_map.h"

BOOST_GEOMETRY_REGISTER_POINT_2D( Point2D, double, cs::cartesian, x(), y() )

namespace bgi = boost::geometry::index;


struct RRTStarNode {
    Point2D vertex;
    int parent;
    std::vector<int> children;
    double probability;

    RRTStarNode() : parent(-1), probability(0) {}
    RRTStarNode( Point2D vertex, int parent, double probability ) :
                 vertex(vertex), parent(parent), probability(probability) {}
};

struct RRTStarPlan {
    long root = -1;
    std::vector<RRTStarNode> nodes;

    void clear(){ root = -1; nodes.clear(); }
    long add_node( RRTStarNode n );
};

class RRTStarPlanner{
public:
    RRTStarPlanner();
    RRTStarPlanner( HeightMap* map, double growth_factor,
                    unsigned int greediness, unsigned int max_iterations,
                    double max_segment_angle, double traversability_threshold );
    void set_parameters( double growth_factor, unsigned int greediness,
                         unsigned int max_iterations, double max_segment_angle,
                         double traversability_threshold );
    void set_map( HeightMap* map );
    const RRTStarPlan& get_plan() const;
    const WaypointPath2D& get_path() const;
    int build_plan( Point2D start, double start_yaw,
                    Point2D target, double target_yaw );
    bool is_traversable( const Point2D& p1, const Point2D& p2 ) const;

protected:
    HeightMap* map;
    double growth_factor;
    unsigned int greediness;
    unsigned int max_iterations;
    double max_segment_angle;
    double traversability_threshold;

    RRTStarPlan rrt;
    typedef std::pair<Point2D, int> RTreeStarValue;
    bgi::rtree< RTreeStarValue, bgi::rstar<16> > nodes_index;
    WaypointPath2D shortest_path;

    Point2D start_point, target_point;
    double start_yaw, target_yaw;

protected:
    void reset();
    long expand_to_target();
    void expand_to_point( const Point2D& to );
    void build_shortest_path( long final_node_idx );
    Point2D compute_step( const Point2D& p1, const Point2D& p2 ) const;
    double step_probability( const Point2D& p1, const Point2D& p2 ) const;
    bool has_similar_sibling( const RRTStarNode& n, const Point2D& to ) const;
    double abs_angle( const RTreeStarValue& n, const Point2D& p2 ) const;
    bool in_bounds( const Point2D& p ) const;
    static double angle_between( const Point2D& from, const Point2D& to );
    static double angle_difference( double alpha_1, double alpha_2 );
    static double get_yaw( const Point2D& p );
};

#endif //RRT_STAR_PLANNER_H
