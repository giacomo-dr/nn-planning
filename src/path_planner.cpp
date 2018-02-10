// path_planner.cpp
// Author: Giacomo Del Rio
// Date: 17 Apr 2017


#include <stdexcept>
#include <cmath>
#include "path_planner.h"

#define PP_DEFAULT_GROWTH_FACTOR 1
#define PP_DEFAULT_GREEDYNESS 10
#define PP_DEFAULT_MAX_ITERATIONS 2000
#define PP_NEIGHBORS_EXPAND 30


long RRTPlan::add_node( RRTNode n ){
    nodes.push_back(n);
    int node_id = (int)nodes.size() - 1;
    if( n.parent != -1 )
        nodes[n.parent].children.push_back( node_id );
    return node_id;
}

RRTPlanner::RRTPlanner(){
    this->map = NULL;
    this->growth_factor = PP_DEFAULT_GROWTH_FACTOR;
    this->greediness = PP_DEFAULT_GREEDYNESS;
    this->max_iterations = PP_DEFAULT_MAX_ITERATIONS;
    start_point.setZero();
    target_point.setZero();
    start_yaw = target_yaw = 0;
}

RRTPlanner::RRTPlanner( HeightMap* map, double growth_factor,
                        unsigned int greediness, unsigned int  max_iterations,
                        double max_segment_angle, double traversability_threshold )
        : map(map){
    this->growth_factor = growth_factor;
    this->greediness = greediness;
    this->max_iterations = max_iterations;
    this->max_segment_angle = max_segment_angle;
    this->traversability_threshold = log(traversability_threshold);
    start_point.setZero();
    target_point.setZero();
    start_yaw = target_yaw = 0;
}

void RRTPlanner::set_parameters( double growth_factor, unsigned int greediness,
                                 unsigned int max_iterations, double max_segment_angle,
                                 double traversability_threshold ){
    this->growth_factor = growth_factor;
    this->greediness = greediness;
    this->max_iterations = max_iterations;
    this->max_segment_angle = max_segment_angle;
    this->traversability_threshold = log(traversability_threshold);
}

void RRTPlanner::set_map( HeightMap* map ){
    this->map = map;
}

const RRTPlan& RRTPlanner::get_plan() const{
    return rrt;
}

const WaypointPath2D& RRTPlanner::get_path() const {
    return shortest_path;
}

int RRTPlanner::build_plan( Point2D start, double start_yaw,
                            Point2D target, double target_yaw ){
    reset();
    this->start_point = start;   this->target_point = target;
    this->start_yaw = start_yaw; this->target_yaw = target_yaw;

    // Add starting point
    rrt.root = rrt.add_node( RRTNode( start, -1, 0 ) );
    nodes_index.insert( std::make_pair(start, rrt.root) );

    // Grow the tree until target is reached or we have hit
    // the maximum number of iteration
    int current_iteration = 0;
    while( current_iteration++ < max_iterations )
        if( current_iteration % greediness == 0 ){
            long target_node_idx = expand_to_target();
            if( target_node_idx > -1 ){
                build_shortest_path( target_node_idx );
                return 0; // Target reached
            }
        }else{
            expand_to_point( map->sample_Cfree(MapOrigin::CENTER_CENTER) );
        }

    return -1; // Max iteration reached
}

void RRTPlanner::reset(){
    rrt.clear();
    nodes_index.clear();
    shortest_path.clear();
}

long RRTPlanner::expand_to_target(){
    // Find nearest n nodes to target
    std::vector<RTreeValue> nearest_k;
    std::copy( nodes_index.qbegin(bgi::nearest(target_point, PP_NEIGHBORS_EXPAND * 5)),
               nodes_index.qend(),
               std::back_inserter(nearest_k) );
    for( const RTreeValue& p: nearest_k )
        if( (p.first - target_point).norm() <= growth_factor ){
            // Try to connect directly to target
            double step_prob = std::log( step_probability( p.first, target_point ) );
            if( step_prob > traversability_threshold &&
                abs_angle( p, target_point ) < max_segment_angle &&
                std::fabs( angle_difference( get_yaw(target_point - p.first), target_yaw ) ) < max_segment_angle ){
                double branch_prob = rrt.nodes[p.second].probability;
                // Direct connection with target found!
                long idx = rrt.add_node( RRTNode( target_point, p.second, step_prob + branch_prob ) );
                nodes_index.insert( std::make_pair(target_point, idx) );
                std::cout << "Target found and put in node " << idx << "\n";
                std::cout << "Total path probability to target " << std::exp(step_prob + branch_prob) << "\n";
                return idx;
            }
        }

    // No direct connection with target found, expand toward it
    expand_to_point( target_point );
    return -1;
}

void RRTPlanner::expand_to_point( Point2D to ){
    std::cout << "Expand to (" << to.x() << ", " << to.y() << ")\n";
    // Find nearest n nodes to expansion point
    std::vector<RTreeValue> nearest_k;
    std::copy( nodes_index.qbegin(bgi::nearest(to, PP_NEIGHBORS_EXPAND)),
               nodes_index.qend(),
               std::back_inserter(nearest_k) );

    for( const RTreeValue& p: nearest_k ){
        std::cout << "  Considering (" << p.first.x() << ", " << p.first.y() << ")\n";
        std::cout << "    Delta vector (" << (to - p.first).x() << ", " << (to - p.first).y() << ")\n";
        if( abs_angle( p, to ) < max_segment_angle && !has_similar_sibling( rrt.nodes[p.second], to ) ){
            Point2D step = compute_step( p.first, to );
            std::cout << "    Step (" << step.x() << ", " << step.y() << ")\n";
            // Test for traversability
            double step_prob = std::log( step_probability( p.first, to ) );
            std::cout << "    Step probability: " << step_prob << " >? " << traversability_threshold << "\n";
            if( in_bounds( step ) && step_prob > traversability_threshold ){
                double branch_prob = rrt.nodes[p.second].probability;
                std::cout << "    Branch probability: " << std::exp(branch_prob) << "\n";
                long idx = rrt.add_node( RRTNode( step, p.second, step_prob + branch_prob ) );
                nodes_index.insert( std::make_pair(step, idx) );
                std::cout << "    Expanded!\n";
                return; // Tree expanded
            }
        }
    }

    // Tree not expanded, just return
    std::cout << "    Not expanded!\n";
}

void RRTPlanner::build_shortest_path( long final_node_idx ){
    shortest_path.clear();
    RRTNode n = rrt.nodes[final_node_idx];
    do{
//        std::cout << "Pushing (" << n.vertex.x() << ", " << n.vertex.y() << ")\n";
        shortest_path.waypoints.push_back( n.vertex );
        n = rrt.nodes[n.parent];
    }while( n.parent != -1 );
    shortest_path.waypoints.push_back( n.vertex );

    std::reverse( shortest_path.waypoints.begin(), shortest_path.waypoints.end() );
}

Point2D RRTPlanner::compute_step( const Point2D& p1, const Point2D& p2 ) const {
    return p1 + growth_factor * (p2-p1).normalized();
}

double RRTPlanner::step_probability( const Point2D& p1, const Point2D& p2 ) const {
    double yaw = get_yaw( p2 - p1 );
    yaw = yaw >= 0 ? yaw : 2.0 * M_PI + yaw;
//    std::cout << "    yaw = " << get_yaw( p2 - p1 ) << " abs(yaw) = " << yaw << "\n";
    return map->traversability_prob( MapOrigin::CENTER_CENTER, p1.x(), p1.y(), yaw );
}

bool RRTPlanner::has_similar_sibling( const RRTNode& n, const Point2D& to ) const {
    double angle_a = get_yaw( to - n.vertex );
    for( int i: n.children ){
        double angle_b = get_yaw( rrt.nodes[i].vertex - n.vertex );
        if( abs( angle_difference( angle_b, angle_a ) ) < max_segment_angle / 2.0 )
            return true;
    }
    return false;
}

double RRTPlanner::abs_angle( const RTreeValue& n, const Point2D& p2 ) const {
    double angle_a, angle_b;
    if( rrt.nodes[n.second].parent == -1 )
        angle_a = start_yaw; // Root node
    else {
        //std::cout << "    angle_a computed from " << n.first << " and " << rrt.nodes[rrt.nodes[n.second].parent].vertex << "\n";
        //std::cout << "    which is " << n.first - rrt.nodes[rrt.nodes[n.second].parent].vertex << "\n";
        angle_a = get_yaw( n.first - rrt.nodes[rrt.nodes[n.second].parent].vertex ); // Inner node
    }
    angle_b = get_yaw( p2 - n.first );
//    std::cout << "    angle_a = " << angle_a << ", angle_b = " << angle_b << " => ";
//    std::cout << "Angle = " << std::fabs( angle_difference( angle_b, angle_a ) ) << "\n";
    return std::fabs( angle_difference( angle_b, angle_a ) );
}

bool RRTPlanner::is_traversable( const Point2D& p1, const Point2D& p2 ) const {
//    std::cout << "    patch from (" << p1.x() << ", " << p1.y() << ") to ("
//              << p2.x() << ", " << p2.y() << ")\n" ;
//    Point2D patch_center = (p2 + p1) / 2.0;
//    std::cout << "    patch_center = (" << patch_center.x() << ", " << patch_center.y() << ")\n";
//    std::cout << "    patch_angle = " << get_yaw( p2 - p1 ) << "\n";
//    cv::Mat patch = map->extract_patch( patch_center.x(), patch_center.y(),
//                                        0.7, growth_factor, get_yaw( p2 - p1 ),
//                                        MapOrigin::CENTER_CENTER );
//    double min, max;
//    cv::minMaxLoc( patch, &min, &max );
//    return 1.0 - std::fabs(max - min) / 256.0;
    double step_prob = std::log( step_probability( p1, p2 ) );
    return in_bounds( p2 ) && step_prob > traversability_threshold;
}

bool RRTPlanner::in_bounds( const Point2D &p ) const {
    return (-map->size_x_mt() <= 2.0 * p.x()) &&
           (2.0 * p.x() <= map->size_x_mt()) &&
           (-map->size_y_mt() <= 2.0 * p.y()) &&
           (2.0 * p.y() <= map->size_y_mt());
}

double RRTPlanner::angle_between( const Point2D& from, const Point2D& to ){
    return angle_difference( get_yaw( to ), get_yaw( from ) );
}

double RRTPlanner::angle_difference( double alpha_1, double alpha_2 ){
    double diff = alpha_1 - alpha_2;
    if( diff < -M_PI )
        diff += 2 * M_PI;
    else if( diff > M_PI )
        diff -= 2 * M_PI;

    return diff;
}

double RRTPlanner::get_yaw( const Point2D& p ){
    return std::atan2( p.y(), p.x() );
}


