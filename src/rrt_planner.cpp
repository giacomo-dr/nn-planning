// rrt_planner.cpp
// Author: Giacomo Del Rio
// Date: 17 Apr 2017


#include <stdexcept>
#include <cmath>
#include "rrt_planner.h"


typedef boost::geometry::model::box<Point2D> Box;


long RRTPlan::add_node( RRTNode n ){
    nodes.push_back(n);
    int node_id = (int)nodes.size() - 1;
    if( n.parent != -1 )
        nodes[n.parent].children.push_back( node_id );
    return node_id;
}

RRTPlanner::RRTPlanner(){
    this->map = nullptr;
    start_point.setZero();
    target_point.setZero();
    start_yaw = target_yaw = 0;
    traversability_threshold_log = log(params.traversability_threshold);
}

RRTPlanner::RRTPlanner( HeightMap* map, const Parameters& params )
        : params(params), map(map) {
    start_point.setZero();
    target_point.setZero();
    start_yaw = target_yaw = 0;
    traversability_threshold_log = log(params.traversability_threshold);
}

void RRTPlanner::set_parameters( const Parameters& params ){
    this->params = params;
    traversability_threshold_log = log(params.traversability_threshold);
}

const RRTPlanner::Parameters& RRTPlanner::get_parameters() const{
    return params;
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

Point2D RRTPlanner::get_start_point() const{
    return start_point;
}

Point2D RRTPlanner::get_target_point() const{
    return target_point;
}

int RRTPlanner::build_plan( const Point2D& start, double start_yaw,
                            const Point2D& target, double target_yaw ){
    reset();
    map->seed_Cfree_sampler();
    this->start_point = start;   this->target_point = target;
    this->start_yaw = start_yaw; this->target_yaw = target_yaw;

    // Add starting point
    rrt.root = rrt.add_node( RRTNode( start, -1, 0 ) );
    nodes_index.insert( std::make_pair(start, rrt.root) );

    // Grow the tree until target is reached or we have hit
    // the maximum number of iteration
    int current_iteration = 0;
    while( current_iteration++ < params.max_iterations )
        if( current_iteration % params.greediness == 0 ){
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
    // Consider all the nodes in the tree that lie within a box centered to 'target_point'
    // with size proportional to growth_factor
    double r = params.growth_factor * 1.01;
    Box neighbors_box( target_point - Point2D(r, r), target_point + Point2D(r, r) );
    for( auto p = nodes_index.qbegin( bgi::within(neighbors_box) ) ; p != nodes_index.qend() ; ++p ){
        if( (p->first - target_point).norm() <= params.growth_factor ){
            // Try to connect directly to target
            double step_prob = std::log( step_probability( p->first, target_point ));
            if( step_prob > traversability_threshold_log &&
                abs_angle( *p, target_point ) < params.max_segment_angle &&
                std::fabs( angle_difference( get_yaw( target_point - p->first ), target_yaw )) < params.max_segment_angle / 2.0 ){
                double branch_prob = rrt.nodes[p->second].probability;
                // Direct connection with target found!
                long idx = rrt.add_node( RRTNode( target_point, p->second, step_prob + branch_prob ));
                nodes_index.insert( std::make_pair( target_point, idx ));
                std::cout << "\t\tTarget found and put in node " << idx << "\n";
                std::cout << "\t\tTotal path probability to target " << std::exp( step_prob + branch_prob ) << "\n";
                return idx;
            }
        }
    }

    // No direct connection with target found, expand toward it
    expand_to_point( target_point );
    return -1;
}

void RRTPlanner::expand_to_point( const Point2D& to ){
//  std::cout << "Expand to (" << to.x() << ", " << to.y() << ")\n";
    // Find nearest n nodes to expansion point

    for( auto p = nodes_index.qbegin( bgi::nearest(to, params.grow_to_point_neighbors) ); p != nodes_index.qend(); ++p ){
//        std::cout << "  Considering (" << p.first.x() << ", " << p.first.y() << ")\n";
//        std::cout << "    Delta vector (" << (to - p.first).x() << ", " << (to - p.first).y() << ")\n";
        if( abs_angle( *p, to ) < params.max_segment_angle && !has_similar_sibling( rrt.nodes[p->second], to ) ){
            Point2D step = compute_step( p->first, to );
//            std::cout << "    Step (" << step.x() << ", " << step.y() << ")\n";
            // Test for traversability
            double step_prob = std::log( step_probability( p->first, to ) );
//            std::cout << "    Step probability: " << step_prob << " >? " << traversability_threshold << "\n";
            if( in_bounds( step ) && step_prob > traversability_threshold_log ){
                double branch_prob = rrt.nodes[p->second].probability;
//                std::cout << "    Branch probability: " << std::exp(branch_prob) << "\n";
                long idx = rrt.add_node( RRTNode( step, p->second, step_prob + branch_prob ) );
                nodes_index.insert( std::make_pair(step, idx) );
//                std::cout << "    Expanded!\n";
                return; // Tree expanded
            }
        }
    }

    // Tree not expanded, just return
//    std::cout << "    Not expanded!\n";
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
    return p1 + params.growth_factor * (p2-p1).normalized();
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
        if( abs( angle_difference( angle_b, angle_a ) ) < params.max_segment_angle / 2.0 )
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
    double step_prob = std::log( step_probability( p1, p2 ) );
    return in_bounds( p2 ) && step_prob > traversability_threshold_log;
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
