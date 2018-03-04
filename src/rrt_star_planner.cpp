// rrt_star_planner.cpp
// Author: Giacomo Del Rio
// Date: 26 Feb 2018


#include <stdexcept>
#include <cmath>
#include <boost/geometry.hpp>
#include "rrt_star_planner.h"


long RRTStarPlan::add_node( RRTStarNode n ){
    nodes.push_back(n);
    int node_id = (int)nodes.size() - 1;
    if( n.parent != -1 )
        nodes[n.parent].children.push_back( node_id );
    return node_id;
}

RRTStarPlanner::RRTStarPlanner(){
    this->map = nullptr;
    start_point.setZero();
    target_point.setZero();
    start_yaw = target_yaw = 0;
    traversability_threshold_log = log(params.traversability_threshold);
}

RRTStarPlanner::RRTStarPlanner( HeightMap* map, const Parameters& params )
        : params(params), map(map){
    start_point.setZero();
    target_point.setZero();
    start_yaw = target_yaw = 0;
    traversability_threshold_log = log(params.traversability_threshold);
}

void RRTStarPlanner::set_parameters( const Parameters& params ){
    this->params = params;
    traversability_threshold_log = log(params.traversability_threshold);
}

const RRTStarPlanner::Parameters& RRTStarPlanner::get_parameters() const{
    return params;
}

void RRTStarPlanner::set_map( HeightMap* map ){
    this->map = map;
}

const RRTStarPlan& RRTStarPlanner::get_plan() const{
    return rrt;
}

const WaypointPath2D& RRTStarPlanner::get_path() const {
    return shortest_path;
}

Point2D RRTStarPlanner::get_start_point() const{
    return start_point;
}

Point2D RRTStarPlanner::get_target_point() const{
    return target_point;
}

int RRTStarPlanner::build_plan( const Point2D& start, double start_yaw,
                                const Point2D& target, double target_yaw ){
    reset();
    map->seed_Cfree_sampler();
    this->start_point = start;   this->target_point = target;
    this->start_yaw = start_yaw; this->target_yaw = target_yaw;
    //std::cout << "neighbors_factor: " << neighbors_factor << "\n";
    // Add starting point
    rrt.root = rrt.add_node( RRTStarNode( start, -1, 0, 0 ) );
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

void RRTStarPlanner::reset(){
    rrt.clear();
    nodes_index.clear();
    shortest_path.clear();
}

long RRTStarPlanner::expand_to_target(){
    // Find nearest n nodes to target
    double r = params.growth_factor * 1.1;
    Box neighbors_box( target_point - Point2D(r, r), target_point + Point2D(r, r) );
    for( auto p = nodes_index.qbegin( bgi::within(neighbors_box) ) ; p != nodes_index.qend() ; ++p ){
        double step_cost = (p->first - target_point).norm();
        if( step_cost <= params.growth_factor ){
            // Try to connect directly to target
            double step_prob = std::log( step_probability( p->first, target_point ));
            //std::cout << "Candidate step to target: " << step_prob << "\n";
            if ( step_prob > traversability_threshold_log &&
                 abs_angle( *p, target_point ) < params.max_segment_angle / 2.0 &&
                 std::fabs( angle_difference( get_yaw( target_point - p->first ), target_yaw )) < params.max_segment_angle ){
                double branch_prob = rrt.nodes[p->second].probability;
                double branch_cost = rrt.nodes[p->second].cost;
                // Direct connection with target found!
                long idx = rrt.add_node( RRTStarNode( target_point, p->second,
                                                      step_prob + branch_prob, step_cost + branch_cost ));
                nodes_index.insert( std::make_pair( target_point, idx ));
//                std::cout << "Target found and put in node " << idx << "\n";
//                std::cout << "    Probability:  " << std::exp( step_prob + branch_prob ) << "\n";
//                std::cout << "    Cost:  " << step_cost + branch_cost << "\n";
                return idx;
            }
        }
    }

    // No direct connection with target found, expand toward it
    expand_to_point( target_point );
    return -1;
}

void RRTStarPlanner::expand_to_point( const Point2D& to ){
    // Find the new point to be added to the tree
    Point2D new_point = compute_step( nearest(to).first, to );
    if( !in_bounds( new_point ) )
        return;
    // TODO Cercarne non solo il piu vicino ma tra 10 o 20

    // Consider all the nodes in the tree that lie within a box centered to 'new_point'
    // with size computed by neighbors_radius(). Among these nodes, select the one
    // with lower cost.
    double new_branch_cost = std::numeric_limits<double>::max();
    double new_branch_prob = -1;
    double new_step_cost = -1;
    double new_step_prob = -1;
    int new_father_idx = -1;
//  auto nn_query = bgi::satisfies( [&](RTreeStarValue const& v) {return (v->first - to).norm() < 2.0;} );
//    double r = neighbors_radius();
//    if( rrt.nodes.size() % 500 == 1)
//        std::cout << "r: " << r << "\n";
    double r = params.growth_factor * 1.1;
    Box neighbors_box( new_point - Point2D(r, r), new_point + Point2D(r, r) );
    for( auto p = nodes_index.qbegin( bgi::within(neighbors_box) ) ; p != nodes_index.qend() ; ++p ){
        // Test for validity
        if( abs_angle( *p, new_point ) < params.max_segment_angle &&
                !has_similar_sibling( rrt.nodes[p->second], new_point ) ){
            // Test for traversability
            double step_cost = (p->first - new_point).norm();
            double step_prob = std::log( step_probability( p->first, new_point ) );
            if( step_prob > traversability_threshold_log && step_cost <= params.growth_factor * 1.1 ){
                double branch_cost = rrt.nodes[p->second].cost;
                if( branch_cost < new_branch_cost ){
                    new_branch_cost = branch_cost;
                    new_branch_prob = rrt.nodes[p->second].probability;
                    new_step_prob = step_prob;
                    new_step_cost = step_cost;
                    new_father_idx = p->second;
                }
            }
        }
    }

    if( new_father_idx != -1 ){
        // Expand the tree linking the new node with the father with lower cost
        long new_idx = rrt.add_node( RRTStarNode( new_point, new_father_idx,
                                                  new_step_prob + new_branch_prob,
                                                  new_step_cost + new_branch_cost ));
        RTreeStarValue new_node = std::make_pair( new_point, new_idx );
        nodes_index.insert( new_node );

        // Reparenting nodes around the new node to enhance tree quality
//        double r = neighbors_radius();
//        double r = params.growth_factor * 2.1;
//        Box neighbors_box( new_point - Point2D(r, r), new_point + Point2D(r, r) );
//        for( auto p = nodes_index.qbegin( bgi::within(neighbors_box) ) ; p != nodes_index.qend() ; ++p ){
//            if( abs_angle( new_node, p->first ) < params.max_segment_angle &&
//                    !has_similar_sibling( rrt.nodes[new_idx], p->first ) ){
//                // Test for reparentability (only leaves can be reparented)
//                double step_prob = std::log( step_probability( new_point, p->first ) );
//                double step_cost = (p->first - new_point).norm();
//                double branch_cost = rrt.nodes[p->second].cost;
//                if( step_prob > traversability_threshold_log && step_cost < params.growth_factor * 1.1 &&
//                        branch_cost > step_cost + rrt.nodes[new_idx].cost && rrt.nodes[p->second].children.empty() ){
//                    rrt.nodes[p->second].parent = (int)new_idx;
//                    rrt.nodes[p->second].probability = step_prob + rrt.nodes[new_idx].probability ;
//                    rrt.nodes[p->second].cost = step_cost + rrt.nodes[new_idx].cost;
//                }
//            }
//        }
    }
}

void RRTStarPlanner::build_shortest_path( long final_node_idx ){
    shortest_path.clear();
    RRTStarNode n = rrt.nodes[final_node_idx];
    do{
        shortest_path.waypoints.push_back( n.vertex );
        n = rrt.nodes[n.parent];
    }while( n.parent != -1 );
    shortest_path.waypoints.push_back( n.vertex );

    std::reverse( shortest_path.waypoints.begin(), shortest_path.waypoints.end() );
}

Point2D RRTStarPlanner::compute_step( const Point2D& p1, const Point2D& p2 ) const {
    return p1 + params.growth_factor * (p2-p1).normalized();
}

double RRTStarPlanner::neighbors_radius() const {
    double n = rrt.nodes.size() + 2;
    return params.neighbors_factor * sqrt(log(n) / n);
}

//double RRTStarPlanner::compute_neighbors_factor( double map_size_x, double map_size_y) const {
//    double r = Point2D( map_size_x / 2.0, map_size_y / 2.0 ).norm();
//    return (10.0 * r) / M_PI;
//}

double RRTStarPlanner::step_probability( const Point2D& p1, const Point2D& p2 ) const {
    double yaw = get_yaw( p2 - p1 );
    yaw = yaw >= 0 ? yaw : 2.0 * M_PI + yaw;
//    std::cout << "    yaw = " << get_yaw( p2 - p1 ) << " abs(yaw) = " << yaw << "\n";
    return map->traversability_prob( MapOrigin::CENTER_CENTER, p1.x(), p1.y(), yaw );
}

bool RRTStarPlanner::has_similar_sibling( const RRTStarNode& n, const Point2D& to ) const {
    double angle_a = get_yaw( to - n.vertex );
    for( int i: n.children ){
        double angle_b = get_yaw( rrt.nodes[i].vertex - n.vertex );
        if( abs( angle_difference( angle_b, angle_a ) ) < params.max_segment_angle / 2.0 )
            return true;
    }
    return false;
}

const RRTStarPlanner::RTreeStarValue& RRTStarPlanner::nearest( const Point2D& to ) const {
    auto p = nodes_index.qbegin( bgi::nearest(to, 1) );
    if( p != nodes_index.qend() )
        return *p;
    else
        assert( false );
}

double RRTStarPlanner::abs_angle( const RTreeStarValue& n, const Point2D& p2 ) const {
    double angle_a, angle_b;
    if( rrt.nodes[n.second].parent == -1 )
        angle_a = start_yaw; // Root node
    else
        angle_a = get_yaw( n.first - rrt.nodes[rrt.nodes[n.second].parent].vertex ); // Inner node
    angle_b = get_yaw( p2 - n.first );
    return std::fabs( angle_difference( angle_b, angle_a ) );
}

bool RRTStarPlanner::is_traversable( const Point2D& p1, const Point2D& p2 ) const {
    double step_prob = std::log( step_probability( p1, p2 ) );
    return in_bounds( p2 ) && step_prob > traversability_threshold_log;
}

bool RRTStarPlanner::in_bounds( const Point2D &p ) const {
    return (-map->size_x_mt() <= 2.0 * p.x()) &&
           (2.0 * p.x() <= map->size_x_mt()) &&
           (-map->size_y_mt() <= 2.0 * p.y()) &&
           (2.0 * p.y() <= map->size_y_mt());
}

double RRTStarPlanner::angle_between( const Point2D& from, const Point2D& to ){
    return angle_difference( get_yaw( to ), get_yaw( from ) );
}

double RRTStarPlanner::angle_difference( double alpha_1, double alpha_2 ){
    double diff = alpha_1 - alpha_2;
    if( diff < -M_PI )
        diff += 2 * M_PI;
    else if( diff > M_PI )
        diff -= 2 * M_PI;

    return diff;
}

double RRTStarPlanner::get_yaw( const Point2D& p ){
    return std::atan2( p.y(), p.x() );
}

double RRTStarPlanner::distance( const Point2D& p1, const Point2D& p2 ){
    return std::sqrt( (p1.x() - p2.x()) * (p1.x() - p2.x()) +
                      (p1.y() - p2.y()) * (p1.y() - p2.y()) );
}

double RRTStarPlanner::lin_combination( const double a, const double b, const double weight ){
    assert( 0 <= weight && weight <= 1 );
    return a * weight + (1.0 - weight) * b;
}