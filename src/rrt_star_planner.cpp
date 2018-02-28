// rrt_star_planner.cpp
// Author: Giacomo Del Rio
// Date: 26 Feb 2018


#include <stdexcept>
#include <cmath>
#include <boost/geometry.hpp>
#include "rrt_star_planner.h"

#define PP_DEFAULT_GROWTH_FACTOR 1
#define PP_DEFAULT_GREEDYNESS 10
#define PP_DEFAULT_MAX_ITERATIONS 2000
#define PP_NEIGHBORS_EXPAND 10


long RRTStarPlan::add_node( RRTStarNode n ){
    nodes.push_back(n);
    int node_id = (int)nodes.size() - 1;
    if( n.parent != -1 )
        nodes[n.parent].children.push_back( node_id );
    return node_id;
}

RRTStarPlanner::RRTStarPlanner(){
    this->map = NULL;
    this->growth_factor = PP_DEFAULT_GROWTH_FACTOR;
    this->greediness = PP_DEFAULT_GREEDYNESS;
    this->max_iterations = PP_DEFAULT_MAX_ITERATIONS;
    start_point.setZero();
    target_point.setZero();
    start_yaw = target_yaw = 0;
}

RRTStarPlanner::RRTStarPlanner( HeightMap* map, double growth_factor,
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

void RRTStarPlanner::set_parameters( double growth_factor, unsigned int greediness,
                                     unsigned int max_iterations, double max_segment_angle,
                                     double traversability_threshold ){
    this->growth_factor = growth_factor;
    this->greediness = greediness;
    this->max_iterations = max_iterations;
    this->max_segment_angle = max_segment_angle;
    this->traversability_threshold = log(traversability_threshold);
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

int RRTStarPlanner::build_plan( Point2D start, double start_yaw,
                                Point2D target, double target_yaw ){
    reset();
    this->start_point = start;   this->target_point = target;
    this->start_yaw = start_yaw; this->target_yaw = target_yaw;

    // Add starting point
    rrt.root = rrt.add_node( RRTStarNode( start, -1, 0, 0 ) );
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

void RRTStarPlanner::reset(){
    rrt.clear();
    nodes_index.clear();
    shortest_path.clear();
}

long RRTStarPlanner::expand_to_target(){
    // Find nearest n nodes to target
    Box neighbors_box( target_point - Point2D(1, 1), target_point + Point2D(1, 1) );
    for( auto p = nodes_index.qbegin( bgi::within(neighbors_box) ) ; p != nodes_index.qend() ; ++p ){
        double step_cost = (p->first - target_point).norm();
        if( step_cost <= growth_factor ){
            // Try to connect directly to target
            double step_prob = std::log( step_probability( p->first, target_point ));
            //std::cout << "Candidate step to target: " << step_prob << "\n";
            if ( step_prob > traversability_threshold &&
                 abs_angle( *p, target_point ) < max_segment_angle / 2.0 &&
                 std::fabs( angle_difference( get_yaw( target_point - p->first ), target_yaw )) < max_segment_angle ){
                double branch_prob = rrt.nodes[p->second].probability;
                double branch_cost = rrt.nodes[p->second].cost;
                // Direct connection with target found!
                long idx = rrt.add_node( RRTStarNode( target_point, p->second,
                                                      step_prob + branch_prob, step_cost + branch_cost ));
                nodes_index.insert( std::make_pair( target_point, idx ));
                std::cout << "Target found and put in node " << idx << "\n";
                std::cout << "    Probability:  " << std::exp( step_prob + branch_prob ) << "\n";
                std::cout << "    Cost:  " << step_cost + branch_cost << "\n";
                return idx;
            }
        }
    }

    // No direct connection with target found, expand toward it
    expand_to_point( target_point );
    return -1;
}

void RRTStarPlanner::expand_to_point( const Point2D& to ){
    // Consider all the nodes in the tree that lie within a box centered to the
    // expansion point with size 2mt x 2mt. Among these nodes, select the one
    // with lower cost.
    double min_branch_cost = std::numeric_limits<double>::max();
    double min_step_prob = -1;
    int min_father_idx = -1;
    Point2D min_step;
//  auto nn_query = bgi::satisfies( [&](RTreeStarValue const& v) {return (v->first - to).norm() < 2.0;} );
    Box neighbors_box( to - Point2D(2, 2), to + Point2D(2, 2) );
    for( auto p = nodes_index.qbegin( bgi::within(neighbors_box) ) ; p != nodes_index.qend() ; ++p ){
        if( abs_angle( *p, to ) < max_segment_angle && !has_similar_sibling( rrt.nodes[p->second], to ) ){
            Point2D step = compute_step( p->first, to );
            // Test for traversability
            double step_prob = std::log( step_probability( p->first, to ) );
            if( in_bounds( step ) && step_prob > traversability_threshold ){
                //double branch_prob = rrt.nodes[p->second].probability;
                double branch_cost = rrt.nodes[p->second].cost;
                if( branch_cost < min_branch_cost ){
                    min_step_prob = step_prob;
                    min_branch_cost = branch_cost;
                    min_father_idx = p->second;
                    min_step = step;
                }
            }
        }
    }

    if( min_father_idx != -1 ){
        // Expand the tree linking the new node with the father with lower cost
        long new_idx = rrt.add_node( RRTStarNode( min_step, min_father_idx,
                                              min_step_prob + rrt.nodes[min_father_idx].probability,
                                              growth_factor + min_branch_cost ));
        RTreeStarValue new_node = std::make_pair( min_step, new_idx );
        nodes_index.insert( new_node );

        // Reparenting nodes around the new node to enhance tree quality
        neighbors_box = Box( min_step - Point2D(growth_factor * 1.1, growth_factor * 1.1),
                             min_step + Point2D(growth_factor * 1.1, growth_factor * 1.1) );
        for( auto p = nodes_index.qbegin( bgi::within(neighbors_box) ) ; p != nodes_index.qend() ; ++p ){
            if( abs_angle( new_node, p->first ) < max_segment_angle &&
                    !has_similar_sibling( rrt.nodes[new_idx], p->first ) ){
                // Test for reparentability (only leaves can be reparented)
                double step_prob = std::log( step_probability( min_step, p->first ) );
                double step_cost = (p->first - min_step).norm();
                double branch_cost = rrt.nodes[p->second].cost;
                if( step_prob > traversability_threshold && step_cost < growth_factor * 1.1 &&
                        branch_cost > step_cost + rrt.nodes[new_idx].cost && rrt.nodes[p->second].children.empty() ){
                    //double branch_prob = rrt.nodes[p->second].probability;
                    // TODO Reparent HERE!
                    rrt.nodes[p->second].parent = (int)new_idx;
                    rrt.nodes[p->second].probability = step_prob + rrt.nodes[new_idx].probability ;
                    rrt.nodes[p->second].cost = step_cost + rrt.nodes[new_idx].cost;
                }
            }
        }
    }
}

void RRTStarPlanner::build_shortest_path( long final_node_idx ){
    shortest_path.clear();
    RRTStarNode n = rrt.nodes[final_node_idx];
    do{
//        std::cout << "Pushing (" << n.vertex.x() << ", " << n.vertex.y() << ")\n";
        shortest_path.waypoints.push_back( n.vertex );
        n = rrt.nodes[n.parent];
    }while( n.parent != -1 );
    shortest_path.waypoints.push_back( n.vertex );

    std::reverse( shortest_path.waypoints.begin(), shortest_path.waypoints.end() );
}

Point2D RRTStarPlanner::compute_step( const Point2D& p1, const Point2D& p2 ) const {
    return p1 + growth_factor * (p2-p1).normalized();
}

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
        if( abs( angle_difference( angle_b, angle_a ) ) < max_segment_angle / 2.0 )
            return true;
    }
    return false;
}

double RRTStarPlanner::abs_angle( const RTreeStarValue& n, const Point2D& p2 ) const {
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

bool RRTStarPlanner::is_traversable( const Point2D& p1, const Point2D& p2 ) const {
    double step_prob = std::log( step_probability( p1, p2 ) );
    return in_bounds( p2 ) && step_prob > traversability_threshold;
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