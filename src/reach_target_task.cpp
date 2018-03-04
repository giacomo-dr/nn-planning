// reach_target_task.cpp
// Author: Giacomo Del Rio
// Date: 17 Apr 2017

#include "reach_target_task.h"


ReachTargetTask::ReachTargetTask( MantaController& manta, HeightMap& map,
                                  Point2D start_pos, double start_yaw,
                                  Point2D target_pos, double target_yaw )
        : manta(manta), map(map){
    this->start_pos = start_pos;
    this->target_pos = target_pos;
    this->start_yaw = start_yaw;
    this->target_yaw = target_yaw;
    RRTPlanner::Parameters params = {
            .growth_factor = 0.3,
            .max_segment_angle = M_PI / 6.0,
            .greediness = 10,
            .max_iterations = 5000,
            .traversability_threshold = 0.9,
            .grow_to_point_neighbors = 1
    };
    rrt_planner.set_map( &(this->map) );
    rrt_planner.set_parameters( params );
}

void ReachTargetTask::setRRTParameters( const RRTPlanner::Parameters& params ){
    rrt_planner.set_parameters( params );
}

void ReachTargetTask::setFollowerParameters( const PIDPathFollower::Parameters& params ){
    follower_params = params;
}

CallResult ReachTargetTask::initialize( int loop_delay_ms ){
    int res = rrt_planner.build_plan( start_pos, start_yaw,
                                      target_pos, target_yaw );
    if( res != 0 )
        return RES_FAIL;
    WaypointPath2D path = rrt_planner.get_path();
    follow_task.reset( new PathFollowingTask( manta, path ) );
    follow_task->setFollowerParameters( follower_params );
    return follow_task->initialize( loop_delay_ms );
}

void ReachTargetTask::finalize(){
    follow_task->finalize();
}

CallResult ReachTargetTask::controlStep( long time_now ){
    return follow_task->controlStep( time_now );
}

void ReachTargetTask::abort(){
    follow_task->abort();
}

const RRTPlan& ReachTargetTask::get_plan() const{
    return rrt_planner.get_plan();
}

const WaypointPath2D& ReachTargetTask::get_path() const{
    return rrt_planner.get_path();
}