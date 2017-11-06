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
    rrt_planner.set_map( &(this->map) );
    rrt_planner.set_parameters( 0.5, 100, 5000, M_PI_4, 0.8 );
}

CallResult ReachTargetTask::initialize( int loop_delay_ms ){
    int res = rrt_planner.build_plan( start_pos, start_yaw,
                                      target_pos, target_yaw );
    if( res != 0 )
        return RES_FAIL;
    WaypointPath2D path = rrt_planner.get_path();
    follow_task.reset( new PathFollowingTask( manta, path ) );
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