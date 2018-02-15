// path_following_task.cpp
// Author: Giacomo Del Rio
// Date: 17 Apr 2017


#include "path_following_task.h"


PathFollowingTask::PathFollowingTask( MantaController& manta, WaypointPath2D& path )
        : manta(manta){
    path_follower.setPath( path );
}

void PathFollowingTask::setFollowerParameters( PIDPathFollower::Parameters follower_params ){
    path_follower.setParameters( follower_params );
}

PIDPathFollower::Parameters PathFollowingTask::getFollowerParameters(){
    return path_follower.getParameters();
}

CallResult PathFollowingTask::initialize( int loop_delay_ms ){
    this->loop_delay_ms = loop_delay_ms;
    manta.register_for_getting_pose( 0 );
    return RES_OK;
}

void PathFollowingTask::finalize(){
    manta.set_velocity( 0.0 );
    manta.set_steer( 0.0 );
    manta.unregister_for_getting_pose();
}

CallResult PathFollowingTask::controlStep( long time_now ){
    Point3D pos = manta.get_position();
    Point3D abg = manta.get_orientation();
    double lin_vel, steer_angle;
    bool finished = path_follower.getVelocities(
            pos(0), pos(1), abg(2) + M_PI_2,  // + M_PI_2 for Manta ugly design (very bad)
            time_now, lin_vel, steer_angle );
    manta.set_velocity( lin_vel );
    manta.set_steer( steer_angle );
    //std::cout << "Manta pos: (" << pos(0) << ", " << pos(1) << ")" << std::endl;
    //std::cout << "Manta ori: (" << abg(2) << ")" << std::endl;
    //std::cout << "Issued vel: " << linAngVel.first << ", " << linAngVel.second << std::endl;
    return finished ? RES_COMPLETED : RES_OK;
}

void PathFollowingTask::abort(){
    finalize();
}