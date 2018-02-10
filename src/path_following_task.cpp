// path_following_task.cpp
// Author: Giacomo Del Rio
// Date: 17 Apr 2017


#include "path_following_task.h"


#define MAX_LINEAR_VEL 12
#define MAX_ANGULAR_VEL 1.1
#define MAX_LINEAR_ACC 100
#define MAX_ANGULAR_ACC 100
#define LINEAR_PID_GAINS 2, 1.5, 0
#define ANGULAR_PID_GAINS 0.8, 0, 0
#define PATH_BLENDING 0.1
#define INPLACE_ROTATION_THRESHOLD 100


PathFollowingTask::PathFollowingTask( MantaController& manta, WaypointPath2D path )
        : manta(manta){
    path_follower.setLinearPIDGains( LINEAR_PID_GAINS );
    path_follower.setAngularPIDGains( ANGULAR_PID_GAINS );
    path_follower.setFollowingParams( PATH_BLENDING, INPLACE_ROTATION_THRESHOLD );
    path_follower.setMaxVelocities( MAX_LINEAR_VEL, MAX_ANGULAR_VEL );
    path_follower.setMaxAccelerations( MAX_LINEAR_ACC, MAX_ANGULAR_ACC );
    path_follower.setPath( path );
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