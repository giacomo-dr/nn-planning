// path_following_task.h
// Author: Giacomo Del Rio
// Date: 17 Apr 2017

#ifndef PATH_FOLLOWING_TASK_H
#define PATH_FOLLOWING_TASK_H


#include "robot_task.h"
#include "manta_controller.h"
#include "path_follower.h"


class PathFollowingTask : public RobotTask {

    struct Parameters{
        double linProportionalGain;      // Linear PID proportional gain
        double linIntegralGain;          // Linear PID integral gain
        double linDerivativeGain;        // Linear PID derivative gain
        double angProportionalGain;      // Angular PID proportional gain
        double angIntegralGain;          // Angular PID integral gain
        double angDerivativeGain;        // Angular PID derivative gain
        double maxLinVel;                // Maximum allowable linear velocity
        double maxAngVel;                // Maximum allowable angular velocity
        double maxLinAcc;                // Maximum allowable linear acceleration
        double maxAngAcc;                // Maximum allowable angular acceleration
        double pathBlending;             // Distance to trigger the next waypoint
        double angleTolerance;           // Allowable error in target angle
        double inPlaceRotationThreshold; // Rotate in place if target angle is bigger than this value
        bool   antiLoop;                 // If true, prevents circle loops around a waypoint
    };

public:
    PathFollowingTask( MantaController& manta, WaypointPath2D& path );
    void setFollowerParameters( PIDPathFollower::Parameters follower_params );
    PIDPathFollower::Parameters getFollowerParameters();

    virtual CallResult initialize( int loop_delay_ms );
    virtual void finalize();
    virtual CallResult controlStep( long time_now );
    virtual void abort();

private:
    MantaController& manta;
    PIDPathFollower path_follower;
    int loop_delay_ms;
};


#endif //PATH_FOLLOWING_TASK_H
