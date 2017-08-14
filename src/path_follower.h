// path_follower.h
// Author: Giacomo Del Rio
// Date: 14 Apr 2017

/*
 * Drives a robot along a path defined by waypoints.
 * Uses a PID controller.
 */

#ifndef PATH_FOLLOWER_H
#define PATH_FOLLOWER_H

#include <iostream>
#include <vector>
#include "geometry.h"
#include "pid_controller.h"


class PIDPathFollower
{
  public:
    PIDPathFollower();
    void setPath( const WaypointPath2D& path );
    void setLinearPIDGains( double k_p, double k_i, double k_d );
    void setAngularPIDGains( double k_p, double k_i, double k_d );
    void setMaxVelocities( double maxLinVel, double maxAngVel );
    void setMaxAccelerations( double maxLinAcc, double maxAngAcc );
    void setFollowingParams( double pathBlending, double inPlaceRotationThreshold );

    bool getVelocities( double x, double y, double theta, pidTime now,
                        double& lin_out, double& ang_out );

  private:
    WaypointPath2D path;             // Path to follow
    PIDController linPidController;  // PID controller for linear velocity
    PIDController angPidController;  // PID controller for angular velocity
    double maxLinVel;                // Maximum allowable linear velocity
    double maxAngVel;                // Maximum allowable angular velocity
    double maxLinAcc;                // Maximum allowable linear acceleration
    double maxAngAcc;                // Maximum allowable angular acceleration
    double pathBlending;             // Distance to trigger the next waypoint
    double inPlaceRotationThreshold; // Rotate in place if target angle is bigger than this value

    int currentWaypoint;             // Current target waypoint
    double prevLinVel;               // Previous computed linear velocity
    double prevAngVel;               // Previous computed angular velocity

    int _compute_errors( double x, double y, double theta,
                         double& lin_error, double& ang_error );
    void _limit_velocities( double& lin_vel, double& ang_vel );

    static inline double dst2( double x1, double y1, double x2, double y2 );
    static inline double dst( double x1, double y1, double x2, double y2 );
    static double angleDifference( double a, double b );
};

#endif // PATH_FOLLOWER_H
