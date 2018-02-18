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

#define  PF_PID_INTEGRAL_WINDOW 30


class PIDPathFollower
{
  public:

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
        double angleTolerance;           // Ignore angle errors smaller than this
        double inPlaceRotationThreshold; // Rotate in place if target angle is bigger than this value
        bool   antiLoop;                 // If true, prevents circle loops around a waypoint
        double antiLoopRadius;           // Minimum turning radius of the vehicle
    };

    PIDPathFollower();
    void setPath( const WaypointPath2D& path );
    void setParameters( Parameters params );
    Parameters getParameters();
    bool getVelocities( double x, double y, double theta, pidTime now,
                        double& lin_out, double& ang_out );

  private:
    Parameters params = {            // Default parameters
        .linProportionalGain = 1,
        .linIntegralGain = 0,
        .linDerivativeGain = 0,
        .angProportionalGain = 1,
        .angIntegralGain = 0,
        .angDerivativeGain = 0,
        .maxLinVel = 1,
        .maxAngVel = 1,
        .maxLinAcc = 10,
        .maxAngAcc = 10,
        .pathBlending = 0,
        .angleTolerance = 0.1,
        .inPlaceRotationThreshold = 100,
        .antiLoop = false
    };
    WaypointPath2D path;             // Path to follow
    PIDController linPidController;  // PID controller for linear velocity
    PIDController angPidController;  // PID controller for angular velocity
    int currentWaypoint;             // Current target waypoint
    double prevLinVel;               // Previous computed linear velocity
    double prevAngVel;               // Previous computed angular velocity

    int _compute_errors( double x, double y, double theta,
                         double& lin_error, double& ang_error );
    void _limit_velocities( double& lin_vel, double& ang_vel );
    bool _is_unreachable( const Point2D& waypoint, double x, double y, double theta ) const;

    static inline double dst2( double x1, double y1, double x2, double y2 );
    static inline double dst( double x1, double y1, double x2, double y2 );
    static double angleDifference( double a, double b );
};

#endif // PATH_FOLLOWER_H
