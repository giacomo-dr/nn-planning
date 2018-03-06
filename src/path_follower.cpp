// path_follower.cpp
// Author: Giacomo Del Rio
// Date: 14 Apr 2017

#include "path_follower.h"


PIDPathFollower::PIDPathFollower() :
    linPidController(params.linProportionalGain,
                     params.linIntegralGain,
                     params.linDerivativeGain,
                     PF_PID_INTEGRAL_WINDOW),
    angPidController(params.angProportionalGain,
                     params.angIntegralGain,
                     params.angDerivativeGain,
                     PF_PID_INTEGRAL_WINDOW)
{
    this->currentWaypoint = 0;
    this->prevLinVel = 0;
    this->prevAngVel = 0;
}

void PIDPathFollower::setPath( const WaypointPath2D& path )
{
    this->path = path;
    this->currentWaypoint = 0;
    this->prevLinVel = 0;
    this->prevAngVel = 0;
}

void PIDPathFollower::setParameters( PIDPathFollower::Parameters params ){
    this->params = params;
    linPidController.setGains( params.linProportionalGain,
                               params.linIntegralGain,
                               params.linDerivativeGain );
    angPidController.setGains( params.angProportionalGain,
                               params.angIntegralGain,
                               params.angDerivativeGain );
}

PIDPathFollower::Parameters PIDPathFollower::getParameters(){
    return params;
}

bool PIDPathFollower::getVelocities(
        double x, double y, double theta, pidTime now,
        double& lin_out, double& ang_out )
{
    // Compute errors
    double lin_error, ang_error;
    int progress = _compute_errors( x, y, theta, lin_error, ang_error );
    //std::cout << "PF::Inputs: " << x << ", " << y << ", " << theta << std::endl;
    //std::cout << "PF::Errors: " << lin_error << ", " << ang_error << std::endl;
    //std::cout << "PF::Current Waypoint: " << currentWaypoint << std::endl;

    // Determine linear and angular velocity to issue
    lin_out = linPidController.controlStep( lin_error, now );
    ang_out = angPidController.controlStep( ang_error, now );
    //std::cout << "PF::Velocities: " << lin_out << ", " << ang_out << std::endl;

    // Limit the velocities to the desired max velocity and acceleration
    _limit_velocities( lin_out, ang_out );

    return progress == -1;
}

// Return: -1: finished, n: current waypoint
int PIDPathFollower::_compute_errors( double x, double y, double theta,
                                      double& lin_error, double& ang_error )
{
    lin_error = dst( x, y, path.waypoints[currentWaypoint].x(),
                     path.waypoints[currentWaypoint].y() );

    double target_theta;
    if( lin_error < params.pathBlending ){
        if( currentWaypoint == path.waypoints.size() -1 ){
            // Final waypoint of the path reached
            lin_error = 0.0;
            ang_error = 0.0;
            return -1;
        }else{
            // Current waypoint reached, move to the next one
            currentWaypoint++;
            double target_x = path.waypoints[currentWaypoint].x();
            double target_y = path.waypoints[currentWaypoint].y();
            target_theta = std::atan2( target_y - y, target_x - x );
            lin_error = dst( x, y, target_x, target_y );
        }
    }else{
        //std::cout << "Is unreachable: " << _is_unreachable(path.waypoints[currentWaypoint], x, y, theta) << std::endl;
        if( params.antiLoop && _is_unreachable(path.waypoints[currentWaypoint], x, y, theta) ){
            // std::cout << "Waypoint " << currentWaypoint << " skipped." << std::endl;
            // Current waypoint not reachable, skip it
            if( currentWaypoint == path.waypoints.size() -1 ){
                // End of the path reached
                lin_error = 0.0;
                ang_error = 0.0;
                return -1;
            }else{
                // Move to the next one
                currentWaypoint++;
                double target_x = path.waypoints[currentWaypoint].x();
                double target_y = path.waypoints[currentWaypoint].y();
                target_theta = std::atan2( target_y - y, target_x - x );
                lin_error = dst( x, y, target_x, target_y );
            }
        }else{
            // Current waypoint not already reached, go toward it
            target_theta = std::atan2( path.waypoints[currentWaypoint].y() - y,
                                       path.waypoints[currentWaypoint].x() - x );
        }
    }

    // Ignore small angle errors
    ang_error = angleDifference( target_theta, theta );
    if( fabs(ang_error) < params.angleTolerance )
        ang_error = 0.0;

    // If ang_error is bigger than the threshold, then rotate in place
    if( fabs(ang_error) > params.inPlaceRotationThreshold )
        lin_error = 0.0;

    return currentWaypoint;
}

void PIDPathFollower::_limit_velocities( double& lin_vel, double& ang_vel )
{
    // Limits linear velocity to maximum allowable
    if( lin_vel > params.maxLinVel ){
        lin_vel = params.maxLinVel;
    }

    // Limits angular velocity to maximum allowable
    if( ang_vel > params.maxAngVel ){
        ang_vel = params.maxAngVel;
    }else if( ang_vel < -params.maxAngVel ){
        ang_vel = -params.maxAngVel;
    }

//  // Limits the linear acceleration
//  if( lin_vel - prevLinVel > maxLinAcc )
//    lin_vel = prevLinVel + maxLinAcc;
//  //else if( lin_vel - prevLinVel < - maxLinAcc ) Do not limit breaking
//  //  lin_vel = prevLinVel - maxLinAcc;
//
//  // Limits the linear acceleration
//  if( ang_vel - prevAngVel > maxAngAcc )
//    ang_vel = prevAngVel + maxAngAcc;
//  else if( ang_vel - prevAngVel < - maxAngAcc )
//    ang_vel = prevAngVel - maxAngAcc;
    prevLinVel = lin_vel;
    prevAngVel = ang_vel;
}

bool PIDPathFollower::_is_unreachable( const Point2D& waypoint, double x, double y, double theta ) const{
    // Tells if a waypoint is unreachable given the current position and the
    // cinematic constraints
    Point2D v( std::cos(theta) * params.antiLoopRadius, std::sin(theta) * params.antiLoopRadius );
    Point2D c1( x - v.y(), y + v.x() );
    Point2D c2( x + v.y(), y - v.x() );
    double d1 = dst( c1.x(), c1.y(), waypoint.x(), waypoint.y() );
//    std::cout << "waypoint: (" << waypoint.x() << ", " << waypoint.y() << ")" << std::endl;
//    std::cout << "antiLoopRadius: " << params.antiLoopRadius << std::endl;
//    std::cout << "(x, y, theta): " << x << ", " << y << ", " << theta << std::endl;
//    std::cout << "v: (" << v.x() << ", " << v.y() << ")" << std::endl;
//    std::cout << "c1: (" << c1.x() << ", " << c1.y() << ")" << std::endl;
//    std::cout << "c2: (" << c2.x() << ", " << c2.y() << ")" << std::endl;
//    std::cout << "d1: " << d1 << std::endl;
    if( d1 < params.antiLoopRadius ){
        return true;
    }else{
        double d2 = dst( c2.x(), c2.y(), waypoint.x(), waypoint.y() );
//        std::cout << "d2: " << d2 << std::endl;
        return d2 < params.antiLoopRadius;
    }
}

double PIDPathFollower::dst2( double x1, double y1, double x2, double y2 )
{
    return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

double PIDPathFollower::dst( double x1, double y1, double x2, double y2 )
{
    return std::sqrt( dst2(x1, y1, x2, y2) );
}

double PIDPathFollower::angleDifference( double a, double b )
{
    double diff = a - b;
    if(diff < -M_PI)
        diff += 2 * M_PI;
    else if(diff > M_PI)
        diff -= 2 * M_PI;

    return diff;
}
